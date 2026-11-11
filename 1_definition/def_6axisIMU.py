import init_GPIO
GPIO = init_GPIO.GPIOsetup()


import smbus2
import time
import math
import numpy as np

# I2C settings
I2C_ADDRESS = 0x68
I2C_BUS = 1

# Measurement phase
PHASE = "descent"  # descent: during free fall, landed: after landing

# Sensor data conversion settings
ACCEL_SCALE = 8192.0
GYRO_SCALE = 131.0

# Detection conditions
TILT_THRESHOLD = 30.0
ABNORMAL_TIME = 2.0
EXPECTED_Z_SIGN = 1
MIN_ACCEL = 0.75
MAX_ACCEL = 1.25
SAMPLE_INTERVAL = 0.01


# Quaternion attitude estimator
class QuaternionAttitudeEstimator:

    def __init__(self):

        # Initialize quaternion [q0, q1, q2, q3]
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def update(self, gyro, dt):
        """
        Update quaternion using gyroscope data.

        gyro: Angular velocity [rad/s] (x, y, z)
        dt: Time interval [s]
        """

        # Calculate angular velocity magnitude
        w_norm = np.linalg.norm(gyro)
        if w_norm > 1e-10:

            # Calculate rotation axis and angle
            rotation_axis = gyro / w_norm
            rotation_angle = w_norm * dt

            # Create rotation quaternion
            dq = np.array([
                np.cos(rotation_angle / 2),
                *(rotation_axis * np.sin(rotation_angle / 2))
            ])

            # Update and normalize quaternion
            self.q = self._quaternion_multiply(self.q, dq)
            self.q /= np.linalg.norm(self.q)

    @staticmethod
    def _quaternion_multiply(q1, q2):
        q10, q11, q12, q13 = q1
        q20, q21, q22, q23 = q2

        return np.array([
            q10*q20 - q11*q21 - q12*q22 - q13*q23,
            q10*q21 + q11*q20 + q12*q23 - q13*q22,
            q10*q22 - q11*q23 + q12*q20 + q13*q21,
            q10*q23 + q11*q22 - q12*q21 + q13*q20
        ])

    def to_euler(self):
        """Convert quaternion to Euler angles."""
        q0, q1, q2, q3 = self.q

        # Calculate roll
        roll = np.arctan2(
            2 * (q0*q1 + q2*q3),
            1 - 2 * (q1*q1 + q2*q2)
        )

        # Calculate pitch
        sin_pitch = 2 * (q0*q2 - q3*q1)
        pitch = np.arcsin(np.clip(sin_pitch, -1.0, 1.0))

        # Convert radians to degrees
        return np.degrees(roll), np.degrees(pitch)

# Start I2C communication
bus = smbus2.SMBus(I2C_BUS)

try:

    # Check the sensor connection
    device_id = bus.read_byte_data(I2C_ADDRESS, 0x75)
    if device_id != 0x42:
        raise RuntimeError("ICM-42605 is not detected")

    # Initialize the sensor
    bus.write_byte_data(I2C_ADDRESS, 0x4E, 0x0F)
    time.sleep(0.05)
    bus.write_byte_data(I2C_ADDRESS, 0x4F, 0x68)
    bus.write_byte_data(I2C_ADDRESS, 0x50, 0x48)
    time.sleep(0.1)

    # Read acceleration and angular velocity
    def read_imu():
        data = bus.read_i2c_block_data(I2C_ADDRESS, 0x1F, 12)
        values = [
            int.from_bytes(bytes(data[i:i+2]), "big", signed=True)
            for i in range(0, 12, 2)
        ]

        # Convert acceleration to g and angular velocity to deg/s
        return (
            *[v / ACCEL_SCALE for v in values[:3]],
            *[v / GYRO_SCALE for v in values[3:]]
        )

    # Calibrate gyroscope offset
    def calibrate_gyro(samples=200):
        total = [0.0, 0.0, 0.0]
        print("Calibrating gyroscope. Keep the device still.")
        for _ in range(samples):
            _, _, _, gx, gy, gz = read_imu()
            for i, value in enumerate([gx, gy, gz]):
                total[i] += value
            time.sleep(0.01)
        print("Calibration completed")
        return [value / samples for value in total]

    # Initialize quaternion estimator
    attitude = QuaternionAttitudeEstimator()

    # Calculate roll and pitch using quaternion
    def calculate_attitude(gx, gy, gz, bias, dt):

        # Correct gyroscope offset
        gx -= bias[0]
        gy -= bias[1]
        gz -= bias[2]

        # Convert angular velocity from deg/s to rad/s
        gyro = np.radians([gx, gy, gz])

        # Update quaternion
        attitude.update(gyro, dt)

        # Convert quaternion to roll and pitch
        return attitude.to_euler()

    # Time when abnormal condition starts
    abnormal_start = None

    # Check gravity direction
    def check_gravity_direction(ax, ay, az, now):
        global abnormal_start

        # Calculate acceleration magnitude
        accel_total = math.sqrt(ax**2 + ay**2 + az**2)

        # Calculate tilt angle from the Z-axis
        if accel_total > 0.001:
            cos_theta = EXPECTED_Z_SIGN * az / accel_total
            cos_theta = max(-1.0, min(1.0, cos_theta))
            tilt_angle = math.degrees(math.acos(cos_theta))
        else:
            tilt_angle = 180.0

        # Check normal condition
        normal = (
            MIN_ACCEL <= accel_total <= MAX_ACCEL
            and tilt_angle < TILT_THRESHOLD
        )

        if normal:
            abnormal_start = None
            return True, tilt_angle

        # Record the start time of abnormal condition
        if abnormal_start is None:
            abnormal_start = now

        # Return False if abnormal condition lasts 2 seconds
        return now - abnormal_start < ABNORMAL_TIME, tilt_angle

    # Main program
    gyro_bias = calibrate_gyro()

    print("Starting IMU measurement")

    previous_time = time.monotonic()
    freefall_detected = False

    while True:

        current_time = time.monotonic()
        dt = current_time - previous_time
        previous_time = current_time

        # Read sensor data
        ax, ay, az, gx, gy, gz = read_imu()

        # Calculate acceleration magnitude
        accel_total = math.sqrt(ax**2 + ay**2 + az**2)

        # Calculate roll and pitch angles
        current_roll, current_pitch = calculate_attitude(
            gx, gy, gz, gyro_bias, dt
        )

        # During descent phase
        if PHASE == "descent":

            # Detect free fall
            if accel_total < MIN_ACCEL:
                freefall_detected = True

            # Detect landing after free fall
            if freefall_detected and MIN_ACCEL <= accel_total <= MAX_ACCEL:
                PHASE = "landed"
                print("Landing detected")
            else:
                print(
                    f"Roll: {current_roll:.2f} deg, "
                    f"Pitch: {current_pitch:.2f} deg"
                )

        # After landing
        elif PHASE == "landed":

            status, tilt_angle = check_gravity_direction(
                ax, ay, az, current_time
            )
            print(
                f"Gravity: {status}, "
                f"Tilt: {tilt_angle:.2f} deg"
            )

        # Repeat measurements at regular intervals
        time.sleep(max(
            0,
            SAMPLE_INTERVAL - (time.monotonic() - current_time)
        ))
except KeyboardInterrupt:
    print("Program terminated")
finally:

    # Close I2C communication
    bus.close()
