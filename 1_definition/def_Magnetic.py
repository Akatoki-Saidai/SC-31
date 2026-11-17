# BM1422AGMV_reader
# ----------------------------------------
# Plz refer to -> https://akizukidenshi.com/goodsaffix/BM1422AGMV.pdf
# ----------------------------------------

from smbus2 import SMbus

# def I2C address
I2C_ADDRESS = 0x0E

class BM:
    def __init__(self, i2cbus)
        self.bus = i2cbus
        device_id = bus.read_byte_data(I2C_ADDRESS, 0x0F)
        if device_id != 0x41:
            raise RuntimeError("BM1422AGMV is not detected")
        else:
            # CTRL_SETTING1 -> OUTPUT_DATA_BIT : 14bit, ODR : 100Hz
            bus.write_byte_data(I2C_ADDRESS, 0x1B, 0xC8)
            # SENSOR_ACTIVATE
            bus.write_byte_data(I2C_ADDRESS, 0x5C, 0x00)
            bus.write_byte_data(I2C_ADDRESS, 0x5D, 0x00)
            # CTRL_SETTING2 -> DRDY_CONF : activate, AD_converter_STATUS : start
            bus.write_byte_data(I2C_ADDRESS, 0x1C, 0x0C)
            bus.write_byte_data(I2C_ADDRESS, 0x1D, 0x40)
    def read():
        data = bus.read_i2c_block_data(I2C_ADDRESS, 0x10, 6)
        values = [
            int.from_bytes(bytes(data[i:i+2]), "little", signed = True)
            for i in range(0, 6, 2)
        ]
        return [v * MAG_SENSITIVITY for v in values]

'''
import time
import math
import numpy as np

# def sensitivity [uT/LSB] magnetic declination [deg] mesurement rate [s]
MAG_SENSITIVITY = 0.042
DECLINATION_DEG = -6.5
CALIBRATION_TIME = 15.0
INTERVAL = 0.05

# calibration
def calibrate_mag():
    min_vals = np.array([float("inf"), float("inf"), float("inf")])
    max_vals = np.array([float("-inf"), float("-inf"), float("-inf")])

    print("キャリブレーションを開始します。センサをゆっくりと回転させてください。")
    start_time = time.monotonic()

    while time.monotonic() - start_time < CALIBRATION_TIME:
        mx, my, mz = read_mag()
        current = np.array([mx, my, mz])
        min_vals = np.minimum(min_vals, current)
        max_vals = np.maximum(max_vals, current)
        time.sleep(0.02)

    offset = (min_vals + max_vals) / 2.0
    radii = (max_vals - min_vals) / 2.0
    scale = np.mean(radii) / radii
    
    print("キャリブレーション完了")
    return offset, scale

# calculate azimath
def calculate_heading(mx, my, mz, offset, scale, roll_deg = 0.0, pitch_deg = 0.0):
    mx_calibrated = (mx - offset[0]) * scale[0]
    my_calibrated = (my - offset[1]) * scale[1]
    mz_calibrated = (mz - offset[2]) * scale[2]

    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)

    hx = mx_calibrated * math.cos(pitch_rad) + my_calibrated * math.sin(roll_rad) * math.sin(pitch_rad) + mz_calibrated * math.cos(roll_rad) * math.sin(pitch_rad)
    hy = my_calibrated * math.cos(roll_rad) - mz_calibrated * math.sin(roll_rad)

    heading_rad = math.atan2(-hy, hx)
    heading_deg = math.degrees(heading_rad) + DECLINATION_DEG
    heading_deg = (heading_deg) % 360
    return heading_deg

# roop
mag_offset, mag_scale = calibrate_mag()

while True:
    current_time = time.monotonic()

    mx, my, mz = read_mag()
    heading = calculate_heading(mx, my, mz, mag_offset, mag_scale)
    print(f"Heading:{heading:.2f} °")
    time.sleep(max(0, INTERVAL - (time.monotonic() - current_time)))
'''