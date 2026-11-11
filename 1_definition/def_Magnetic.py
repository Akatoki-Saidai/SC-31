import init_GPIO
GPIO = init_GPIO.GPIOsetup()

import smbus2
import time
import math
import numpy as np

# 1. 定数の定義
I2C_ADDRESS = 0x0E
I2C_BUS = 1

# センサの磁気感度 [uT/LSB]、磁気偏角 [deg]、測定周期 [s] を定義する
MAG_SENSITIVITY = 0.042
DECLINATION_DEG = -6.5
CALIBRATION_TIME = 15.0
SAMPLE_INTERVAL = 0.05

bus = smbus2.SMBus(I2C_BUS)

try:
    # 2. 接続確認とセンサの初期化
    device_id = bus.read_byte_data(I2C_ADDRESS, 0x0F)
    if device_id != 0x41:
        raise RuntimeError("BM1422AGMVは検出されませんでした")
    else:
        bus.write_byte_data(I2C_ADDRESS, 0x1B, 0xD0)
        bus.write_byte_data(I2C_ADDRESS, 0x5C, 0x00)
        bus.write_byte_data(I2C_ADDRESS, 0x5D, 0x00)
        bus.write_byte_data(I2C_ADDRESS, 0x1C, 0x0C)
        bus.write_byte_data(I2C_ADDRESS, 0x1D, 0x40)
        time.sleep(0.01)

    # 3. データ読み取り関数
    def read_mag():
        data = bus.read_i2c_block_data(I2C_ADDRESS, 0x10, 6)
        values = [
            int.from_bytes(bytes(data[i:i+2]), "little", signed = True)
            for i in range(0, 6, 2)
        ]
        return tuple(v * MAG_SENSITIVITY for v in values)

    # 4. キャリブレーション関数
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

    # 5. 方位角計算関数
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

    # 6. メインループ
    mag_offset, mag_scale = calibrate_mag()
    
    while True:
        current_time = time.monotonic()

        mx, my, mz = read_mag()
        heading = calculate_heading(mx, my, mz, mag_offset, mag_scale)
        print(f"Heading:{heading:.2f} °")
        time.sleep(max(0, SAMPLE_INTERVAL - (time.monotonic() - current_time)))
    
except KeyboardInterrupt:
    print("終了します")
finally:
    bus.close()