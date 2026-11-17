# BMP581_reader
# ----------------------------------------
# Plz refer to -> https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf
# ----------------------------------------
from smbus2 import SMBus

# addr
TH_addr = 0x47
DATA_first_addr = 0x1D
OSR_addr = 0x36 # oversampling register address
ODR_addr = 0x37 # output data rate register address

# config
PRS_OVERSAM = 0x7 # oversampling
TEMP_OVERSAM = 0x0
# 0x0: 1x, 0x1: 2x, 0x2: 4x,
# 0x3: 8x, 0x4: 16x, 0x5: 32x,
# 0x6: 64x, 0x7: 128x
ODR = 0x17

PWR_mode = 0x01
# 0x00: Standby, 0x01: Normal, 0x02: Forced, 0x03: Non-stop

class BMP:
    def __init__(self, i2cbus):
        self.bus = i2cbus
        OSR_CONFIG = (PRS_OVERSAM << 3) | TEMP_OVERSAM | (0b01 << 6)
        ODR_CONFIG = PWR_mode | (0b0 << 7) | (ODR << 2)
        bus.write_byte_data(TH_addr, OSR_addr, OSR_CONFIG)
        bus.write_byte_data(TH_addr, ODR_addr, ODR_CONFIG)
    def read():
        data = []
        data = bus.read_i2c_block_data(TH_addr, DATA_first_addr, 6)

        temprature = (data[0] | (data[1] << 8) | (data[2] << 16)) / (2 ** 16)
        pressure = (data[3] | (data[4] << 8) | (data[5] << 16)) / (2 ** 6)

        return temprature, pressure
