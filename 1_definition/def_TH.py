# BMP581_reader
import init_GPIO
from smbus2 import SMBus

bus = SMBus(1)
GPIO = init_GPIO.GPIOsetup()

# addr
TH_addr = 0x47
start_data_addr = 0x1D

def THread():
    data = []
    data = bus.read_i2c_block_data(TH_addr, start_data_addr, 6)

    temprature = (data[0] | (data[1] << 8) | (data[2] << 16)) / (2 ** 16)
    pressure = (data[3] | (data[4] << 8) | (data[5] << 16)) / (2 ** 6)

    return temprature, pressure
