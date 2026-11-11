import init_GPIO
GPIO = init_GPIO.GPIOsetup()
from smbus2 import SMBus


def read_GNSS(bus):
    message = b""
