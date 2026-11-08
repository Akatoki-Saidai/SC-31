import init_GPIO
GPIO = init_GPIO.GPIOsetup()

#LiDAR_reader
import serial
import time

def read_LiDAR():
    ser = serial.Serial("/dev/serial0",115200,timeout = 0.1)
    date = ser.read(9)

    distance = date[2] + date[3] * 256
    strength = date[4] + date[5] * 256

    if distance == 65535 or strength < 100 or distance == 65534 or distance == 65532:
        return None
    else:
        return distance