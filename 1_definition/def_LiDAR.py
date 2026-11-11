import init_GPIO
GPIO = init_GPIO.GPIOsetup()

#LiDAR_reader
# ----------------------------------------
# Plz refer to -> https://en.benewake.com/uploadfiles/2024/04/20240426140053930.pdf
# ----------------------------------------
from smbus2 import SMBus, i2c_msg
import time

LiDAR_addr = 0x10
bus = SMBus(1)


def LiDARread():
    try:
        #Obtain Data Frame in 9bytes-cm
        command = [0x5A, 0x05, 0x00, 0x01, 0x60]

        write_data = i2c_msg.write(LiDAR_addr, command)
        bus.i2c_rdwr(write_data)
        
        time.sleep(0.001)
        
        read_data = i2c_msg.read(LiDAR_addr, 9)
        bus.i2c_rdwr(read_data)

        data = list(read_data)
    
    except Exception:
        return None
        
    if len(data) != 9:
        return None
        
    if data[0] != 0x59 or data[1] != 0x59:
        return None
        
    checksum = sum(data[0:8]) & 0xFF
        
    if checksum != data[8]:
        return None
        
    # Extract distance and strength values
    distance = data[2] + data[3] * 256
    strength = data[4] + data[5] * 256
        
    # Filter out invalid readings
    if (distance == 65535 or distance == 65534 or distance == 65532 or strength < 100):
        return None
    else:
        return distance
    