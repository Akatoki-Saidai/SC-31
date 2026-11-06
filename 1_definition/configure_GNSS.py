import init_GPIO
GPIO = init_GPIO.GPIOsetup()
from smbus2 import SMBus

with SMBus(1) as bus:
    pl__ubx_cfg_valset = bytes([0x00,0x01,0x00,0x00])
    
    cfg_i2coutprot_nmea = bytes([0x02,0x00,0x72,0x10]) # disable NMEA output on I2C
    cfg_msgout_ubx_nav_pvt_i2c = bytes([0x06,0x00,0x91,0x20]) # output NAV_PVT for all positioning solutions
    cfdData = bytes([cfg_i2coutprot_nmea,0x00,cfg_msgout_ubx_nav_pvt_i2c,0x01])
    
    pass