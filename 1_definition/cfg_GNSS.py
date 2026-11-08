# import init_GPIO
# GPIO = init_GPIO.GPIOsetup()
from smbus2 import i2c_msg
import time

device_address = 0x42 # I2c address of the GNSS module
# print("device_address_type: ",type(device_address))
# print("device_address: ",device_address)

num_bytes_addr = 0xfd
message_addr = 0xff

### calculate checksum ###
def calculate_checksum(data):
    Buffer = data[2:]
    # print("Buffer: ",Buffer.hex(" "))
    CK_A = 0
    CK_B = 0

    for i in range(len(Buffer)):
        # print("Buffer[i]: ",Buffer[i])
        CK_A = (CK_A + Buffer[i]) & 0xFF
        CK_B = (CK_B + CK_A) & 0xFF

    CK = bytes([CK_A,CK_B])
    return CK
########################

##### ~begin~ check for message available on GNSS module #####
def check_message(bus):
    cnt =0
    message = b""
    ack_ack = bytes.fromhex("b56205010200068a98c1")
    ack_nak = bytes.fromhex("b56205000200068a97bc")

    while cnt<10:

        write = i2c_msg.write(device_address,[num_bytes_addr])
        read = i2c_msg.read(device_address,2)
        bus.i2c_rdwr(write,read)
        result = list(read)
        length = result[0]<<8 | result[1]

        if length ==0: 
            print("No message available. ")
            cnt +=1
            time.sleep(0.1)
            continue
        else:
            write = i2c_msg.write(device_address,[message_addr])
            read = i2c_msg.read(device_address, length)
            bus.i2c_rdwr(write,read)
            message += bytes(read)

            if ack_ack in message:
                print("UBX-ACK-ACK Message has received.")  
                return 1
            elif ack_nak in message:
                print("UBX-ACK-NAK Message has received.")  
                return -1
            else:
                cnt +=1
                time.sleep(0.1)

    return 0

    
##### ~end~ check for message available on GNSS module #####

### ~begin~ make ubx_cfg_valset message ###

PLfirst4bytes__ubx_cfg_valset = bytes([0x00,0x01,0x00,0x00])

cfg_i2coutprot_nmea = bytes([0x02,0x00,0x72,0x10]) # disable NMEA output on I2C
cfg_msgout_ubx_nav_pvt_i2c = bytes([0x06,0x00,0x91,0x20]) # output NAV_PVT for all positioning solutions

cfgData = cfg_i2coutprot_nmea + bytes([0x00]) + cfg_msgout_ubx_nav_pvt_i2c + bytes([0x01])
## make payload for ubx_cfg_valset message
PL__ubx_cfg_valset = PLfirst4bytes__ubx_cfg_valset + cfgData
## calculate payload length
PLlen_ubx_cfg_valset = len(PL__ubx_cfg_valset).to_bytes(2,byteorder='little')
# print("PLlen_ubx_cfg_valset: ",PLlen_ubx_cfg_valset)

## add header, payloadlength and payload to ubx_cfg_valset
ubx_cfg_valset = bytes([0xB5,0x62,0x06,0x8a])+PLlen_ubx_cfg_valset + PL__ubx_cfg_valset

## calculate checksum for ubx_cfg_valset
CK_ubx_cfg_valset = calculate_checksum(ubx_cfg_valset)
# print("CK_ubx_cfg_valset: ",CK_ubx_cfg_valset.hex(" "))

## update ubx_cfg_valset with checksum
ubx_cfg_valset = ubx_cfg_valset + CK_ubx_cfg_valset

# print("cfgData: ",cfgData.hex(" "))    
# print("cfgData Length: ",len(cfgData))

# print("PL__ubx_cfg_valset: ",PL__ubx_cfg_valset.hex(" "))
# print("PL__ubx_cfg_valset Length: ",len(PL__ubx_cfg_valset))

# print("ubx_cfg_valset: ",ubx_cfg_valset.hex(" "))
# print("ubx_cfg_valset Length: ",len(ubx_cfg_valset))

### ~end~ make ubx_cfg_valset message ###

### ~begin~ GNSS Configuration ###

def configure_gnss(bus):
    write = i2c_msg.write(device_address,ubx_cfg_valset)
    bus.i2c_rdwr(write)

    flag = check_message(bus)
    return flag

### ~end~ GNSS Configuration ###
        




    


