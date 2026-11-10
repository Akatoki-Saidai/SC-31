# import init_GPIO
# GPIO = init_GPIO.GPIOsetup()
from smbus2 import i2c_msg,SMBus
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

def check_checksum(data):
    Buffer = data[:-2]
    CS=calculate_checksum(Buffer)
    if CS == data[-2:]:
        return 1 
    else:
        return 0

#####  check for message available on GNSS module #####
def check_message(bus):
    cnt =0
    message = b""
    ack_ack = bytes.fromhex("b56205010200068a98c1")
    ack_nak = bytes.fromhex("b56205000200068a97bc")

    while cnt<30:

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

    

###  make ubx_cfg_valset message ###
def make_ubx_cfg_valset():

    PLfirst4bytes__ubx_cfg_valset = bytes([0x00,0x01,0x00,0x00])

    cfg_i2coutprot_nmea = bytes([0x02,0x00,0x72,0x10]) # disable NMEA output on I2C
    cfg_msgout_ubx_nav_pvt_i2c = bytes([0x06,0x00,0x91,0x20]) # output NAV_PVT for all positioning solutions

    cfgData = cfg_i2coutprot_nmea + bytes([0x00]) + cfg_msgout_ubx_nav_pvt_i2c + bytes([0x01])

    PL__ubx_cfg_valset = PLfirst4bytes__ubx_cfg_valset + cfgData# make payload for ubx_cfg_valset message

    PLlen_ubx_cfg_valset = len(PL__ubx_cfg_valset).to_bytes(2,byteorder='little')# calculate payload length
    # print("PLlen_ubx_cfg_valset: ",PLlen_ubx_cfg_valset)

    ubx_cfg_valset = bytes([0xB5,0x62,0x06,0x8a])+PLlen_ubx_cfg_valset + PL__ubx_cfg_valset# add header, payloadlength and payload to ubx_cfg_valset

    CK_ubx_cfg_valset = calculate_checksum(ubx_cfg_valset)# calculate checksum for ubx_cfg_valset
    # print("CK_ubx_cfg_valset: ",CK_ubx_cfg_valset.hex(" "))

    ubx_cfg_valset = ubx_cfg_valset + CK_ubx_cfg_valset# update ubx_cfg_valset with checksum

    # print("cfgData: ",cfgData.hex(" "))    
    # print("cfgData Length: ",len(cfgData))

    # print("PL__ubx_cfg_valset: ",PL__ubx_cfg_valset.hex(" "))
    # print("PL__ubx_cfg_valset Length: ",len(PL__ubx_cfg_valset))

    # print("ubx_cfg_valset: ",ubx_cfg_valset.hex(" "))
    # print("ubx_cfg_valset Length: ",len(ubx_cfg_valset))
    return ubx_cfg_valset


###  GNSS Configuration ###
def configure_gnss(bus):
    ubx_cfg_valset = make_ubx_cfg_valset()
    write = i2c_msg.write(device_address,ubx_cfg_valset)
    bus.i2c_rdwr(write)

    flag = check_message(bus)
    return flag

### make test data for checking ubx_nav_pvt packet reading function is OK###
def make_ubx_nav_pvt_testdata():
    header_ubx_nav_pvt = bytes.fromhex("b56201075c00")
    # print("header_ubx_nav_pvt is: ",header_ubx_nav_pvt.hex(" "))
    payload = bytes(92)
    # print("the payload of test_ubx_nav_pvt is: ",payload.hex(" "))
    ubx_nav_pvt_testdata = header_ubx_nav_pvt + payload
    # print("test data without Checksum is :",ubx_nav_pvt_testdata.hex(" "))
    CS = calculate_checksum(ubx_nav_pvt_testdata)
    # print("check sum is : ",CS.hex(" "))
    ubx_nav_pvt_testdata += CS
    # print("the test_ubx_nav_pvt is: ",ubx_nav_pvt_testdata.hex(" "))
    return ubx_nav_pvt_testdata

def read_GNSS(bus):
    message = b""

    write = i2c_msg.write(device_address,[num_bytes_addr])
    read = i2c_msg.read(device_address,2)
    bus.i2c_rdwr(write,read)
    result = list(read)
    length = result[0]<<8 | result[1]

    if length ==0:
        return message
    else:
        write = i2c_msg.write(device_address,[message_addr])
        read = i2c_msg.read(device_address, length)
        bus.i2c_rdwr(write,read)
        message = bytes(read)
        return message





def find_ubx_nav_pvt(message):
    header_ubx_nav_pvt = bytes.fromhex("b56201075c00")
    index=None
    if len(message) <6:
        print("message length is too short.")
        return "incomplete",message,b""
    else:
        for i in range(len(message)-5):
            if message[i:i+6] == header_ubx_nav_pvt:
                index = i
                break
            else:
                continue

        if index is None:
            print("ubx_nav_pvt header is not found")
            return "no-header",message[-5:],b""
        elif len(message)-index >= 100:
            packet_data = message[index:index+100]
            checksum = calculate_checksum(packet_data[:-2])
            if checksum == packet_data[-2:]:
                print("clean ubx_nav_pvt is found!")
                pending = message[index+100:]
                return  "complete", pending,packet_data
            else:
                print("check sum is not right.")
                return "checksum_error",message[index+1:],b""
        if len(message)-index < 100:
            message = message[index:]
            print("a part of ubx_nav_pvt is found!")
            return "incomplete", message,b""
    
def collect_ubx_nav_pvt(bus,pending=b""):
    cnt = 0
    message = pending
    while cnt <30:
        flag,message,packetdata= find_ubx_nav_pvt(message)
        if flag == "complete":
            print("ubx_nav_pavt packet is full.")
            return message,packetdata
        if flag =="checksum_error":
            continue

        else:
            message += read_GNSS(bus)
            cnt +=1
            time.sleep(0.1)

    flag,message,packetdata= find_ubx_nav_pvt(message)
    if flag == "complete":
        print("ubx_nav_pavt packet is full.")
        return message,packetdata
    
    print("ubx_nav_pvt reception time out.")
    return message,None
            
            



    


if __name__ == "__main__":
    # test_message = bytes(3)+make_ubx_nav_pvt_testdata()
    # flag,frag_data,packetdata = find_ubx_nav_pvt(test_message)
    # print("flag is : ",flag,"\nfrag_data is : ",frag_data.hex(" "),"\npacket data is : ",packetdata.hex(" ")) 

    # short_test_message = test_message[:-10]
    # flag,frag_data,packetdata = find_ubx_nav_pvt(short_test_message)
    # print("flag is : ",flag,"\nfrag_data is : ",frag_data.hex(" "),"\npacket data is : ",packetdata.hex(" ")) 

    busNum = 1
    with SMBus(busNum) as bus:
        flag=configure_gnss(bus)
        if flag ==1:
            message, packetdata = collect_ubx_nav_pvt(bus,pending=b"")

            if packetdata != None:
                print("ubx_nav_pvt packet is received.")
                print("length of packet is : ",len(packetdata))
            else:
                print("ubx_nav_pvt acquisition is failed.")

        else: 
            print("GNSS configuration is failed.",flag)









