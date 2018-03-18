import serial
import mock_mursu
import struct

READ_HOLDING_REGISTERS = 0x03
WRITE_SINGLE_REGISTER = 0x06
CHANGE_DEVICE_ADDRESS = 0xCAFE

def crc(message):
    crc = 0xFFFF
    for ch in message:
        crc ^= ord(ch)
        for i in range( 8 ):
            if (crc & 0x0001) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def open_port(port,baudrate,timeout):
    try:
        com = serial.Serial(port,baudrate,timeout=timeout)
        return com
    except serial.serialutil.SerialException as se:
        print str(se)
        return None

def open_and_return_local_mursu_port():
    com = mock_mursu.Serial()
    return com

def close(port):
    port.close()

def read_holding_register(port,address,register,amount):

    print "Device address:"
    print address
    print "Command:"
    print READ_HOLDING_REGISTERS
    print "register to read:"
    print register
    print "Amount:"
    print amount
    msg = struct.pack( ">BBHH",address,READ_HOLDING_REGISTERS,register,amount)
    checksum = crc( msg )
    msg += struct.pack( "<H", checksum )
    port.write(msg)
    print "We wrote: "
    print_message(msg)
    response = port.read(1024)
    print "We read:"
    print_response(response)
    return response

def write_single_register(port,address,register,amount):
    msg = struct.pack( ">BBHH", mursu_address,
                       WRITE_SINGLE_REGISTER, register, amount)
    checksum = crc( msg )
    msg += struct.pack( "<H", checksum )
    port.write(msg)
    response = port.read(1024)
    return response

def print_message(message):
    print "message format: | address | function code | register | amount | crc |"
    print "->",["%02x" % ord(x) for x in message ]

def print_response(response):
    print "message format: | address | function code | byte count | register contents | crc |"
    print "<-", ["%02x" % ord(x) for x in response ]

def change_device_address(port,old_address,new_address):
    write_single_register(port,old_address,CHANGE_DEVICE_ADDRESS,new_address)
