import serial
import crc
import struct

READ_HOLDING_REGISTERS = 0x03
WRITE_SINGLE_REGISTER = 0x06
CHANGE_DEVICE_ADDRESS = 0xCAFE

def open_port(port=6,baudrate=38400,timeout = 0.1)
    com = serial.Serial(port,baudrate,timeout)
    com.open()
    return com

def close(port)
    port.close()

def read_holding_register(port,address,register,amount):
    msg = struct.pack( ">BBHH",address,
                       READ_HOLDING_REGISTERS,register,amount)
    checksum = crc.laske_crc( msg )
    msg += struct.pack( "<H", checksum )
    port.write(msg)
    response = port.read(1024)
    return response

def write_single_register(port,address,register,amount):
    msg = struct.pack( ">BBHH", mursu_address,
                       WRITE_SINGLE_REGISTER, register, amount)
    checksum = crc.laske_crc( msg )
    msg += struct.pack( "<H", checksum )
    port.write(msg)
    response = port.read(1024)
    return response

def print_message(message):
    print "->",["%02x" % ord(x) for x in message ]

def print_response(response):
    print "<-", ["%02x" % ord(x) for x in response ]

def change_device_address(port,old_address,new_address):
    write_single_register(port,old_address,CHANGE_DEVICE_ADDRESS,new_address)

