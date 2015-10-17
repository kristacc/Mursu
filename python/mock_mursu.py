#!/usr/bin/env python

import struct, time

def crc( viesti ):
	crc = 0xFFFF
	for ch in viesti:
		crc ^= ord(ch)
		for i in range( 8 ):
			if (crc & 0x0001) != 0:
				crc >>= 1
				crc ^= 0xA001
			else:
				crc >>= 1
	
	return crc


class Serial( object ):
	def __init__( self, *args, **kwargs ):
		self.buffer = ""
		self.address = 250
		self.t0 = time.time()
		self.output = ""
	
	def write( self, data ):
		t1 = time.time()
		if t1-self.t0 > 1e-3:
			self.buffer = ""
		
		self.buffer += data
		
		try:
			self.process()
		except:
			pass
		
		self.t0 = time.time()
	
	def read( self, N ):
		out = self.output[:N]
		self.output = self.output[N:]
		
		return out
	
	def close( self ):
		pass	
	
	def error_response( self, func, code ):
		msg = struct.pack( "BBB", self.address, func | 0x80, code )
		msg += struct.pack( "<H", crc( msg ) )
		
		self.output = msg
	
	def process( self ):
		#print repr(self.buffer)
		if len( self.buffer ) <= 6:
			return
		
		# check address
		address = struct.unpack_from("B", self.buffer )[0]
		
		if address != self.address:
			self.buffer = ""
			return
		
		
		
		func_code = struct.unpack_from( "B", self.buffer, 1 )[0]
		
		if func_code != 0x03:
			self.error_response( func_code, 0x01 )
			self.buffer = ""
			return
		
		mem_address = struct.unpack_from( ">H", self.buffer, 2 )[0]
		if (mem_address < 1000) or (mem_address > 1010):
			self.error_response( func_code, 0x02 )
			self.buffer = ""
			return
		
		amount = struct.unpack_from( ">H", self.buffer, 4 )[0]
		if mem_address + amount > 1010:
			self.error_response( func_code, 0x02 )
			self.buffer = ""
			return
		
		values = [mem_address - 1000 + i + 1 for i in range( amount ) ]
		msg = struct.pack( "BBB", self.address, 0x03, len(values)*2 )
		for value in values:
			msg += struct.pack( ">H", value )
		
		msg += struct.pack( "<H", crc( msg ) )
		
		self.output = msg
		self.buffer = ""
		

## tmp

#com = Serial()
#
#msg = struct.pack( ">BBHH", 42, 0x03, 1007, 5 )
#msg += struct.pack( "<H", crc(msg) )
#
#com.write( msg )
#print repr(com.read(1000))
