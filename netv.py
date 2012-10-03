import posix


''' Basterdized from the quick2wire project '''
from contextlib import closing
import posix
from fcntl import ioctl
from ctypes import create_string_buffer, sizeof, c_int, byref, pointer, addressof, string_at
from ctypes import c_int, c_uint16, c_ushort, c_short, c_ubyte, c_char, POINTER, Structure
import struct
#
# Converted from i2c.h and i2c-dev.h
# I2C only, no SMB definitions

# /usr/include/linux/i2c-dev.h: 38
class i2c_msg(Structure):
    """<linux/i2c-dev.h> struct i2c_msg"""
    
    _fields_ = [
        ('addr', c_uint16),
        ('flags', c_ushort),
        ('len', c_short),
        ('buf', POINTER(c_char))]
    
    __slots__ = [name for name,type in _fields_]



# i2c_msg flags
I2C_M_TEN		= 0x0010	# this is a ten bit chip address
I2C_M_RD		= 0x0001	# read data, from slave to master
I2C_M_NOSTART		= 0x4000	# if I2C_FUNC_PROTOCOL_MANGLING
I2C_M_REV_DIR_ADDR	= 0x2000	# if I2C_FUNC_PROTOCOL_MANGLING
I2C_M_IGNORE_NAK	= 0x1000	# if I2C_FUNC_PROTOCOL_MANGLING
I2C_M_NO_RD_ACK		= 0x0800	# if I2C_FUNC_PROTOCOL_MANGLING
I2C_M_RECV_LEN		= 0x0400	# length will be first received byte


# /usr/include/linux/i2c-dev.h: 155
class i2c_rdwr_ioctl_data(Structure):
    """<linux/i2c-dev.h> struct i2c_rdwr_ioctl_data"""
    _fields_ = [
        ('msgs', POINTER(i2c_msg)),
        ('nmsgs', c_int)]

    __slots__ = [name for name,type in _fields_]

I2C_FUNC_I2C			= 0x00000001
I2C_FUNC_10BIT_ADDR		= 0x00000002
I2C_FUNC_PROTOCOL_MANGLING	= 0x00000004 # I2C_M_NOSTART etc.


# ioctls

I2C_SLAVE	= 0x0703	# Change slave address			
				# Attn.: Slave address is 7 or 10 bits  
I2C_SLAVE_FORCE	= 0x0706	# Change slave address			
				# Attn.: Slave address is 7 or 10 bits  
				# This changes the address, even if it  
				# is already taken!			
I2C_TENBIT	= 0x0704	# 0 for 7 bit addrs, != 0 for 10 bit	
I2C_FUNCS	= 0x0705	# Get the adapter functionality         
I2C_RDWR	= 0x0707	# Combined R/W transfer (one stop only) 


class I2CMaster:
    def __init__(self, n=0):
        self.fd = posix.open("/dev/i2c-%i"%n, posix.O_RDWR)
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
    
    def close(self):
        posix.close(self.fd)
    
    def transaction(self, *msgs):
        msg_count = len(msgs)
        msg_array = (i2c_msg*msg_count)(*msgs)
        ioctl_arg = i2c_rdwr_ioctl_data(msgs=msg_array, nmsgs=msg_count)
        
        ioctl(self.fd, I2C_RDWR, addressof(ioctl_arg))
        
        return [i2c_msg_to_bytes(m) for m in msgs if (m.flags & I2C_M_RD)]

def i2c_read(addr, n_bytes):
    return _new_i2c_msg(addr, I2C_M_RD, create_string_buffer(n_bytes))

def i2c_write(addr, byte):
    return _new_i2c_msg(addr, 0, create_string_buffer(struct.pack("B", byte), 1))

def _new_i2c_msg(addr, flags, buf):
    #print sizeof(buf), repr(buf.raw)
    return i2c_msg(addr=addr, flags=flags, len=sizeof(buf), buf=buf)

def i2c_msg_to_bytes(m):
    return string_at(m.buf, m.len)







''' NeTv FPGA interface '''
class fpga(object):
	DEVADDR = 0x3C
	FPGA_DNA_ADR = 0x38 
	FPGA_MAJOR_ADR = 0x3f

	def __init__(self):
		self._fpga_filename = "/dev/fpga"
		self._fpga_file = open(self._fpga_filename, 'rb')
		
		self._i2c_bus = I2CMaster()
		
	def __del__(self):
		if self._fpga_file:
			self._fpga_file.close()
	
	def deviceId(self):
		deviceId = ""
	
		for i in range(7):
			read_results = self._i2c_bus.transaction(
				i2c_write(self.DEVADDR >> 1, self.FPGA_DNA_ADR + i),
				i2c_read(self.DEVADDR >> 1, 1))
				
			deviceId += "%02x" % ord(read_results[0])
			
		return deviceId

	def version(self):		
		read_results = self._i2c_bus.transaction(
					i2c_write(self.DEVADDR >> 1, self.FPGA_MAJOR_ADR),
					i2c_read(self.DEVADDR >> 1, 1))
				
		return ord(read_results[0])
		
	def dump_registers(self):	
		print ""
		for n in range(8):
			myBytes = [0,0,0,0]
			for i in range(4):	
				read_results = self._i2c_bus.transaction(
						i2c_write(self.DEVADDR >> 1, (n * 4) + i),
						i2c_read(self.DEVADDR >> 1, 1))
						
				myBytes[i] = read_results[0]
				
			print "0x%02x: %02x %02x %02x %02x" % ((n*4), ord(myBytes[0]), ord(myBytes[1]), ord(myBytes[2]), ord(myBytes[3]))
		print ""