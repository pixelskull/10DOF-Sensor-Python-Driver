#!/usr/bin/env python3

from smbus import SMBus		# pip install smbus 
import serial 				# pip install pyserial
import math
import time
import numpy as np

address2 = 0x53
address3 = 0x68
address4 = 0x77

# bus = SMBus(1) # for RasberryPi Model B+ (Model A = 0)


class Bus_Hepler_i2c():
	def __init__(self, bus_location=1):
		self.bus = SMBus(bus_location)
		
	def write_byte(self, address, register, byte):
		self.bus.write_i2c_block_data(address, register, [byte])
		
	def read_block_data(self, address, cmd): 
		return self.bus.read_i2c_block_data(address, cmd)
		
	
class MagnetometerValues(): 
	def __init__(self, x, y, z):
		self.x = x
		self.y = y 
		self.z = z 

	
class Magnetometer:
	
	def __init__(self):
		self.address = 0x1e # Magnetometer address on I2C Bus
		self.scale = 1.0
		self.conf_register1 = 0x00
		self.conf_register2 = 0x01
		self.bus = Bus_Hepler_i2c()
		
	def set_scale(self, gauss=1.3):
		# setup all register addresses for applied values
		reg_pattern = {
			0.88 : 0x00,
			1.3  : 0x01,
			1.9  : 0x02,
			2.5  : 0x03,
			4.0  : 0x04,
			4.7  : 0x05,
			5.6  : 0x06,
			8.1  : 0x07
		}
		# setup all scale factors for applied values
		scale_pattern = {
			0.88 : 0.73,
			1.3  : 0.92,
			1.9  : 1.22,
			2.5  : 1.52,
			4.0  : 2.27,
			4.7  : 2.56,
			5.6  : 3.03,
			8.1  : 4.35
		}
		# check if given value is in 'range'
		if gauss not in reg_pattern.keys() or gauss not in scale_pattern.keys():
			print("gauss Value has to be in this list [0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1]")
			exit() 
		# set values 
		self.scale = scale_pattern[gauss]
		reg_value = reg_pattern[gauss]
		reg_value = reg_value << 5
		# write to i2c bus 
		self.bus.write_byte(self.address, self.conf_register2, reg_value)
	
	def write_continuous_command(self):
		# 0x02 ist register for reading cmd and 0x00 is continuos reading 
		self.bus.write_byte(self.address, 0x02, 0x00) 
	
	def read_magnetometer_buffer(self, begin=3, length=6):
		tmp_block = self.bus.read_block_data(self.address, 0x00) # get buffer block 
		
		buffer = []
		end = begin + length
		# read relevant data from block 
		for index in range(begin, end):
			buffer.append(np.int8(tmp_block[index]))
		return buffer
		
	def read_raw_axis_data(self): 
		buffer = self.read_magnetometer_buffer()
		# fancy stuff to get the correct values from i2c
		x = (buffer[0] << 8) | buffer[1]
		y = (buffer[4] << 8) | buffer[5]
		z = (buffer[2] << 8) | buffer[3]
		
		raw_data = MagnetometerValues(x, y, z) # setup helper class with raw values
		return raw_data
		
	def read_scaled_axis_data(self, scale=0.92):
		raw_axis = self.read_raw_axis_data()
		# scale data to given value 
		x = float(raw_axis.x * scale)
		y = float(raw_axis.y * scale)
		z = float(raw_axis.z * scale)
		
		scaled_axis = MagnetometerValues(x, y, z) # setup helper class with scaled values
		return scaled_axis
		
	def get_heading(self):
		scaled_axis = self.read_scaled_axis_data()
		heading = math.atan2(scaled_axis.y, scaled_axis.x)

		# TODO: find right declination angle for location 		
# 		declinationAngle = 0.0457
# 		heading += declinationAngle

		double_pi = 2.0 * math.pi
		# handle wrap due addition of declination
		if heading > double_pi:
			heading -= double_pi
		# handle reversed signs
		if heading < 0:# 
			heading += double_pi
		# Convert radians to degrees 
		heading = heading * (180 / math.pi)
		
		return int(heading)




# read this as tutorial then uncomment the main stuff 	
def main():
	# setup magnetometer
	mag = Magnetometer()
	mag.set_scale(1.3)
	mag.write_continuous_command() # setup magnetometer for continuous measurement
	
	while True:
		heading = mag.get_heading()
		print( "Magnetometer------" )
		print( "heading: ", heading)
		print( "------Magnetometer" )
		
		time.sleep(0.1) # don´t remove this 
	
	
if __name__ == "__main__":
    main()
