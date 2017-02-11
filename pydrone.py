#	ARGE-D pydrone
#	
#	Tab Width:4
#
#	RaspberryPi
#	I2C
#	|
#	|---(0x40)> PCA9685(Pwm Controller) ---> ESC (PWM Control) ---> Motors
#	| 
#	|---> MPU9255(Sensor Gyro)
#
# PCA9685
#      ____________
#  ____|           |____
# 0    ^(on)       ^(off)   65535
#        _______________
# _______|
from __future__ import division

# to use sleep function
import time
import math
# to shift bits
import ctypes

# Logging
import logging
logger = logging.getLogger(__name__)


# In order to control i2c bus, we will use smbus2 module
from smbus2 import *


# PCA9685 8bit Registers:
PCA9685_ADDRESS		= 0x40
MODE1				= 0x00
MODE2				= 0x01
SUBADR1				= 0x02
SUBADR2				= 0x03
SUBADR3				= 0x04
PRESCALE			= 0xFE

LED0_ON_L			= 0x06
LED0_ON_H			= 0x07
LED0_OFF_L			= 0x08
LED0_OFF_H			= 0x09

ALL_LED_ON_L		= 0xFA
ALL_LED_ON_H		= 0xFB
ALL_LED_OFF_L		= 0xFC
ALL_LED_OFF_H		= 0xFD

# PCA9685 Bits:
RESTART				= 0x80
SLEEP				= 0x10
ALLCALL				= 0x01
INVRT				= 0x10
OUTDRV				= 0x04


class PCA9685(object):

	#PCA9685 Constructor
	def __init__(self, bus, address=PCA9685_ADDRESS):

		# Setup I2C interface for the device.
		self._device = bus
		self._addr = address

		#Initialize the PCA9685		
		self.set_all_pwm(0, 0)
		self._device.write_byte_data(self._addr,MODE2, OUTDRV)
		self._device.write_byte_data(self._addr,MODE1, ALLCALL)
		time.sleep(0.005)  # wait for oscillator
		mode1 = self._device.read_byte_data(self._addr,MODE1)
		mode1 = mode1 & ~SLEEP  # wake up (reset sleep)
		self._device.write_byte_data(self._addr,MODE1, mode1)
		time.sleep(0.005)  # wait for oscillator

	def set_pwm_freq(self, freq_hz):
		# Set the PWM frequency to the provided value in hertz.
		self.__freq=freq_hz
		prescaleval = 25000000.0	# 25MHz
		prescaleval /= 4096.0		# 12-bit
		prescaleval /= float(freq_hz)
		prescaleval -= 1.0
		logger.debug('Setting PWM frequency to {0} Hz'.format(freq_hz))
		logger.debug('Estimated pre-scale: {0}'.format(prescaleval))
		prescale = int(math.floor(prescaleval + 0.5))
		logger.debug('Final pre-scale: {0}'.format(prescale))
		oldmode = self._device.read_byte_data(self._addr,MODE1);
		newmode = (oldmode & 0x7F) | 0x10	# sleep
		self._device.write_byte_data(self._addr,MODE1, newmode)  # go to sleep
		self._device.write_byte_data(self._addr,PRESCALE, prescale)
		self._device.write_byte_data(self._addr,MODE1, oldmode) # return back
		time.sleep(0.005)
		self._device.write_byte_data(self._addr,MODE1, oldmode | 0x80)

	def set_pwm(self, channel, on, off):
		"""Sets a single PWM channel."""
		self._device.write_byte_data(self._addr,LED0_ON_L+4*channel, on & 0xFF)
		self._device.write_byte_data(self._addr,LED0_ON_H+4*channel, on >> 8)
		self._device.write_byte_data(self._addr,LED0_OFF_L+4*channel, off & 0xFF)
		self._device.write_byte_data(self._addr,LED0_OFF_H+4*channel, off >> 8)

	def set_all_pwm(self, on, off):
		"""Sets all PWM channels."""
		self._device.write_byte_data(self._addr,ALL_LED_ON_L, on & 0xFF)
		self._device.write_byte_data(self._addr,ALL_LED_ON_H, on >> 8)
		self._device.write_byte_data(self._addr,ALL_LED_OFF_L, off & 0xFF)
		self._device.write_byte_data(self._addr,ALL_LED_OFF_H, off >> 8)

	def set_gas(self,gas):
		self.set_all_pwm(0, int(((gas/100+1)/1000*self.__freq)*4095))
	def go(self,gas,proportion):
		for i in range(4):
			self.set_pwm(12+i, 0, int(((gas*proportion[i]/100+1)/1000*self.__freq)*4095))


class mpu9255:
	def __init__(self, bus):
		self.__addr=0x68
		self.__bus=bus
		self.__bus.write_byte_data(self.__addr,0x6B,0x00)
		self.__bus.write_byte_data(self.__addr,0x19,0x07)
		self.__bus.write_byte_data(self.__addr,0x1A,0x18)
		self.__bus.write_byte_data(self.__addr,0x1B,0x18)
		self.__bus.write_byte_data(self.__addr,0x1C,0x01)
		if(0x73!=self.__bus.read_byte_data(self.__addr,0x75)):
			print("MPU ERROR")
		buffer = self.__bus.read_i2c_block_data(self.__addr,0x3B,6)
		self.__acc= [ctypes.c_int16(buffer[2*i]*16*16+buffer[2*i+1]).value/32768 for i in range(3)]
	def readAcceleration(self):
		buffer = self.__bus.read_i2c_block_data(self.__addr,0x3B,6)
		return [ctypes.c_int16(buffer[2*i]*16*16+buffer[2*i+1]).value/32768*math.pi-self.__acc[i] for i in range(3)]
	def readGyroscope(self):
		buffer = self.__bus.read_i2c_block_data(self.__addr,0x43,6)
		return [buffer[2*i]*16*16+buffer[2*i+1] for i in range(3)]
	def readCompass(self):
		buffer = 0
		return buffer
	def readTemperature(self):
		buffer = self.__bus.read_i2c_block_data(self._addr,0x41,2)
		return buffer[0]<<8+buffer[1]


class PID:
	def __init__(self,kp,ki,kd):
		self.kp=kp
		self.ki=ki
		self.kd=kd
		self.__I=0
		self.__lastError=0
	def pid(self,error,t):
		self.__I+=t*error
		d=error-self.lastError
		lastError=error
		return self.kp*error+self.ki*self.__I+self.kd*d/t


class drone:
	def __init__(self):
		x=SMBus(1)
		self.__motors=PCA9685(x)
		self.__sensor=mpu9255(x)
		self.__motors.set_pwm_freq(120)
		self.__motors.set_gas(0)
		time.sleep(0.5)
		self.__balance=PID(0.5,0.2,0)
	def loop(self, x, y):	
		while True:
			period=0.1
			time.sleep(period)
			degree=[0,0]
			acceleration=[0,0]
			distance=[0,0,0]
			for i in range(10):
				degree = [(x + y)/2 for x, y in zip(degree, self.__sensor.readAcceleration()[0:2])]
							
			#distance =[x + y*period**2/2 for x, y in zip(distance, self.__sensor.readGyroscope()]
			buffer=[0.707106*(degree[0]-degree[1]),
					0.707106*(degree[0]+degree[1])]
			m=[]			
			for i in buffer:
				m.append(self.balance.pid(-1*i,0.1))			
			buffer=[1+m[1],1+m[0],1-m[1],1-m[0]]	
			for i in range(4):
				if(buffer[i]>1):
					buffer[i]=1			
			self.__motors.go(40,buffer)
	def stop(self):
		self.__motors.set_gas(0)


a=drone()
#a.loop()
