# In VS Code, imports will appear

import board
import busio
import storage
import analogio
import digitalio
import time as clock
import microcontroller

import adafruit_gps
import adafruit_sdcard
import adafruit_bme680
import adafruit_max1704x
from adafruit_lis3mdl import LIS3MDL
import adafruit_pcf8591.pcf8591 as PCF
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_pcf8591.analog_in import AnalogIn
from adafruit_pcf8591.analog_out import AnalogOut
from adafruit_datetime import datetime, date, time
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219

import random


try:
	
	###################################################################
	#                                                                 #
	#                     CC BY-NC-SA 4.0 License                     #
	#                                                                 #
	#   Except where otherwise noted, this code is licensed under a   #
	# Attribution-NonCommercial-ShareAlike 4.0 International License. #
	#                                                                 #
	###################################################################

	###################################################################################################################

	callsign = "monkeyPox" #[CHANGE BEFORE LAUNCH] Sets the callsign for the device (Line one in Configuration.txt)
	beaconInterval = 5 #Controls the interval of beacon signal (Line two in Configuration.txt)

	measurementInterval = 5 #Controls interval of data collection (Line three in Configuration.txt)
	maxDataNumber = 100000 #Maximum number of data measurements stored on the SD card (Line four in Configuration.txt)

	maxErrorNumber = 100 #Maximum number of error messages stored on the SD card (Line five in Configuration.txt)

	debugLevel = 1;#Defines the level of debugging required, from high level very important logs all the way down to the minutia
	# ranges from 1 -> 5, 1 being minimal logging, 5 being advanced logging
	###################################################################################################################

	#Establishes data pins
	I2C0 = busio.I2C(
		board.GP1,
		board.GP0,
		frequency=10000
	)

	SPI1 = busio.SPI(
		board.GP10,
		MOSI=board.GP11,
		MISO=board.GP12
	)	

	CS1 = digitalio.DigitalInOut(board.GP13)

	UART0 = busio.UART(
		board.GP16,
		board.GP17,
		baudrate=9600,
		timeout=10
	)

	UART1 = busio.UART(
		board.GP8,
		board.GP9,
		baudrate=9600,
		timeout=0.1
	)

	# Sets up analog pins (may read non-zero value when not connected due to noise)
	GPIO26 = analogio.AnalogIn(board.GP26)
	GPIO27 = analogio.AnalogIn(board.GP27)
	GPIO28 = analogio.AnalogIn(board.GP28)

	#Established LED Outputs
	imbeddedLed = digitalio.DigitalInOut(board.GP25)
	imbeddedLed.direction = digitalio.Direction.OUTPUT

	processLed = digitalio.DigitalInOut(board.GP21)
	processLed.direction = digitalio.Direction.OUTPUT

	gpsLed = digitalio.DigitalInOut(board.GP22)
	gpsLed.direction = digitalio.Direction.OUTPUT

	transmitLed = digitalio.DigitalInOut(board.GP20)
	transmitLed.direction = digitalio.Direction.OUTPUT

	receiveLed = digitalio.DigitalInOut(board.GP18)
	receiveLed.direction = digitalio.Direction.OUTPUT

	errorLed = digitalio.DigitalInOut(board.GP19)
	errorLed.direction = digitalio.Direction.OUTPUT

	#Established LoRa Outputs
	M0 = digitalio.DigitalInOut(board.GP6)
	M0.direction = digitalio.Direction.OUTPUT
	M0.value = False

	M1 = digitalio.DigitalInOut(board.GP7)
	M1.direction = digitalio.Direction.OUTPUT
	M1.value = False

	#Defines sensors globally
	transceiver = None
	gps = None
	bme = None
	imu = None
	mag = None
	powerDraw = None
	solarOne = None
	solarTwo = None
	solarThree = None
	battery = None
	adc = None

	#Defines constants for sensor calibration
	temperature_offset = 0 #Each temperature sensor can be different, calibrate to a know temperature if possible
	vbus_reference_voltage = 5 #4.5V when powered by battery, 5V when powered by USB
	
	battery_detection_threshold = 2.5 #Determines when the battery is connected or not

	gpsStatus = True #Stores status of GPS connection for the GPS LED

	#Defines SD and storage card globally and sets dataNumber
	sdcard = None
	dataNumber = -1 #Line one in StateBackup.txt

	#Defines the sequence of beacon data globally
	beaconSequence = 0 #Line two in StateBackup.txt

	#Defines the errorNumber counter globally
	errorNumber = -1 #Line three in StateBackup.txt

	currentLED = 0;

	#Defines GPS object
	class GPS: 
			
			#GPS __init__
			def __init__(self, Latitude, Longitude, Altitude, Speed, Satellites, DOP):
					
					self.Latitude = Latitude
					self.Longitude = Longitude
					self.Altitude = Altitude
					self.Speed = Speed
					self.Satellites = Satellites
					self.DOP = DOP
					if(debugLevel >= 3): 
						print("GPS Setup Complete")
						if(debugLevel >= 5):
							print("\n"+self)
			
			#Defines how GPS will be printed
			def __repr__(self):
					
					try:
					
							return "<Lat: {:.6f} degrees, Long: {:.6f} degrees, Altitude: {:.2f} m, Speed: {:.2f} m/s, Satellites: {}, DOP: {}>".format(
									self.Latitude,
									self.Longitude,
									self.Altitude,
									self.Speed,
									self.Satellites,
									self.DOP
							)
					
					except:
							if(debugLevel >= 5):
								print("GPS Data Printing Failed")
							return "<Lat: {} degrees, Long: {} degrees, Altitude: {} m, Speed: {} m/s, Satellites: {}, DOP: {}>".format(
									self.Latitude,
									self.Longitude,
									self.Altitude,
									self.Speed,
									self.Satellites,
									self.DOP
							)

	#Defines ALTIMETER object
	class ALTIMETER:
			
			#ALTIMETER __init__
			def __init__(self, Altitude, Temperature, Pressure, Humidity, Gas):
					
					self.Altitude = Altitude
					self.Temperature = Temperature
					self.Pressure = Pressure
					self.Humidity = Humidity
					self.Gas = Gas
					if(debugLevel >= 3): 
						print("Altimeter Setup Complete")
						if(debugLevel >= 5):
							print("\n"+self)
			
			#Defines how ALTIMETER will be printed
			def __repr__(self):
					
					try:
							
							return "<Altitude: {:.2f} m, Temperature: {:.2f} C, Pressure: {:.2f} kPa, Humidity {:.2f} %, Gas: {:.0f} ohms>".format(
									self.Altitude,
									self.Temperature,
									self.Pressure,
									self.Humidity,
									self.Gas
							)
			
					except:
							
							if(debugLevel >= 5):
								print("GPS Data Printing Failed")
							return "<Altitude: {} m, Temperature: {} C, Pressure: {} kPa, Humidity {} %, Gas: {} ohms>".format(
									self.Altitude,
									self.Temperature,
									self.Pressure,
									self.Humidity,
									self.Gas
							)

	#Defines ACCELEROMETER object
	class ACCELEROMETER:
			
			#ACCELEROMETER __init__
			def __init__(self, X, Y, Z):
					
					self.X = X
					self.Y = Y
					self.Z = Z
					if(debugLevel >= 3): 
						print("Accelerometer Setup Complete")
						if(debugLevel >= 5):
							print("\n"+self)
			
			#Defines how ACCELEROMETER will be printed
			def __repr__(self):
					
					try:
					
							return "<X-Axis Acceleration: {:.2f} m/s^2, Y-Axis Acceleration: {:.2f} m/s^2, Z-Axis Acceleration: {:.2f} m/s^2>".format(
									self.X,
									self.Y,
									self.Z
								)

					except:
							if(debugLevel >= 5):
								print("GPS Data Printing Failed")
							return "<X-Axis Acceleration: {} m/s^2, Y-Axis Acceleration: {} m/s^2, Z-Axis Acceleration: {} m/s^2>".format(
									self.X,
									self.Y,
									self.Z
								)

	#Defines GYROSCOPE object
	class GYROSCOPE:
			
			#GYROSCOPE __init__
			def __init__(self, X, Y, Z):
					
					self.X = X
					self.Y = Y
					self.Z = Z	
					if(debugLevel >= 3): 
						print("Gyroscope Setup Complete")
						if(debugLevel >= 5):
							print("\n"+self)
			
			#Defines how GYROSCOPE will be printed
			def __repr__(self):
					
					try:
							
							return "<X-Axis Rotational Velocity: {:.2f} radians/s, Y-Axis Rotational Velocity: {:.2f} radians/s, Z-Axis Rotational Velocity: {:.2f} radians/s>".format(
									self.X,
									self.Y,
									self.Z
								)

					except:
							if(debugLevel >= 5):
								print("GPS Data Printing Failed")
							return "<X-Axis Rotational Velocity: {} radians/s, Y-Axis Rotational Velocity: {} radians/s, Z-Axis Rotational Velocity: {} radians/s>".format(
									self.X,
									self.Y,
									self.Z
								) 

	#Defines MAGNETOMETER object  
	class MAGNETOMETER:
			
			#MAGNETOMETER __init__
			def __init__(self, X, Y, Z):
					
					self.X = X
					self.Y = Y
					self.Z = Z
					if(debugLevel >= 3): 
						print("Magnometer Setup Complete")
						if(debugLevel >= 5):
							print("\n"+self)
			
			#Defines how MAGNETOMETER will be printed
			def __repr__(self):
					
					try:
					
							return "<X-Axis Field Strength: {:.2f} uT, Y-Axis Field Strength: {:.2f} uT, Z-Axis Field Strength: {:.2f} uT>".format(
									self.X,
									self.Y,
									self.Z
								)

					except:
							
							if(debugLevel >= 5):
								print("Magnometer Data Printing Failed")
							return "<X-Axis Field Strength: {} uT, Y-Axis Field Strength: {} uT, Z-Axis Field Strength: {} uT>".format(
									self.X,
									self.Y,
									self.Z
								)

	#Defines POWER object
	class POWER:
			
			#POWER __init__
			def __init__(self, voltage, current, wattage):
					
					self.Voltage = voltage
					self.Current = current
					self.Wattage = wattage
					if(debugLevel >= 3): 
						print("Power Setup Complete")
						if(debugLevel >= 5):
							print("\n"+self)
			
			#Defines how POWER will be printed
			def __repr__(self):
					
					try:
							
							return "<Voltage: {:.3} V, Current: {:.3} A, Wattage: {:.3} W>".format(
									self.Voltage,
									self.Current,
									self.Wattage
							)

					except:
							if(debugLevel >= 5):
								print("Power Data Printing Failed")
							return "<Voltage: {} V, Current: {} A, Wattage: {} W>".format(
									self.Voltage,
									self.Current,
									self.Wattage
							)

	#Defines BATTERY object
	class BATTERY:
			
			#BATTERY __init__
			def __init__(self, voltage, percentage):
					
					self.Voltage = voltage
					self.Percentage = percentage
					if(debugLevel >= 3): 
						print("Battery Setup Complete")
						if(debugLevel >= 5):
							print("\n"+self)
			
			#Defines how BATTERY will be printed
			def __repr__(self):
					
					try:
							
							return "<Voltage: {:.2f} V, Percentage: {:.2f} %>".format(
									self.Voltage,
									self.Percentage
								)
					
					except:
							if(debugLevel >= 5):
								print("Battery Data Printing Failed")
							return "<Voltage: {} V, Percentage: {} %>".format(
									self.Voltage,
									self.Percentage
								)

	#Defines SOLARPANEL object
	class SOLARPANEL:
			
			#SOLARPANEL __init__
			def __init__(self, one, two, three):
					
					self.One = one
					self.Two = two
					self.Three = three
					if(debugLevel >= 3): 
						print("Solar Panel Setup Complete")
						if(debugLevel >= 5):
							print("\n"+self)
			
			#Defines how SOLARPANEL will be printed
			def __repr__(self):
				
					return "Solar Panel Power Output One: {},\nSolar Panel Power Output Two: {},\nSolar Panel Power Output Three:{}".format(
							self.One,
							self.Two,
							self.Three
						)

	#Defines ANALOG object
	class ANALOG:
			
			#ANALOG __init__
			def __init__(self, A0, A1, A2, A3, A4, A5, A6):
					
					self.A0 = A0
					self.A1 = A1
					self.A2 = A2
					self.A3 = A3
					self.A4 = A4
					self.A5 = A5
					self.A6 = A6
					if(debugLevel >= 3): 
						print("Analog Setup Complete")
						if(debugLevel >= 5):
							print("\n"+self)
					
			#Defines how ANALOG will be printed
			def __repr__(self):
					
					try:
							
							return "<A0: {:.4f} V, A1: {:.4f} V, A2: {:.4f} V, A3: {:.4f} V, A4: {:.4f} V, A5: {:.4f} V, A6: {:.4f} V>".format(
									self.A0,
									self.A1,
									self.A2,
									self.A3,
									self.A4,
									self.A5,
									self.A6
								)

					except:
							
							if(debugLevel >= 5):
								print("Analog Data Printing Failed")
							return "<A0: {} V, A1: {} V, A2: {} V, A3: {} V, A4: {} V, A5: {} V, A6: {} V>".format(
									self.A0,
									self.A1,
									self.A2,
									self.A3,
									self.A4,
									self.A5,
									self.A6
								)

	#Defines AVIONICSDATA object
	class AVIONICSDATA:
			
			#AVIONICSDATA __init__
			def __init__(self, TIME, GPS, ALTIMETER, ACCELEROMETER, GYROSCOPE, MAGNETOMETER, POWERDRAW, SOLARPANEL, BATTERY, ANALOG):
					
					self.TIME = TIME
					self.GPS = GPS
					self.ALTIMETER = ALTIMETER
					self.ACCELEROMETER = ACCELEROMETER
					self.GYROSCOPE = GYROSCOPE
					self.MAGNETOMETER = MAGNETOMETER
					self.POWERDRAW = POWERDRAW
					self.SOLARPANEL = SOLARPANEL
					self.BATTERY = BATTERY
					self.ANALOG = ANALOG
					if(debugLevel >= 3): 
						print("Avionics Data Setup Complete")
						if(debugLevel >= 5):
							print("\n"+self)
			
			#Defines how AVIONICSDATA will be printed
			def __repr__(self):
					return "{}\nGPS: {},\nAltimeter: {},\nAccelerometer: {},\nGyroscope: {},\nMagnetometer: {},\nSystem Power Draw: {},\n{},\nBattery: {},\nAnalog: {}".format(
							self.TIME,
							self.GPS,
							self.ALTIMETER,
							self.ACCELEROMETER,
							self.GYROSCOPE,
							self.MAGNETOMETER,
							self.POWERDRAW,
							self.SOLARPANEL,
							self.BATTERY,
							self.ANALOG
						)

	#getTime() handles the multiple ways the current time can be acquired
	def getTime():
			
			global gpsStatus
			
			try:
					
					gpsStatus = True
					
					#Attempts to get time from GPS
					return getGpsTime()
					
			except:
					
					try:
							
							gpsStatus = False
							
							#Attempts to get time from monotonic clock
							if(debugLevel >= 3):
								print("\n[Using Monotonic Time]\n")
							return round(clock.monotonic())
					
					except Exception as e:
							
							errorCode(3, e)
							
							return None

	#Acquires time from GPS
	def getGpsTime():
					
			gps.update()

			return datetime.combine(date(
					gps.timestamp_utc.tm_year,
					gps.timestamp_utc.tm_mon,
					gps.timestamp_utc.tm_mday),
					time(gps.timestamp_utc.tm_hour,
					gps.timestamp_utc.tm_min,
					gps.timestamp_utc.tm_sec)
				)

	#Acquires data from GPS
	def getGpsData():
			
			try:
					
					gps.update()
					
					return GPS(
							gps.latitude,
							gps.longitude,
							gps.altitude_m,
							gps.speed_knots,#*(463/900) Currently exists as none and mutiplying by integers throws errors
							gps.satellites,
							gps.horizontal_dilution
						)

			except:
					
					if(debugLevel >= 2):
						print("\n Failed to get GPS Data Returned As None \n")
					return GPS(
							None,
							None,
							None,
							None,
							None,
							None
						)

	#Acquires data from Altimeter
	def getAltimeterData():
			
			try:

					return ALTIMETER(
							bme.altitude,
							bme.temperature + temperature_offset, #Applies temperature offset
							bme.pressure,
							bme.humidity,
							round(bme.gas)
						)

			except:
					
					errorLed.value = True;
					if(debugLevel >= 2):
						print("\n Failed to get Altimeter Data Returned As None \n")
					return ALTIMETER(
							None,
							None,
							None,
							None,
							None
						)

	#Acquires data from Accelerometer
	def getAccelerometerData():
			
			try:
					
					return ACCELEROMETER(
							imu.acceleration[0],
							imu.acceleration[1],
							imu.acceleration[2]
						)

			except:
					
					if(debugLevel >= 2):
						print("\n Failed to get Accelerometer Data Returned As None \n")
					return ACCELEROMETER(
							None,
							None,
							None
						)

	#Acquires data from Gyroscope
	def getGyroscopeData():
			
			try:
			
					return GYROSCOPE(
							imu.gyro[0],
							imu.gyro[1],
							imu.gyro[2]
						)

			except:
					
					if(debugLevel >= 2):
						print("\n Failed to get Gyroscope Data Returned As None \n")
					return GYROSCOPE(
							None,
							None,
							None
						)

	#Acquires data from Magnetometer
	def getMagnetometerData():
			
			try:
					
					return MAGNETOMETER(
							mag.magnetic[0],
							mag.magnetic[1],
							mag.magnetic[2]
						)

			except:
					
					if(debugLevel >= 2):
						print("\n Failed to get Magnometer Data Returned As None \n")
					return MAGNETOMETER(
							None,
							None,
							None
						)

	#Acquires data from current sensor to determine power draw
	def getPowerDrawData():
			
			try:
					
					return POWER(
							powerDraw.bus_voltage,
							(powerDraw.current / 1000),
							abs(powerDraw.bus_voltage * (powerDraw.current / 1000))
						)

			except:
							
					if(debugLevel >= 2):
						print("\n Failed to get Power Draw Data Returned As None \n")
					return POWER(
							None,
							None,
							None
						)

	#Acquires data from current sensor to determine solar input
	def getSolarData():
			
			solarData = SOLARPANEL(
					None,
					None,
					None
				)

			#Pulls data for solar panel set one
			try:
					
					solarData.One = POWER(
							solarOne.bus_voltage,
							(solarOne.current / 1000),
							abs(solarOne.bus_voltage * (solarOne.current / 1000))
						)

			except:
					
					if(debugLevel >= 2):
						print("\n Failed to get Solar Panel Output One Data Returned As None \n")
					solarData.One =  POWER(
							None,
							None,
							None
						)

			#Pulls data for solar panel set Two
			try:
					
					solarData.Two = POWER(
							solarTwo.bus_voltage,
							(solarTwo.current / 1000),
							abs(solarTwo.bus_voltage * (solarTwo.current / 1000))
						)

			except:
					
					print("\n Failed to get Solar Panel Output two Data Returned As None \n")
					solarData.Two = POWER(
							None,
							None,
							None
						)

			#Pulls data for solar panel set Three
			try:
					
					solarData.Three = POWER(
							solarThree.bus_voltage,
							(solarThree.current / 1000),
							abs(solarThree.bus_voltage * (solarThree.current / 1000))
						)

			except:
					
					print("\n Failed to get Solar Panel Output Three Data Returned As None \n")
					solarData.Three = POWER(
							None,
							None,
							None
						)

			return solarData

	#Acquires data from battery sensor
	def getBatteryData():
			
			try:
					
					if battery.cell_voltage >= battery_detection_threshold:
					
							return BATTERY(
									battery.cell_voltage,
									battery.cell_percent
								)
					
					else:
							
							return BATTERY(
									None,
									None
								)
			
			except:
					
					if(debugLevel >= 2):
						print("\n Failed to get Battery Data Returned As None \n")
					return BATTERY(
							None,
							None
						)
			
	#Acquires data from Analog sensors
	def getAnalogData():
			
			try:
					
					return ANALOG(
							((AnalogIn(adc, PCF.A0).value / 65535) * vbus_reference_voltage),
							((AnalogIn(adc, PCF.A1).value / 65535) * vbus_reference_voltage),
							((AnalogIn(adc, PCF.A2).value / 65535) * vbus_reference_voltage),
							((AnalogIn(adc, PCF.A3).value / 65535) * vbus_reference_voltage),
							((GPIO26.value / 65536) * vbus_reference_voltage),
							((GPIO27.value / 65536) * vbus_reference_voltage),
							((GPIO28.value / 65536) * vbus_reference_voltage)
						)

			except:
					
					if(debugLevel >= 2):
						print("\n Failed to get Analog Data Returned As None \n")
					return ANALOG(
						None,
						None,
						None,
						None,
						None,
						None,
						None
					)

	#Acquires data from all sensors at once
	def getAvionicsData():
			
			data = AVIONICSDATA(
					getTime(),
					getGpsData(),
					getAltimeterData(),
					getAccelerometerData(),
					getGyroscopeData(),
					getMagnetometerData(),
					getPowerDrawData(),
					getSolarData(),
					getBatteryData(),
					getAnalogData()
				)

			return data

	#Converts a value into bytes
	def convertData(value, numBytes, multValue):
			# in order to make code simpler, the value is checkde within this function to be none
			# almost all values are made int() then *___ a scaling factor which takes place through the multValue
			# we chech if they arent none because it throws a fatal error when trying to convert int(None)
			try:
				if(value != None):
					return (int(value)*multValue).to_bytes(numBytes, 'big', signed = 'True')
				return int(0).to_bytes(numBytes, 'big', signed = 'True')
			except:
				return int(0).to_bytes(numBytes, 'big', signed = 'True')
			
	#Acquired and encodes data into bytes
	def getRawData():
			
			processLed.value = True;
			rawData = bytes()
			
			#Acquires data from all sensors
			data = getAvionicsData()
			
			#Converts year, month, day, hour, minute, and second to bytes
			#try:
					#rawData += (
							#data.TIME.year.to_bytes(2, 'big', signed='True') +
							#data.TIME.month.to_bytes(2, 'big', signed='True') +
							#data.TIME.day.to_bytes(2, 'big', signed='True') +
							
							#data.TIME.hour.to_bytes(2, 'big', signed='True') +
							#data.TIME.minute.to_bytes(2, 'big', signed='True') +
							#data.TIME.second.to_bytes(2, 'big', signed='True'))
			#except:
					
					#try:
							#rawData += (data.TIME).to_bytes(2, 'big', signed='True')
					#except:
							#rawData += (clock.monotonic()).to_bytes(2, 'big', signed='True')
			
			#Converts GPS latitude, longitude, altitude, speed, satellites, and DOP to bytes
			rawData += convertData(1, 4, 1)            
			rawData += convertData(data.GPS.Latitude, 4, 100)            
			rawData += convertData(data.GPS.Longitude, 4, 100)  
			rawData += convertData(data.GPS.Altitude, 4, 100)               
			rawData += convertData(data.GPS.Speed, 4, 100)         
			#rawData += convertData(data.GPS.Satellites, 1)         
			rawData += convertData(data.GPS.DOP, 4, 100)   
			
			#Converts altimeter altitude, temperature, pressure, humidty, and gas to bytes
			rawData += convertData(2, 4, 1)            
			rawData += convertData(data.ALTIMETER.Altitude, 4, 100)
			rawData += convertData(data.ALTIMETER.Temperature, 4, 1)
			rawData += convertData(data.ALTIMETER.Pressure, 4, 100)
			rawData += convertData(data.ALTIMETER.Humidity, 4, 100)
			rawData += convertData(data.ALTIMETER.Gas, 4, 100)
		
			#Convert accelerometer X, Y, and Z to bytes
			rawData += convertData(3, 4, 1)            
			rawData += convertData(data.ACCELEROMETER.X, 4, 100)
			rawData += convertData(data.ACCELEROMETER.Y, 4, 100)
			rawData += convertData(data.ACCELEROMETER.Z, 4, 100)
		
			#Convert gyroscope X, Y, and Z to bytes
			rawData += convertData(4, 4, 1)            
			rawData += convertData(data.GYROSCOPE.X*100, 4, 100)
			rawData += convertData(data.GYROSCOPE.Y*100, 4, 100)
			rawData += convertData(data.GYROSCOPE.Z*100, 4, 100)
				
			#Convert Magnometer X, Y, and Z to bytes
			rawData += convertData(5, 4, 1)            
			rawData += convertData(data.MAGNETOMETER.X, 4, 100)
			rawData += convertData(data.MAGNETOMETER.Y, 4, 100)
			rawData += convertData(data.MAGNETOMETER.Z, 4, 100)
					
			#Convert power draw voltage, current, and wattage to bytes
			rawData += convertData(6, 4, 1)            
			rawData += convertData(data.POWERDRAW.Voltage, 4, 1000)
			rawData += convertData(data.POWERDRAW.Current, 4, 1000)
			rawData += convertData(data.POWERDRAW.Wattage, 4, 1000)
			
			#Convert solar panel voltage, current, and wattage for panel sets one, two, and three to bytes
			#if(data.SOLARPANEL.One.Voltage != None):
			rawData += convertData(7, 4, 1)            
			rawData += convertData(data.SOLARPANEL.One.Voltage, 4, 1000)
			rawData += convertData(data.SOLARPANEL.One.Current, 4, 1000)
			rawData += convertData(data.SOLARPANEL.One.Wattage, 4, 1000)
			rawData += convertData(data.SOLARPANEL.Two.Voltage, 4, 1000)
			rawData += convertData(data.SOLARPANEL.Two.Current, 4, 1000)
			rawData += convertData(data.SOLARPANEL.Two.Wattage, 4, 1000)
			rawData += convertData(data.SOLARPANEL.Three.Voltage, 4, 1000)
			rawData += convertData(data.SOLARPANEL.Three.Current, 4, 1000)
			rawData += convertData(data.SOLARPANEL.Three.Wattage, 4, 1000)
					
			#Convert battery voltage and percentage to bytes
			rawData += convertData(8, 4, 1)            
			rawData += convertData(data.BATTERY.Voltage, 4, 100)
			rawData += convertData(data.BATTERY.Percentage, 4, 100)
					
			#Convert analog voltage to bytes
			rawData += convertData(9, 4, 1)            
			rawData += convertData(data.ANALOG.A0, 4, 100)
			rawData += convertData(data.ANALOG.A1, 4, 100)
			rawData += convertData(data.ANALOG.A4, 4, 100)
			rawData += convertData(data.ANALOG.A3, 4, 100)
			rawData += convertData(data.ANALOG.A4, 4, 100)
			rawData += convertData(data.ANALOG.A5, 4, 100)
			rawData += convertData(data.ANALOG.A6, 4, 100)

			# 55 data points stored
						
			processLed.value = False;			
			return rawData

	#Grabs data from a specific recording
	def getSpecificData(specificDataNumber):
			
			try:
					
					with open("/sd/" + str(specificDataNumber), "rb") as dataFile:
							return dataFile.read()
					
			except:
					
				print("\n[Unable to Find Stored Data]\n")
				errorCode(2)
					
	#Runs setup commands for the transceiver
	def setupTransceiver():
			
			global transceiver
			
			try:
			
					transceiver = UART1
					
			except Exception as e:

					print("\n[Unable to Setup Transceiver]\n")
					errorCode(1, e)

	#Runs setup commands for the gps
	def setupGps():
			
			global gps
			
			try:
					
					gps = adafruit_gps.GPS(
							UART0,
							debug=False
						)

					gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0') #Issues parameters to GPS
					gps.send_command(b'PMTK220,1000')
					
			except Exception as e:
					
					print("\n[Unable to Setup GPS]\n")
					errorCode(3, e)
			log(gps)

	#Runs setup commands for the altimeter
	def setupAltimeter():
			
			global bme
			
			try:
					
					bme = adafruit_bme680.Adafruit_BME680_I2C(
							I2C0,
							address=0x77
						)

					bme.sea_level_pressure = 1013.25
					
					bme.pressure_oversampling = 8
					bme.temperature_oversampling = 2
					
			except Exception as e:
					
					print("\n[Unable to Setup Altimeter]\n")
					errorCode(1, e)

	#Runs setup commands for the imu
	def setupImu():
			
			global imu
			
			try:
					
					imu = LSM6DSOX(
							I2C0,
							address=0x6a
						)

			except Exception as e:
					
					print("\n[Unable to Setup IMU]\n")
					errorCode(1, e)
					
	#Runs setup commands for the imu
	def setupMag():
			
			global mag
			
			try:
					
					mag = LIS3MDL(
							I2C0,
							address=0x1c
						)

			except Exception as e:
					
					print("\n[Unable to Setup Magnetometer]\n")
					errorCode(1, e)

	#Runs setup commands for the current sensor that monitors power draw
	def setupPowerDraw():
			
			global powerDraw
			
			try:
					
					powerDraw = INA219(
							I2C0,
							addr=0x45
						)

					powerDraw.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
					powerDraw.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
					powerDraw.bus_voltage_range = BusVoltageRange.RANGE_16V

			except Exception as e:
					
					print("\n[Unable to Connect to Power Draw Current Sensor]\n")
					errorCode(1, e)

	#Runs setup commands for the current sensors that monitor solar input
	def setupSolar():
			
			global solarOne
			global solarTwo
			global solarThree
			
			#Sets up solar set one
			try:

					solarOne = INA219(
							I2C0,
							addr=0x40
						)

					solarOne.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
					solarOne.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
					solarOne.bus_voltage_range = BusVoltageRange.RANGE_16V
					
			except Exception as e:
					
					print("\n[Unable to Connect to Solar Panel One Current Sensor]\n")
					errorCode(1, e)
			
			#Sets up solar set two
			try:
					
					solarTwo = INA219(
							I2C0,
							addr=0x41
						)

					solarTwo.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
					solarTwo.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
					solarTwo.bus_voltage_range = BusVoltageRange.RANGE_16V
					
			except Exception as e:
					
					print("\n[Unable to Connect to Solar Panel Two Current Sensor]\n")
					errorCode(1, e)
			
			#Sets up solar set three
			try:
					
					solarThree = INA219(
							I2C0,
							addr=0x44
						)

					solarThree.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
					solarThree.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
					solarThree.bus_voltage_range = BusVoltageRange.RANGE_16V

			except Exception as e:
					
					print("\n[Unable to Connect to Solar Panel Three Current Sensor]\n")
					errorCode(1, e)

	#Runs setup commands for the battery monitor
	def setupBattery():
			
			global battery

			try:
					
					battery = adafruit_max1704x.MAX17048(
							I2C0,
							address=0x36
						)
				
			except Exception as e:
					print("\n[Unable to Connect to Battery Sensor]\n")
					errorCode(1, e)

	#Runs setup commands for the analog to digital converter
	def setupAnalog():
			
			global adc

			try:
					
					adc = PCF.PCF8591(
							I2C0,
							address=0x48
						)
					
			except Exception as e:
					
					print("\n[Unable to Connect to Analog to Digital Converter]\n")
					errorCode(1, e)

	#Sets up SD Card
	def setupSD():
			
			global sdcard

			try:
					
					sdcard = adafruit_sdcard.SDCard(
							SPI1,
							CS1
						)
					
					vfs = storage.VfsFat(sdcard)
					storage.mount(vfs, "/sd")
					
			except Exception as e:
					
					print("\n[Unable to Connect to SD Card]\n")
					errorCode(2, e)

	#Stores data on sd card
	def storeData(data):
			
			global dataNumber
			
			try:
					
					dataNumber += 1
					
					if dataNumber >= maxDataNumber:
							
							dataNumber = 0
			
					saveState() #Updates backup information

					with open("/sd/" + str(dataNumber), "w") as dataFile:
							dataFile.write(data)
							
			except Exception as e:
					
					print("\n[Unable to Save Data to SD Card]\n")
					(2, e)

	#Handles the transmission of data through the LoRa
	def transmit(data):
			
			try:
					
					transmitLed.value = True
					
					packet = callsign.encode("ascii")
					
					if isinstance(data, bytes): #Checks if data is in byte form
							packet += data
					else:
							packet += data.encode("ascii")
					
					if len(packet) > 236: #Checks if packet is short enough for LoRa buffer
							print("[Packet is Too Long for Transmission]")
					else:
							transceiver.write(packet) #Transmits packet
							
					transmitLed.value = False
							
			except Exception as e:
					
					print("\n[Unable to Transmit Data]\n")
					errorCode(1, e)
					
					transmitLed.value = False

	#Generates the beacon signal and sends it
	def sendBeacon():
			
			global beaconSequence
			
			try:
					
					clock.sleep(0.25) #Delay to prevent altimeter oversampling
					transmitLed.value = True;

					if beaconSequence == 0:
							
							beaconSequence += 1
							
							dataPoint = getBatteryData().Percentage
							beaconSensor = "BAT: "
							beaconUnits = "%"
							
					elif beaconSequence == 1:
							
							beaconSequence += 1
							
							dataPoint = getGpsData().Altitude
							beaconSensor = "ALT: "
							beaconUnits = "m"
							
					elif beaconSequence == 2:
							
							beaconSequence += 1
	
							dataPoint = str(round(getGpsData().Latitude*1000)/1000) + ", " + str(round(getGpsData().Longitude*1000)/1000)
							beaconSensor = "GPS: "
							beaconUnits = ""
							
					elif beaconSequence == 3 :
							
							beaconSequence = 0
							
							dataPoint = getGpsData().Speed
							beaconSensor = "SPD: "
							beaconUnits = "m/s"
							
					else:   
							
							beaconSequence = 0
							
							dataPoint = ""
							beaconSensor = ""
							beaconUnits = ""
					
					saveState() #Updates backup information
					
					if dataPoint == None:
							dataPoint = 0
					
					try:
							dataString = str(round(dataPoint*10)/10) #Converts data to str with one decimal point
					except:
							dataString = dataPoint #Used for preformated data points
					
					transmit(" <" + beaconSensor + dataString + beaconUnits + ">")
					print("\n" + callsign + " <" + beaconSensor + dataString + beaconUnits + ">\n")
					
					clock.sleep(0.25) #Delay to prevent altimeter oversampling
					transmitLed.value = False;
					
			except Exception as e:

					print("[Beacon Data Points Could Not Be Sent]")
					errorCode(1, e)
					
					try:
							
							transmit("")
							
					except Exception as e:
							
							print("Beacon Could Not Be Sent")
							errorCode(1, e)

	#Receives and processes groundstation commands
	def commandProcessor(command):

			global callsign
			global beaconInterval 
			global measurementInterval
			global maxDataNumber
			global maxErrorNumber

			printValue = "";

			try:
					
				command = command.decode("ascii").lower().replace(" ", "") #Decodes and removes formatting from the message

				if command[0:3] == "set":
					command = command[3:]
			
					if command[0:8] == "callsign":
							
							command = command[8:]
							callsign = command.upper()
							printValue = "Callsign Set To: " + str(callsign)							
							saveConfig()								
					elif command[0:14] == "beaconInterval":
							
							command = command[4:]
							beaconInterval = int(command)
							printValue = "Beacon Interval Set To: " + str(beaconInterval)							
							saveConfig()								
					elif command[0:15] == "measureInterval":
							
							command = command[15:]
							measurementInterval = int(command)
							printValue = "Measurement Interval Set To: " + str(measurementInterval)
							saveConfig()								
					elif command[0:7] == "dataMac":
							
							command = command[7:]
							maxDataNumber = int(command)					
							printValue = "Max Data Set To: " + str(maxDataNumber)
							saveConfig()								
					elif command[0:6] == "errMax":
							
							command = command[6:]
							maxErrorNumber = int(command)
							printValue = "Max Error Set To: " + str(maxErrorNumber)		
							saveConfig()
					else:
							
							print("[Command Not Understood]")
				elif command[0:3] == "get":
					if command == "Current":
						printValue = getRawData()
					elif command[0:4] == "file" :
						command = command[4:]						
						printValue = getSpecificData(int(command))				
					elif command == "runtime":
							printValue = "Runtime: " + str(clock.monotonic() - uptimeTicker) + "s"       
					elif command == "monotonic":
							printValue = "Monotonic Time: " + str(clock.monotonic()) + "s"                 
					elif command == "UTC":
							printValue = "Time: " + str(getTime())              
					elif command == "dataNum":
							printValue = "Data Entries: " + str(dataNumber)              
					elif command == "errorNum":
							printValue = "Error Entries: " + str(errorNumber)                               
					elif command[0:5] == "data":
						command = command[5:]
						if command == "GPS":
							printValue = getGpsData()
						if command == "Altimeter":
							printValue = getAltimeterData()
						if command == "Accelerometer":
							printValue = getAccelerometerData()
						if command == "Gyroscope":
							printValue = getGyroscopeData()
						if command == "Magnometer":
							printValue = getMagnetometerData()
						if command == "powerDraw":
							printValue = getPowerDrawData()
						if command == "Solar":
							printValue = getSolarData()
						if command == "Battery":
							printValue = getBatteryData()
						if command == "Analog":
							printValue = getAnalogData()
						if command == "All":
							printValue = getAvionicsData()
						else:

							printValue = "Command Not Understood"
					else:
							
							printValue = "Command Not Understood"
				elif command[0:3] == "msg":

					command = command[3:]						
					printValue = str(str(command))
				elif command[0:5] == "cube.":
					command = command[5:]
					if command == "reload":

						transmit("Reloading Cubesat")                 
						saveState()
						saveConfig()
						microcontroller.reset()
					elif command == "ping":	
						printValue = "pong"							
					elif command == "pong":
						printValue = "ping"					
				else:

						printValue = "Command Not Understood"
							
			except:

					printValue = "Command Not Understood"

			transmit(printValue);
			print(printValue);

	#Retrieves saved configuration from file
	def reloadConfiguration():
			
			global callsign
			global beaconInterval
			global measurementInterval
			global maxDataNumber
			global maxErrorNumber
			
			try:
							
					with open("/sd/Configuration.txt", "r") as configFile:

							configData = configFile.readlines()
							
							callsign = str(configData[0]).replace("\n", "").replace("\r", "")
							beaconInterval = int(configData[1])
							measurementInterval = int(configData[2])
							maxDataNumber = int(configData[3])
							maxErrorNumber = int(configData[4])
					
					print("[Machine Configuration Restored]")
					
			except Exception as e:
					
					print("[Unable to Reload Configuration]")
					
					saveConfig()

	#Updates configuration file
	def saveConfig():
			
			try:
			
					with open("/sd/Configuration.txt", "w") as stateFile:
									stateFile.write(str(callsign) + "\r\n" + str(beaconInterval) + "\r\n" + str(measurementInterval) + "\r\n" + str(maxDataNumber) + "\r\n" + str(maxErrorNumber))
					
					print("[Machine Configuration Saved]")
					
			except Exception as e:
					
					print("[Unable to Save Machine Configuration]")
					errorCode(1, e)

	#Retrieves state information from file
	def resumeState():
			
			global dataNumber
			global beaconSequence
			global errorNumber
			
			try:
			
					with open("/sd/StateBackup.txt", "r") as stateFile:

							stateData = stateFile.readlines()

							dataNumber = int(stateData[0])
							beaconSequence = int(stateData[1])
							errorNumber = int(stateData[2])
					
					print("[Backup State Restored]")
					
			except Exception as e:
					
					print("[Unable to Restore Backup State]")
					
					updateDataNumber()
					saveState()
					
	def updateDataNumber():
			
			global dataNumber
			
			try:
					
					print("[Recovering Data]")
					
					fileSearch = True
					currentFile = -1
					
					while fileSearch:
							
							currentFile += 1
							
							try:
									
									with open("/sd/" + str(currentFile), "r") as searchFile:
											pass
									
							except:
									
									fileSearch = False
									
					dataNumber = currentFile
					
			except:
					
					dataNumber = -1 #Line one in StateBackup.txt

	#Updates state information on file
	def saveState():
			
			try:
					
					with open("/sd/StateBackup.txt", "w") as stateFile:
									stateFile.write(str(dataNumber) + "\r\n" + str(beaconSequence) + "\r\n" + str(errorNumber))
					
			except Exception as e:
					
					print("[Unable to Save Machine State]")
					errorCode(1, e)

	#Produces an error code on LED and saves error message in log
	def errorCode(value, message):
			
			global errorNumber
			
			print(message)
			
			try:
							
					if value >= 0:
							code = value
					else:
							code = 0
					
					errorNumber += 1
							
					if errorNumber >= maxErrorNumber:
							
							with open("/sd/ErrorLog.txt", "w") as errorFile:
									errorFile.write("")
							
							errorNumber = 0
					
					saveState() #Updates backup information
					
					with open("/sd/ErrorLog.txt", "a") as errorFile:
							errorFile.write("[" + str(getTime()) + "] " + str(message) + "\n")
					
					for i in range(0, code):

							errorLed.value = True
							clock.sleep(0.1)
							errorLed.value = False
							clock.sleep(0.1)
							
					clock.sleep(0.25)
					
			except Exception as e:
					
					print("[Could Not Log Error]\n")
					print(e)

	def log(message):
		try:
				errorNumber += 1
						
				if errorNumber >= maxErrorNumber:
						
						with open("/sd/ErrorLog.txt", "w") as errorFile:
								errorFile.write("")
						
						errorNumber = 0
				
				saveState() #Updates backup information
				
				with open("/sd/ErrorLog.txt", "a") as errorFile:
						errorFile.write("[" + str(getTime()) + "] " + str(message) + "\n")
				
		except Exception as e:
				
				print("[Could Not Log]\n")
				print(e)

	def testLeds():
			if(random.randint(0, 1) == 1):
				if(gpsLed.value == True):
					gpsLed.value = False
				else:
					gpsLed.value = True
			
			if(random.randint(0, 1) == 1):
				if(transmitLed.value == True):
					transmitLed.value = False
				else:
					transmitLed.value = True
			
			if(random.randint(0, 1) == 1):
				if(receiveLed.value == True):
					receiveLed.value = False
				else:
					receiveLed.value = True
			
			if(random.randint(0, 1) == 1):
				if(processLed.value == True):
					processLed.value = False
				else:
					processLed.value = True
			
			if(random.randint(0, 1) == 1):
				if(errorLed.value == True):
					errorLed.value = False
				else:
					errorLed.value = True
			if(random.randint(0, 1) == 1):
				if(imbeddedLed.value == True):
					imbeddedLed.value = False
				else:
					imbeddedLed.value = True
	
	#Acquires time from monotonic clock
	uptimeTicker = clock.monotonic()
	
	measurementTicker = clock.monotonic()
	beaconTicker = clock.monotonic()

	#Sets up SD card
	setupSD()

	#Attempts to reload configuration
	reloadConfiguration()
	resumeState()

	#Sets up GPS, locks GPS, and sets internal time
	setupGps()

	#Runs sensor setups
	setupTransceiver()
	setupAltimeter()
	setupImu()
	setupMag()
	setupPowerDraw()
	setupSolar()
	# If battery is not plugged in for testing or what have you, it will error and reset code trying to run setupBattery
	#setupBattery()
	setupAnalog()


	#Main program loop
	while True:
			#Updates GPS
			gps.update()
			gpsLed.value = True

			#testLeds()

			#Listen for transmitted commands
			command = transceiver.read(960)
			
			#Process commands if one is received
			if command != None:
					if(debugLevel >= 3):
						print("Command Recieved")
					receiveLed.value = True
					
					commandProcessor(command)
					
					receiveLed.value = False
			
			# Measure and store the data on an interval
			if clock.monotonic() - measurementTicker > measurementInterval:
					
					measurementTicker = clock.monotonic()
					if(debugLevel >= 4):
						print("Data Stored")
					storeData(getRawData()) #Gets and stores data

			# Send the beacon back down on the predefined beaconInterval=
			if clock.monotonic() - beaconTicker > beaconInterval:
					
					beaconTicker = clock.monotonic()
							
					if(debugLevel >= 4):
						print("Beacon Sent")

					#sendBeacon()

#Reboots device in the case of a critical error
except Exception as e:

	try:
			
			print("Critical Error: " + str(e))
			errorCode(1, e)
			#Comment out to have continuous connection to the cubeSat
			#If it continuously disconnects and reconnects, comenting this out stops that
			#microcontroller.reset()

	
	except:
	
			print("Could Not Save Error")
			#Comment out to have continuous connection to the cubeSat
			#If it continuously disconnects and reconnects, comenting this out stops that
			#(File explorer opens to the raspberry pie, closes to desktop, opens new tab, etc. etc.)
			#microcontroller.reset()
	
