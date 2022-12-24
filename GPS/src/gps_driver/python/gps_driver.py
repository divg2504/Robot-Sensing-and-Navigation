#!/usr/bin/env python
import sys
import serial
import rospy
import math
import utm
from gps_driver.msg import gps_msg

#global variables
latitude = 0
longitude = 0
timestamp = 0.0
altitude = 0.0
easting = 0
northing = 0
zone = 32
letter = 'A'

sec_data = 0.0
nsec_data = 0.0
lat_v = 0.0
lat_d = 'N'
long_v = 0.0
long_d = 'E'


def collect_GPS_data(port):	
	received_data = port.readline()
	received_data = str(received_data)
	required_value = 'GPGGA'
	global timestamp, lat_v, lat_d, long_v, long_d, altitude
	if required_value in received_data:
		gps_data = received_data.split(',')
		timestamp = float(gps_data[1])
		lat_v = float(gps_data[2])
		lat_d = gps_data[3]
		long_v = float(gps_data[4])
		long_d = gps_data[5]
		altitude = float(gps_data[9])
		
		#Convert lat_v and long_v from DDM format to DD format
		lat_v = lat_v//100 + (lat_v%100)/60
		long_v = long_v//100 + (long_v%100)/60
		return True
	else:
		return False

def data_formating():
	#converting time to sec and nsec
	global latitude, longitude, nsec_data, sec_data
	temp_time = timestamp
	nsec_data = (temp_time-math.floor(temp_time))*pow(10,9)
	temp_time = math.floor(temp_time)
	sec_data = (temp_time%100) + (temp_time%10000-temp_time%100)*0.60 + (temp_time-temp_time%10000)*0.3600
	
	#converting Latitude and Longitude in gps_msg format: 'Value direction'
	latitude = lat_v
	longitude = long_v

def GPS_to_UTM():
	global easting,northing,zone,letter, lat_v, long_v
	if lat_d == 'S':
		lat_v = -1*lat_v
	if long_d == 'W':
		long_v = -1*long_v
	easting, northing, zone, letter = utm.from_latlon(lat_v,long_v)

def publish_gps_data(gps_pub):
	 msg = gps_msg()
	 #msg.Header.stamp = sec_data*10^9 + nsec_data
	 msg.Header.stamp.secs = int(sec_data)
	 msg.Header.stamp.nsecs = int(nsec_data)
	 msg.Header.frame_id = 'GPS1_Frame'
	 msg.Latitude = latitude
	 msg.Longitude = longitude
	 msg.Altitude = altitude
	 msg.UTM_easting = round(easting,2)
	 msg.UTM_northing = round(northing,2)
	 msg.Zone = zone
	 msg.Letter = letter
	 gps_pub.publish(msg)


if __name__ == "__main__":
	#check for serial-port address in arguments
	input_check = True
	for item in sys.argv:
		check_value = '/dev/'
		if type(item)==str and check_value in item:
			print("Serial-port address: %s" %item)
			input_check = False
	if input_check:
		sys.exit("Program needs to ls run with an argument for serial-port address on which GPS is connected. Example: GPS_driver.py /dev/ttyUSB")

	#Initiate serial port to receive data
	serial_port = str(sys.argv[1])
	baud_rate = 4800
	try:
		port = serial.Serial(serial_port,baud_rate,timeout=1)
	except:
		sys.exit("Unable to connect with GPS module at serial-port %s"%serial_port)

	#Initialize node publishing on topic /gps
	rospy.init_node('gps_driver', anonymous = True)

	#Initiate publisher for gps_msg
	if len(sys.argv) == 3: data_freq = sys.argv[2]
	else: data_freq = 1
	gps_publisher = rospy.Publisher('gps', gps_msg, queue_size = 10)
	publish_rate = rospy.Rate(data_freq)
	
	#Counter for data samples
	count = 0

	while not rospy.is_shutdown():
		#collect GPS data through serial port 'port'
		collected = collect_GPS_data(port)
		if collected==False:
			continue
		else:
			count+=1
		rospy.loginfo('Collected GPS data sample %d for timestamp: %f'%(count,timestamp))
		
		#Convert latitude and longitude data to UTM format
		GPS_to_UTM()

		#Format data for publishing on ros topic /gps
		data_formating()
		
		#Publish data on topic /gps in format of gps_msg
		publish_gps_data(gps_publisher)

		#delay as per msg frequency 
		publish_rate.sleep()
