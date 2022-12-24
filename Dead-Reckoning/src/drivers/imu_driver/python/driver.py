#!/usr/bin/env python
import sys
import serial
import rospy
import math
from imu_driver.msg import imu_msg
import numpy as np

data_arr = np.ones(13)
raw_data = 'raw_string'

def read_data(port):
	global data_arr, raw_data
	received_data = port.readline()
	raw_data = str(received_data)
	
	imu_data = raw_data.split(',')
	temp = imu_data[-1].split('*')

	for i in range(1,12):
		data_arr[i] = float(imu_data[i])
	data_arr[12] = float(temp[0])

def to_quaternions():
	global data_arr
	y=math.radians(data_arr[1])
	p=math.radians(data_arr[2])
	r=math.radians(data_arr[3])
	cy = math.cos(y)
	cp = math.cos(p)
	cr = math.cos(r)
	sy = math.sin(y)
	sp = math.sin(p)
	sr = math.sin(r)

	dcm = [cp*cy,(sr*sp*sy)+(cr*cy),cr*cp]
	qx = math.sqrt(0.25*(1+dcm[0]-dcm[1]-dcm[2]))
	qy = math.sqrt(0.25*(1-dcm[0]+dcm[1]-dcm[2]))
	qz = math.sqrt(0.25*(1-dcm[0]-dcm[1]+dcm[2]))
	qw = math.sqrt(0.25*(1+dcm[0]+dcm[1]+dcm[2]))
	
	data_arr[0]=qx
	data_arr[1]=qy
	data_arr[2]=qz
	data_arr[3]=qw

def publish_imu_data(pub,count):
	msg = imu_msg()
	msg.Header.seq = count
	msg.Header.frame_id="IMU1_Frame"
	msg.Header.stamp = rospy.Time.now()
	msg.IMU.header.frame_id = 'IMU1_Frame'
	msg.IMU.header.stamp = rospy.Time.now()
	msg.IMU.orientation.x = data_arr[0]
	msg.IMU.orientation.y = data_arr[1]
	msg.IMU.orientation.z = data_arr[2]
	msg.IMU.orientation.w = data_arr[3]
	msg.MagField.header.frame_id = 'IMU1_Frame'
	msg.MagField.header.stamp = rospy.Time.now()
	msg.MagField.magnetic_field.x = data_arr[4]
	msg.MagField.magnetic_field.y = data_arr[5]
	msg.MagField.magnetic_field.z = data_arr[6]
	msg.IMU.linear_acceleration.x = data_arr[7]
	msg.IMU.linear_acceleration.y = data_arr[8]
	msg.IMU.linear_acceleration.z = data_arr[9]
	msg.IMU.angular_velocity.x = data_arr[10]
	msg.IMU.angular_velocity.y = data_arr[11]
	msg.IMU.angular_velocity.z = data_arr[12]
	msg.raw_datastring = raw_data
	pub.publish(msg)

if __name__ == "__main__":
	

	#Initiate serial port to receive data
	baud_rate = 115200
	serial_port = rospy.get_param("/imu_port")
	try:
		port = serial.Serial(serial_port,baud_rate,timeout=1)
	except:
		sys.exit("Unable to connect with IMU at serial-port %s"%serial_port)


	#configuring IMU to set output data frequency at 40Hz and data output to VNYMR 
	data_freq = 40
	data_output_mode = 14
	
	try:
		port.write(b"$VNWRG,07,%d*xx\r" %data_freq)
		port.write(b"$VNWRG,06,%d*xx\r" %data_output_mode)
		check = 0
		while check == 0:
			aodf_req = port.write(b"VNRRG,07*xx\r")
			AODF = port.readline()
			if str(data_freq) in str(aodf_req) or str(data_freq) in str(AODF):
				check = 1

		while check == 1:
			aodf_req = port.write(b"VNRRG,06*xx\r")
			AODF = port.readline()
			if str(data_output_mode) in str(aodf_req) or str(data_output_mode) in str(AODF):
				check = 0
		print('IMU configured')
	except:
		print('IMU not configured as per requirement')


	#Initialize node publishing on topic imu
	rospy.init_node('imu_driver', anonymous = True)

	#Initiate publisher for imu_msg
	imu_publisher = rospy.Publisher('imu', imu_msg, queue_size = 10)
	publish_rate = rospy.Rate(data_freq)
	
	#Required data points in terms of minutes
	#required_data=sys.input('Duration of data collection in minutes: ')

	count = 1
	while not rospy.is_shutdown():

		#read data from serial-port
		collected = read_data(port)
		if collected==False:
			continue
		
		#Convert yaw-roll-pitch data to quaternions
		to_quaternions()

		#Publish data on topic imu in format of imu_msg
		publish_imu_data(imu_publisher, count)

		count+=1

		#delay as per msg frequency 
		publish_rate.sleep()

	port.close()




