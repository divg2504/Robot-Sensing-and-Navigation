#!/usr/bin/env python
from bagpy import bagreader
import bagpy
import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import stats

#Load rosbag
address = '/home/divi/eece5554/LAB3/src/data/individual_data_correct.bag'
bag = bagreader(address) 
data_freq = 40

#read imu data
imu_data = bag.message_by_topic(topic = '/imu')
data = pd.read_csv(imu_data)
data_len = len(data['Header.stamp.secs'])
data['yaw']=np.ones(data_len)
data['pitch']=np.ones(data_len)
data['roll']=np.ones(data_len)
time=data['Header.stamp.secs']-data['Header.stamp.secs'].min()+round(data['Header.stamp.nsecs']/10**9,4)

# convert quaternions back to Euler
conv = 180/math.pi
for i in range(data_len):
	qx = data.at[i,'IMU.orientation.x']
	qy = data.at[i,'IMU.orientation.y']
	qz = data.at[i,'IMU.orientation.z']
	qw = data.at[i,'IMU.orientation.w']
	data.at[i,'roll'] = conv*math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);
	data.at[i,'pitch'] = conv*math.asin(2.0*(qx*qz - qw*qy));
	data.at[i,'yaw'] = conv*math.atan2(-2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);

print(data['raw_datastring'].head())
print(data['yaw'].head())
print(data['pitch'].head())
print(data['roll'].head())
#angular velocity data
omega=[data['IMU.angular_velocity.x'],data['IMU.angular_velocity.y'],data['IMU.angular_velocity.z']]
fig, plot_gyro = plt.subplots(2,3)
fig.subplots_adjust(hspace=0.4)
fig.suptitle('Angular velocity')
axis = ['X','Y','Z']
i=0
for item in omega:
	item*=180/math.pi
	mean = np.mean(item)
	stdDev = np.std(item)
	plot_gyro[0,i].plot(time,item)
	plot_gyro[0,i].set_ylabel('Omega (deg/sec)')
	plot_gyro[0,i].set_xlabel('Time (sec)')
	plot_gyro[0,i].grid(True)
	plot_gyro[0,i].set_title('Axis: %s'%axis[i])
	bins = ([mean-3*stdDev,mean-2*stdDev,mean-1*stdDev,mean,mean+1*stdDev,mean+2*stdDev,mean+3*stdDev])
	plot_gyro[1,i].hist(item,bins)
	plot_gyro[1,i].set_ylabel('# of Data points')
	plot_gyro[1,i].set_xlabel('Omega (deg/sec)')
	plot_gyro[1,i].set_xticks(np.round_(bins,5))
	plot_gyro[1,i].set_title('Error Distribution, Mean = %.5f deg/sec,'%mean)
	i+=1
plt.show()

#linear acceleration data
acc=[data['IMU.linear_acceleration.x'],data['IMU.linear_acceleration.y'],data['IMU.linear_acceleration.z']]
fig, plot_gyro = plt.subplots(2,3)
fig.subplots_adjust(hspace=0.4)
fig.suptitle('Linear acceleration')
axis = ['X','Y','Z']
i=0
for item in acc:
	mean = np.mean(item)
	stdDev = np.std(item)
	plot_gyro[0,i].plot(time,item)
	plot_gyro[0,i].set_ylabel('Acceleration (m/sec/sec)')
	plot_gyro[0,i].set_xlabel('Time (sec)')
	plot_gyro[0,i].grid(True)
	plot_gyro[0,i].set_title('Axis: %s'%axis[i])
	bins = ([mean-3*stdDev,mean-2*stdDev,mean-1*stdDev,mean,mean+1*stdDev,mean+2*stdDev,mean+3*stdDev])
	plot_gyro[1,i].hist(item,bins)
	plot_gyro[1,i].set_ylabel('# of Data points')
	plot_gyro[1,i].set_xlabel('Acceleration (m/sec/sec)')
	plot_gyro[1,i].set_xticks(np.round_(bins,4))
	plot_gyro[1,i].set_title('Error Distribution, Mean = %.5f m/sec/sec,'%mean)
	i+=1
plt.show()

#magnetic field data
mag=[data['MagField.magnetic_field.x'],data['MagField.magnetic_field.y'],data['MagField.magnetic_field.z']]
fig, plot_gyro = plt.subplots(2,3)
fig.subplots_adjust(hspace=0.4)
fig.suptitle('Magnetic field')
axis = ['X','Y','Z']
i=0
for item in mag:
	mean = np.mean(item)
	stdDev = np.std(item)
	plot_gyro[0,i].plot(time,item)
	plot_gyro[0,i].set_ylabel('Magnetic field (Gauss)')
	plot_gyro[0,i].set_xlabel('Time (sec)')
	plot_gyro[0,i].grid(True)
	plot_gyro[0,i].set_title('Axis: %s'%axis[i])
	bins = ([mean-3*stdDev,mean-2*stdDev,mean-1*stdDev,mean,mean+1*stdDev,mean+2*stdDev,mean+3*stdDev])
	plot_gyro[1,i].hist(item,bins)
	plot_gyro[1,i].set_ylabel('# of Data points')
	plot_gyro[1,i].set_xlabel('Magnetic field')
	plot_gyro[1,i].set_xticks(np.round_(bins,4))
	plot_gyro[1,i].set_title('Error Distribution around mean, Mean = %.5f Gauss,'%mean)
	i+=1
plt.show()

#Orientation data
orient=[data['yaw'],data['pitch'],data['roll']]
fig2, plot_gyro = plt.subplots(2,3)
fig2.subplots_adjust(hspace=0.4)
fig2.suptitle('Orientation - Euler Angles')
axis = ['Yaw','Pitch','Roll']
i=0
for item in orient:
	mean = np.mean(item)
	stdDev = np.std(item)
	plot_gyro[0,i].plot(time,item)
	plot_gyro[0,i].set_ylabel('Degree')
	plot_gyro[0,i].set_xlabel('Time (sec)')
	plot_gyro[0,i].grid(True)
	plot_gyro[0,i].set_title('Axis: %s'%axis[i])
	bins = ([mean-3*stdDev,mean-2*stdDev,mean-1*stdDev,mean,mean+1*stdDev,mean+2*stdDev,mean+3*stdDev])
	plot_gyro[1,i].hist(item,bins)
	plot_gyro[1,i].set_ylabel('# of Data points')
	plot_gyro[1,i].set_xlabel('Angle (deg)')
	plot_gyro[1,i].set_xticks(np.round_(bins,4))
	plot_gyro[1,i].set_title('Error Distribution around mean, Mean = %.5f deg,'%mean)
	i+=1
plt.show()
