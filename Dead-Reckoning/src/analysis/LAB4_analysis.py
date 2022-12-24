#!/usr/bin/env python
from bagpy import bagreader
import bagpy
import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import stats
from scipy import signal
from math import cos
from math import sin
from scipy.spatial.transform import Rotation as R

imu_freq = 40
gps_freq = 1

def load_data(address, get_topic):
	bag = bagreader(address) 
	data = bag.message_by_topic(topic = get_topic)
	data = pd.read_csv(data)
	return data
def periodic_points(data, avg_period, data_freq):
	data_len = len(data)
	t0 = 1/data_freq
	i_idx = int(data_len//2)
	f_idx = int(i_idx + avg_period*3*data_freq)
	period_idx = [0]
	slope = np.ones(30)

	for i in range(i_idx,f_idx):
		for j in range(30):
			slope[j] = np.diff(data)[i-j]/t0
		key = i - np.argmin(abs(slope))
		if abs(np.mean(slope)) < 0.01 and (key-period_idx[-1])>200:
			period_idx.append(key)

	return period_idx

def calibration_value_mag(data):
	# get best fit ellipse
	magxmean, magymean = data[0].mean(), data[1].mean()
	data[0] -= magxmean
	data[1] -= magymean
	U, S, V = np.linalg.svd(np.stack((data[0], data[1])))
	N = len(data[0])
	tt = np.linspace(0, 2*np.pi, N)
	circle = np.stack((np.cos(tt), np.sin(tt)))    # unit circle
	transform = np.sqrt(2/N) * U.dot(np.diag(S))   # transformation matrix
	fit = transform.dot(circle) + np.array([[0], [0]])

	#get calibration values:
	a_index = np.argmax(fit[0,:])
	b_index = np.argmax(fit[1,:])

	theta = math.atan2(fit[1,a_index],fit[0,a_index])
	a = math.sqrt(fit[1,a_index]**2 + fit[0,a_index]**2)
	b = math.sqrt(fit[1,b_index]**2 + fit[0,b_index]**2)

	data[0] += magxmean
	data[1] += magymean

	return magxmean, magymean, theta, a, b

def calibrate_mag(data, cx, cy, theta, a, b):
	#translate
	out_data = [data[0],data[1]]
	out_data[0] = data[0] - cx
	out_data[1] = data[1] - cy
	#rotate
	out_data[0] = math.cos(theta)*out_data[0] + math.sin(theta)*out_data[1]
	out_data[1] = -1*math.sin(theta)*out_data[0] + math.cos(theta)*out_data[1]
	#scale
	out_data[0] *= b / a
	return out_data

def rotate_frame(data_x,data_y, theta):
	theta = math.radians(theta)
	out_data_x = math.cos(theta)*data_x + math.sin(theta)*data_y
	out_data_y = -1*math.sin(theta)*data_x + math.cos(theta)*data_y
	return out_data_x, out_data_y

def butter_filter(data, fs, cutoff, type):
    nyq = 0.5*fs
    order = 2 
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    sos = signal.butter(order, normal_cutoff, btype=type, output='sos', analog=False)
    y = signal.sosfilt(sos, data)
    return y

def moving_average(a, n=3):
	ret = np.cumsum(a, dtype=float)
	ret[n:] = ret[n:] - ret[:-n]
	return ret[n - 1:] / n

def find_stationary(data, stat_time, data_freq):
	key = stat_time*data_freq
	stat_period = []
	count = 0
	jerk_data = np.diff(data)
	for i in range(1,len(jerk_data)):
		check = count
		if abs(jerk_data[i])<0.05:
			count +=1
		else:
			count = 0
		if check > key and count == 0:
			stat_period.append(i-1-check)
			stat_period.append(i-1)
	return stat_period
	plt.plot(range(len(jerk_data)),jerk_data)
	plt.show()

def find_stationary2(data,stat_time,data_freq):
	key = stat_time*data_freq
	stat_period = []
	count = 0
	for i in range(1,len(data)):
		check = count
		if abs(data[i]-data[i-1])<0.05:
			count +=1
		else:
			count = 0
		if check > key and count == 0:
			stat_period.append(i-1-check)
			stat_period.append(i-1)
	return stat_period

def remove_noise(noisy_data, stat_periods):
	N=len(stat_periods)
	data=np.ones(len(noisy_data))
	for i in range(1,N):
		mean_noise = np.mean(noisy_data[stat_periods[i-1]:stat_periods[i]])
		if i==1:
			start = 0
		else:
			start = stat_periods[i-1]
		if i<N-1:
			end = stat_periods[i+1]
		else:
			end = len(noisy_data)
		for j in range(start,end):
			data[j] = noisy_data[j]- 1*mean_noise
	return data


def gps_vel_disp(gps_data):
	samples_gps = len(gps_data)
	vel_gps = np.ones(samples_gps)
	disp_gps = np.zeros(samples_gps)
	cx = gps_data['UTM_easting'][0]
	cy = gps_data['UTM_northing'][0]
	for i in range(1,samples_gps):
		x0 = gps_data['UTM_easting'][i-1]
		y0 = gps_data['UTM_northing'][i-1]
		x1 = gps_data['UTM_easting'][i]
		y1 = gps_data['UTM_northing'][i]
		vel_gps[i-1] = math.sqrt((y1-y0)**2+(x1-x0)**2)
		disp_gps[i] = math.sqrt((y1-cy)**2+(x1-cx)**2)
	vel_gps[-1]=vel_gps[-2]
	vel_gps2 = np.diff(disp_gps)
	np.append(vel_gps2,0)
	return vel_gps, disp_gps, vel_gps2

def hard_correction(data,start,end,bias):
	data[start*imu_freq:end*imu_freq] += bias
	return data

def correct_accel(acc_data):
	acc_data = acc_data-np.mean(acc_data)
	stationary_periods = find_stationary(acc_data,2,imu_freq)
	acc_data = remove_noise(acc_data,stationary_periods)
	return acc_data, stationary_periods

def correct_accel2(acc_data):
	acc_data = acc_data-acc_data[0]
	print(acc_data)
	stationary_periods = find_stationary2(acc_data,3,imu_freq)
	acc_data = remove_noise(acc_data,stationary_periods)
	return acc_data, stationary_periods

def get_grav_comp(yaw,pitch,roll):
	rot_mat = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
	rot_mat = rot_mat.as_matrix()
	grav_vect = [[0],[0],[-9.8]]
	return np.matmul(rot_mat,grav_vect)

def correct_for_pitch(data,t0,t1):
	t0 *= imu_freq
	t1 *= imu_freq
	pitch = np.mean(imu_data['IMU.pitch'][t0:t1])
	roll = np.mean(imu_data['IMU.pitch'][t0:t1])
	grav_vector = get_grav_comp(0,pitch,roll)
	data[t0:t1] -= grav_vector[0]
	return grav_vector[0]

# Load calibration data
calb_address = '/home/divi/ros_workspace/eece5554/LAB4/src/data/magnetometer_calb.bag'
imu_data = load_data(calb_address, '/imu')
mag_x= imu_data['MagField.magnetic_field.x']
mag_y= imu_data['MagField.magnetic_field.y']

#selecting good data_points (data_points where continuous movements - data in middle for 3 periods)
mag_periods = periodic_points(mag_x,20,imu_freq)
mag_data = [mag_x[mag_periods[4]:mag_periods[6]],mag_y[mag_periods[4]:mag_periods[6]]]

periodic_val = np.zeros(len(mag_periods))
# plt.plot(range(len(mag_x)), mag_x)
# plt.scatter(mag_periods, periodic_val,color = 'r')
# plt.legend(['Mag yaw X','Periodic points'])
# plt.xlabel('Data Points')
# plt.ylabel('Magnetic_field X (Gauss)')
# plt.grid(True)
# plt.show()

#get calibration values
center_x, center_y, rot_angle, major_ax, minor_ax =calibration_value_mag(mag_data)
print(center_x, center_y, rot_angle, major_ax, minor_ax)

#calibrate magnetometer
calb_mag_data = calibrate_mag(mag_data,center_x,center_y,rot_angle,major_ax,minor_ax)
# plt.scatter(calb_mag_data[0],calb_mag_data[1])
# plt.scatter(mag_data[0],mag_data[1])
# plt.legend(['Mag calibrated','Mag uncalibrated'])
# plt.xlabel('Magnetic_field X (Gauss)')
# plt.ylabel('Magnetic_field Y (Gauss)')
# plt.axis('equal')
# plt.grid(True)
# plt.show()

# load moving data
moving_address = '/home/divi/ros_workspace/eece5554/LAB4/src/data/moving_data.bag'
imu_data = load_data(moving_address, '/imu')
imu_time = imu_data['Header.stamp.secs']-imu_data['Header.stamp.secs'].min()+(imu_data['Header.stamp.nsecs']//pow(10,6))/1000
samples_imu = len(imu_time)


# correct for error in data collection by fetching data from raw_datastring
imu_data['IMU.yaw'] = np.ones(samples_imu)
imu_data['IMU.pitch'] = np.ones(samples_imu)
imu_data['IMU.roll'] = np.ones(samples_imu)
for i in range(samples_imu):
	test=imu_data['raw_datastring'][i].split(',')
	imu_data['IMU.yaw'][i]=float(test[1])
	imu_data['IMU.pitch'][i] = float(test[2])
	imu_data['IMU.roll'][i] = float(test[3])
	test2=test[-1].split('*')
	imu_data['IMU.angular_velocity.z'][i]=float(test2[0])

imu_data['IMU.pitch']=imu_data['IMU.pitch'] - np.mean(imu_data['IMU.pitch'][0:50])
imu_data['IMU.roll']=imu_data['IMU.roll'] - np.mean(imu_data['IMU.roll'][0:50])

#get mag data and calibrate for soft iron and hard iron bias
mag_data = [imu_data['MagField.magnetic_field.x'], imu_data['MagField.magnetic_field.y']]
calb_mag_data = calibrate_mag(mag_data,center_x,center_y,rot_angle,major_ax,minor_ax)

#yaw from IMU
imu_data['IMU.yaw'] = np.unwrap(imu_data['IMU.yaw'])
	
	#shift to origin in start IMU yaw
imu_data['IMU.yaw'] = imu_data['IMU.yaw']-np.mean(imu_data['IMU.yaw'][0:20])

#yaw from magnetometer
mag_yaw = np.ones(samples_imu)
mag_yaw_uncalb = np.ones(samples_imu)
for i in range(samples_imu):
	mag_yaw_uncalb[i] = -math.degrees(math.atan2(mag_data[1][i],mag_data[0][i]))
	mag_yaw [i] = -math.degrees(math.atan2(calb_mag_data[1][i],calb_mag_data[0][i]))

	#correct magnetic values around time period 500-700 - Error due to tram power lines
avg_size = 50
corr_start = 19000
corr_end = 26500
mag_yaw[corr_start:corr_end-avg_size+1] = moving_average(mag_yaw[corr_start:corr_end],avg_size)
mag_yaw_uncalb[corr_start:corr_end-avg_size+1] = moving_average(mag_yaw_uncalb[corr_start:corr_end],avg_size)
mag_yaw[20160:21400] = moving_average(mag_yaw[24760:26049],avg_size)
mag_yaw_uncalb[20160:21400] = moving_average(mag_yaw_uncalb[24760:26049],avg_size)
avg_size = 100
mag_yaw[corr_start:corr_end-avg_size+1] = moving_average(mag_yaw[corr_start:corr_end],avg_size)
mag_yaw_uncalb[corr_start:corr_end-avg_size+1] = moving_average(mag_yaw_uncalb[corr_start:corr_end],avg_size)

	#unwrap data
mag_yaw = np.unwrap(mag_yaw,period=360)
mag_yaw_uncalb = np.unwrap(mag_yaw_uncalb, period=360)

#imu_time = range(samples_imu)
# plt.plot(imu_time, mag_yaw)
# plt.plot(imu_time, mag_yaw_uncalb)
# plt.legend(['Mag yaw calibrated','Mag_yaw uncalibrated'])
# plt.xlabel('Time (sec)')
# plt.ylabel('Orientation (deg)')
# plt.grid(True)
# plt.show()

	#shift to origin in start - mag yaw
mag_yaw = mag_yaw - np.mean(mag_yaw[0:40])


# yaw from gyroscope
gyro_yaw = np.cumsum(imu_data['IMU.angular_velocity.z'])*1/imu_freq
for i in range(samples_imu):
	gyro_yaw[i] = math.degrees(math.atan2(math.sin(gyro_yaw[i]), math.cos(gyro_yaw[i])))
gyro_yaw = np.unwrap(gyro_yaw)


## plot yaw values
# plt.plot(imu_time,gyro_yaw, imu_time, mag_yaw)
# plt.legend(['Gyro yaw','Mag yaw'])
# plt.xlabel('Time (sec)')
# plt.ylabel('Orientation (deg)')
# plt.show()

#Applying low pass filter to Mag
mag_yaw_filtered = butter_filter(mag_yaw, imu_freq, 0.08, 'low')

#Applying high pass filter to Gyro
gyro_yaw_filtered = butter_filter(gyro_yaw, imu_freq, 0.0000001, 'high')

#Using complementary filter with MAG and GYRO to calculate YAW
alpha = 0.2
comp_yaw = alpha*mag_yaw_filtered + (1-alpha)*gyro_yaw_filtered

# plt.plot(imu_time,comp_yaw,imu_time,gyro_yaw_filtered, imu_time, mag_yaw_filtered)
# plt.legend(['Complementary','Gyro yaw','Mag yaw'])
# plt.xlabel('Time (sec)')
# plt.ylabel('Orientation (deg)')
# plt.show()

# plt.plot(imu_time, imu_data['IMU.yaw'],'--')
# plt.plot(imu_time, comp_yaw)
# # plt.plot(imu_time,gyro_yaw_filtered, imu_time, mag_yaw_filtered)
# plt.legend([ 'IMU yaw','Complementary','Gyro yaw','Mag yaw'])
# plt.xlabel('Time (sec)')
# plt.ylabel('Orientation (deg)')
# plt.show()

#Load moving data for gps
moving_address = '/home/divi/ros_workspace/eece5554/LAB4/src/data/moving_data.bag'
gps_data = load_data(moving_address, '/gps')
gps_time = gps_data['Header.stamp.secs']-gps_data['Header.stamp.secs'].min()
gps_data['UTM_easting'] = gps_data['UTM_easting'] - gps_data['UTM_easting'][0]
gps_data['UTM_northing'] = gps_data['UTM_northing'] - gps_data['UTM_northing'][0]
samples_gps = len(gps_time)

#Before correction
vel_gps,disp_gps, vel_gps2 = gps_vel_disp(gps_data)
acc_x = imu_data['IMU.linear_acceleration.x'] - imu_data['IMU.linear_acceleration.x'].mean()
vel_imu = np.cumsum(acc_x*1/imu_freq)

# plt.plot(imu_time, vel_imu)
# plt.plot(gps_time, vel_gps)
# plt.legend(['IMU velocity', 'GPS Velocity'])
# plt.xlabel('Velocity (m/sec)')
# plt.ylabel('Time (sec)')
# plt.grid(True)
# plt.show()

#After correction
acc_x, stationary_periods = correct_accel(imu_data['IMU.linear_acceleration.x'])
stat_values= np.zeros(len(stationary_periods))

stationary_time = [float(i)/imu_freq for i in stationary_periods]
# plt.scatter(stationary_time,stat_values,color='r')
# plt.plot(imu_time,acc_x)
# plt.grid(True)
# plt.show()

imu_data['IMU.pitch']=butter_filter(imu_data['IMU.pitch'],imu_freq,0.08,'low')
imu_data['IMU.roll']=butter_filter(imu_data['IMU.roll'],imu_freq,0.08,'low')
# plt.plot(imu_time, imu_data['IMU.pitch'])
# plt.plot(imu_time, imu_data['IMU.roll'])
# plt.legend(['Pitch','Roll'])
# plt.show()

# 	#correcting acceleration using IMU Pitch
correction = correct_for_pitch(acc_x,205,260)
print('correction', correction)
# acc_x[300*imu_freq:400*imu_freq] += 3*correction
# acc_x[410*imu_freq:420*imu_freq] -= 25*correction
acc_x[300*imu_freq:410*imu_freq] += 3.2*correction
acc_x[405*imu_freq:495*imu_freq] -= 0.58*(1*(50)**2+3*(100)**2)/(75**2)*correction

vel_gps,disp_gps, vel_gps2 = gps_vel_disp(gps_data)
vel_imu = np.cumsum(acc_x*1/imu_freq)
# acc_x[440*imu_freq:495*imu_freq] = 0
vel_imu[440*imu_freq:495*imu_freq] = 0

plt.plot(imu_time, vel_imu)
plt.plot(gps_time, vel_gps)
plt.legend(['IMU velocity', 'GPS Velocity'])
plt.xlabel('Velocity (m/sec)')
plt.ylabel('Time (sec)')
plt.grid(True)
plt.show()

imu_data['IMU.yaw']=imu_data['IMU.yaw']+171
comp_yaw = comp_yaw + 171
#get displacement from imu
disp_imu= np.zeros(len(vel_imu))
disp_e = np.zeros(len(vel_imu))
disp_n = np.zeros(len(vel_imu))
for i in range(1,len(vel_imu)):
	ve = vel_imu[i]*cos(math.radians(comp_yaw[i]))
	vy = vel_imu[i]*sin(math.radians(comp_yaw[i]))
	disp_e[i]= disp_e[i-1]+(ve*1/imu_freq)
	disp_n[i]= disp_n[i-1]+(vy*1/imu_freq)
	disp_imu[i] = math.sqrt(disp_e[i]**2 + disp_n[i]**2)

plt.plot(imu_time, disp_imu)
gps_time = range(0,len(disp_gps))
plt.plot(gps_time, disp_gps)
plt.legend(['IMU distance', 'GPS dispacement'])
plt.grid(True)
plt.show()

for i in range(35):
	ang = math.atan2(gps_data['UTM_northing'][i+1]-gps_data['UTM_northing'][i],gps_data['UTM_easting'][i+1]-gps_data['UTM_easting'][i])
	ang = math.degrees(ang)
	if ang > 105:
		print(i,math.degrees(ang))
		break

gps_e,gps_n = rotate_frame(gps_data['UTM_easting'][0:samples_gps],gps_data['UTM_northing'][0:samples_gps],105)
disp_n = -1*disp_n

ang2 = math.degrees(math.atan2(disp_n[-1]-disp_n[-2],disp_e[-1]-disp_e[-2]))
print(ang2)
# disp_e,disp_n = rotate_frame(disp_e,disp_n,-105)

# plt.plot(gps_data['UTM_easting'],gps_data['UTM_northing'])
plt.plot(gps_e,gps_n,'--')
plt.plot(disp_e,disp_n)
plt.xlabel('UTM_easting')
plt.ylabel('UTM_northing')
plt.legend(['GPS data', 'IMU data'])
plt.axis('equal')
plt.grid(True)
plt.show()

Y2 = np.zeros(len(vel_imu))
for i in range(len(vel_imu)):
	Y2[i] = imu_data['IMU.angular_velocity.z'][i]*vel_imu[i]


acc_y, stationary_periods2 = correct_accel2(imu_data['IMU.linear_acceleration.y'])
acc_y = butter_filter(acc_y,imu_freq,1,'low')
print(stationary_periods2)
print(acc_y)
stat_values = np.zeros(len(stationary_periods2))
plt.plot(imu_time, Y2)
plt.plot(imu_time, acc_y)
plt.legend(['Omega.X*','y**observed'])
plt.xlabel('Time (sec)')
plt.ylabel('Accelerarion (m/sec/sec)')
# plt.scatter(stationary_periods2,stat_values,color='r')
plt.show()

omega = imu_data['IMU.angular_velocity.z']
alpha = np.gradient(omega, 1/imu_freq)
y_jerk = np.gradient(acc_y,1/imu_freq)
w_jerk = np.gradient(alpha, 1/imu_freq)

xc = np.ones(samples_imu)
for i in range(samples_imu):
	num = (omega[i]*y_jerk[i]) - (alpha[i]*acc_y[i]) - (omega[i]**2)*acc_x[i]
	den = (omega[i]*w_jerk[i]) + (omega[i]**4) - (alpha[i]**2)
	xc[i] = num/den

print(xc)
print('Mean',np.mean(xc))
