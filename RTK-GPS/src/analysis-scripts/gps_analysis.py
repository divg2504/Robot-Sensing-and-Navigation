#!/usr/bin/env python
from bagpy import bagreader
import bagpy
import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import stats
import sympy as sy

bag_name=input('Enter rosbag address (Eg. /data/stationary_data.bag): ')
address = '/home/divi/eece5554/LAB2/src/data/%s'%bag_name
bag = bagreader(bag_name)
bag_name.lower()

#read gps data
gps_data = bag.message_by_topic(topic = '/gps')
data = pd.read_csv(gps_data)
print(len(data['UTM_northing']))

if 'stationary' in bag_name:
	#deviations from stationary position
	if len(data) >= 500:
		data=data.iloc[0:500]
		
	# northing_offset=int((data['UTM_northing']//1000).min()*1000)
	# easting_offset=int((data['UTM_easting']//1000).min()*1000)
	northing_offset = data['UTM_northing'][0]
	easting_offset = data['UTM_easting'][0]
	data['Offset_northing'] = data['UTM_northing'] - northing_offset
	data['Offset_easting'] = data['UTM_easting'] - easting_offset

	#estimating error radius for an easting-northing position
	easting_mean = data['Offset_easting'].mean()
	northing_mean = data['Offset_northing'].mean()
	data['y2_error']=pow((data['Offset_northing']-northing_mean),2)
	data['x2_error']=pow((data['Offset_easting']-easting_mean),2)
	data["error_distance"] = pow((data['x2_error']+data['y2_error']),0.5)
	rad = 0.0001
	bounded = False
	while bounded == False:
		frac_data = float(len(data[(data['error_distance'])<=rad]))/float(len(data['error_distance']))
		#print(frac_data, rad)
		if frac_data>=0.95:
			bounded=True
			break
		rad+=0.0001
	rad= round(rad,3)

	#estimating error distribution in terms of standard deviation
	mean_dist = data['error_distance'].mean()
	std_dist = data['error_distance'].std()
	data['dist_to_mean'] = data['error_distance']-mean_dist
	
	#Table plot
	fig, (plt1, plt2, plt3) = plt.subplots(3)
	fig.subplots_adjust(hspace=0.5)
	fig.suptitle('Deviations in stationary GPS data, Bounding radius: %s meter'%rad)
	plt1.scatter(data['Offset_easting'], data['Offset_northing'])
	plt1.set_ylabel('Northing (Offset: %d meter)' %northing_offset)
	plt1.set_xlabel('Easting (Offset: %d meter)' %easting_offset)
	plt1.grid(True)
	#plt1.set_xlim([310,314])
	# plt1.set_ylim([407,409])
	#plt1.set_yticks([407.98,408.00,408.02,408.04,408.06,408.08,408.10])
	plt1.set_title('Position of GPS sensor in easting and northing space')
	x = range(0,len(data['error_distance']),1)
	plt2.scatter(x,data['error_distance'], label='Error')
	plt2.set_ylabel('Distance from mean (meter)')
	plt2.set_xlabel('Data points')
	plt2.set_title('Deviations from mean easting and northing position')
	#plt2.set_xlim(0,650)
	#plt2.set_ylim(0,2.5)
	plt2.grid(True)
	ax2 = plt2.twinx()
	ax2.plot(x, data['GNSS_fix_quality'], '--', color = 'orange', label='Fix quality')
	ax2.set_ylabel('Fix quality')
	ax2.grid(False)
	ax2.set_yticks([2,3,4,5])
	plt2.legend()
	
	bins = ([-3*std_dist,-2*std_dist,-1*std_dist,mean_dist,1*std_dist,2*std_dist,3*std_dist])
	plt3.hist(data['dist_to_mean'],bins)
	plt3.set_ylabel('Number of data points')
	plt3.set_xlabel('Standard Deviation')
	plt3.set_xticks([-3*std_dist,-2*std_dist,-1*std_dist,0,1*std_dist,2*std_dist,3*std_dist])
	plt3.set_title('Error Distribution, Mean error: %f meter' %round(mean_dist,3))
	plt3.grid(False)
	plt.show()
	#altitude graph
	mean_alt = data['Altitude'].mean()
	plt.scatter(x, data['Altitude'])
	plt.grid(True)
	plt.xlabel('Data points')
	plt.ylabel('Altitude (meters)')
	plt.title('Mean altitude = %f meter' %round(mean_alt,2))
	ax2 = plt.twinx()
	ax2.plot(x, data['GNSS_fix_quality'], '--', color = 'orange')
	ax2.set_ylabel('Fix quality')
	ax2.grid(False)
	ax2.set_yticks([2,3,4,5])
	plt.show()
	
else:
	#Estimating mean trajectory of GPS
	if len(data) >= 215:
		data = data.iloc[0:216]
	northing_offset=int((data['UTM_northing']//1000).mean()*1000)
	easting_offset=int((data['UTM_easting']//1000).mean()*1000)
	data['Offset_northing'] = data['UTM_northing'] - northing_offset
	data['Offset_easting'] = data['UTM_easting'] - easting_offset
	idx3 = data['Offset_northing'].idxmax()
	idx2 = data['Offset_easting'].idxmax()
	idx1 = data['Offset_northing'].idxmin()
	idx4 = data['Offset_easting'].idxmin()
	idx0 = 0
	idx = [idx0, idx1, idx2, idx3, idx4]
	print(idx0,idx1, idx2, idx3, idx4)
	print(len(data))
	data['Y_fn_X'] = data['Offset_northing']
	for i in range(4):
		t0 = idx[i]
		t1 = idx[i+1]
		if i == 3:
			t1 = len(data['Offset_easting']) - 1
		line1_x = data['Offset_easting'].iloc[t0:t1]
		line1_y = data['Offset_northing'].iloc[t0:t1]
		regr = stats.linregress(line1_x,line1_y)
		data['Y_fn_X'].iloc[t0:t1] = regr.slope*data['Offset_easting'].iloc[t0:t1] + regr.intercept
	data['error_distance']  = data['Offset_northing'] - data['Y_fn_X']
	
	rad = 0.02
	bounded = False
	while bounded == False:
		frac_data = float(len(data[(data['error_distance'])<=rad]))/float(len(data['error_distance']))
		print(frac_data, rad)
		if frac_data>=0.95:
			bounded=True
			break
		rad+=0.01
	rad="{:.2f}".format(rad)

	#Table plot
	fig, (plt1, plt2, plt3) = plt.subplots(3)
	fig.subplots_adjust(hspace=0.5)
	fig.suptitle('Deviations in moving GPS data, Bounding radius: %s meter'%rad)
	plt1.scatter(data['Offset_easting'], data['Offset_northing'],s=20,marker='o', label='Measurement')
	plt1.scatter(data['Offset_easting'], data['Y_fn_X'], s=4,c='orange', marker = 'o', label='Estimated trajectory')
	plt1.set_ylabel('Northing (Offset: %d meter)' %northing_offset)
	plt1.set_xlabel('Easting (Offset: %d meter)' %easting_offset)
	plt1.legend()
	plt1.grid(True)
	plt1.set_title('Position of GPS sensor in easting and northing space')
	x = range(0,len(data['UTM_northing']),1)
	plt2.scatter(x,data['error_distance'], s=10, label='Error') 
	plt2.set_ylabel('Error Distance (meter)')
	plt2.set_xlabel('Data points')
	plt2.set_title('Deviations [True measurement - estimated position(regression)]')
	plt2.set_yticks([-2,-1.5,-1,-0.5,0,0.5,1,1.5,2])
	plt2.grid(True)
	ax2 = plt2.twinx()
	ax2.plot(x, data['Fix_quality'], '--', color = 'orange', label='Fix quality')
	ax2.set_ylabel('Fix quality')
	ax2.grid(False)
	ax2.set_yticks([2,3,4,5])
	plt2.legend()

	std_dist = data['error_distance'].std()
	mean_dist = data['error_distance'].mean()
	bins = ([-3*std_dist,-2*std_dist,-1*std_dist,mean_dist,1*std_dist,2*std_dist,3*std_dist])
	plt3.hist(data['error_distance'],bins)
	plt3.set_ylabel('# of Data points')
	plt3.set_xlabel('Standard Deviation')
	plt3.set_xticks([-3*std_dist,-2*std_dist,-1*std_dist,mean_dist,1*std_dist,2*std_dist,3*std_dist])
	plt3.set_title('Error Distribution around mean, Mean = %f meter,'%round(mean_dist,3))
	plt.show()
	
	#altitude graph
	mean_alt = data['Altitude'].mean()
	plt.scatter(x, data['Altitude'])
	plt.grid(True)
	plt.xlabel('Data points')
	plt.ylabel('Altitude (meters)')
	plt.title('Mean altitude: %f meters' %round(mean_alt,2))
	ax2 = plt.twinx()
	ax2.plot(x, data['Fix_quality'], '--', color = 'orange')
	ax2.set_ylabel('Fix quality')
	ax2.grid(False)
	ax2.set_yticks([2,3,4,5])
	plt.show()
