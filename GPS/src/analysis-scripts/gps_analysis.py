#!/usr/bin/env python
from bagpy import bagreader
import bagpy
import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import stats

bag_name=input('Enter rosbag name (Eg. stationary_data.bag): ')
bag = bagreader('/home/divi/eece5554/LAB1/src/data/%s'%bag_name)

#read gps data
gps_data = bag.message_by_topic(topic = '/gps')
data = pd.read_csv(gps_data)
print(len(data['UTM_northing']))

if 'stationary' in bag_name:
	#deviations from stationary position
	northing_offset=int((data['UTM_northing']//1000).mean()*1000)
	easting_offset=int((data['UTM_easting']//1000).mean()*1000)
	data['Offset_northing'] = data['UTM_northing'] - northing_offset
	data['Offset_easting'] = data['UTM_easting'] - easting_offset

	#estimating error radius for an easting-northing position
	easting_mean = data['Offset_easting'].mean()
	northing_mean = data['Offset_northing'].mean()
	data['x2']=pow((data['Offset_northing']-northing_mean),2)
	data['y2']=pow((data['Offset_easting']-easting_mean),2)
	data["error_distance"] = pow((data['x2']+data['y2']),0.5)
	rad = 0.5
	bounded = False
	while bounded == False:
		frac_data = float(len(data[(data['error_distance'])<=rad]))/float(len(data['error_distance']))
		print(frac_data, rad)
		if frac_data>=0.95:
			bounded=True
			break
		rad+=0.01
	rad="{:.2f}".format(rad)

	#estimating error distribution in terms of standard deviation
	mean_dist = data['error_distance'].mean()
	std_dist = data['error_distance'].std()
	data['dist_to_mean'] = data['error_distance']-mean_dist
	
	#Table plot
	fig, (plt1, plt2, plt3) = plt.subplots(3)
	fig.subplots_adjust(hspace=0.5)
	fig.suptitle('Deviations in stationary GPS data, Bounding radius: %s meter'%rad)
	plt1.plot(data['Offset_easting'], data['Offset_northing'], marker='o', linestyle='dashed')
	plt1.set_ylabel('Northing (Offset: %d meter)' %northing_offset)
	plt1.set_xlabel('Easting (Offset: %d meter)' %easting_offset)
	plt1.grid(True)
	plt1.set_xlim([310,314])
	plt1.set_ylim([395,399])
	plt1.set_title('Position of GPS sensor in easting and northing space')
	x = range(0,len(data['error_distance']),1)
	plt2.plot(x,data['error_distance'])
	plt2.set_ylabel('Distance from mean (meter)')
	plt2.set_xlabel('Data points')
	plt2.set_title('Deviations from mean easting and northing position')
	plt2.set_xlim(0,650)
	plt2.set_ylim(0,2.5)
	plt2.grid(True)
	bins = ([-1.5,-1,-0.5,0,0.5,1,1.5,2])
	plt3.hist(data['dist_to_mean'],bins)
	plt3.set_ylabel('Number of data points')
	plt3.set_xlabel('Deviation')
	plt3.set_title('Error Distribution around mean: %f meter' %mean_dist)
	plt3.grid(True)
	plt.show()
	#altitude graph
	plt.plot(x, data['Altitude'])
	plt.grid(True)
	plt.xlabel('Data points')
	plt.ylabel('Altitude (meters)')
	plt.title('Variation in estimated altitude')
	plt.show()
	
else:
	#Estimating mean trajectory of GPS
	northing_offset=int((data['UTM_northing']//1000).mean()*1000)
	easting_offset=int((data['UTM_easting']//1000).mean()*1000)
	data['Offset_northing'] = data['UTM_northing'] - northing_offset
	data['Offset_easting'] = data['UTM_easting'] - easting_offset
	res = stats.linregress(data['Offset_easting'],data['Offset_northing'])
	m = (data['Offset_northing'].iloc[-1]-data['Offset_northing'].iloc[0])/(data['Offset_easting'].iloc[-1]-data['Offset_easting'].iloc[0])
	c = data['Offset_northing'].iloc[-1]-(m*data['Offset_easting'].iloc[-1])
	data['Y_fn_X'] = res.slope*data['Offset_easting'] + res.intercept
	#data['Y_fn_X'] = m*data['Offset_easting'] + c
	#Deviations from mean trajectory
	data['error_distance'] = abs(data['Offset_northing']-data['Y_fn_X'])
	
	rad = 0.5
	bounded = False
	while bounded == False:
		frac_data = float(len(data[(data['error_distance'])<=rad]))/float(len(data['error_distance']))
		print(frac_data, rad)
		if frac_data>=0.95:
			bounded=True
			break
		rad+=0.01
	rad="{:.2f}".format(rad)
	
	#estimating error distribution in terms of standard deviation
	mean_dist = data['error_distance'].mean()
	std_dist = data['error_distance'].std()
	data['dist_to_mean'] = data['error_distance']-mean_dist

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
	plt1.plot(data['Offset_easting'], data['Offset_northing'], data['Offset_easting'], data['Y_fn_X'])
	plt1.set_ylabel('Northing (Offset: %d meter)' %northing_offset)
	plt1.set_xlabel('Easting (Offset: %d meter)' %easting_offset)
	plt1.legend('Sensor data', 'Estimated trajectory')
	plt1.grid(True)
	plt1.set_xlim([125,275])
	plt1.set_ylim([370,390])
	plt1.set_title('Position of GPS sensor in easting and northing space')
	x = range(0,len(data['error_distance']),1)
	plt2.plot(x,data['error_distance'])
	plt2.set_ylabel('Distance from mean (meter)')
	plt2.set_xlabel('Data points')
	plt2.set_title('Deviations from mean easting and northing position')
	plt2.set_xlim(0,110)
	plt2.set_ylim(0,2)
	plt2.grid(True)
	bins = ([-1.5,-1,-0.5,0,0.5,1,1.5,2])
	plt3.hist(data['dist_to_mean'],bins)
	plt3.set_ylabel('Number of data points')
	plt3.set_xlabel('Deviation')
	plt3.set_title('Error Distribution around mean: %f meter' %mean_dist)
	plt3.grid(True)
	plt.show()
	#altitude graph
	plt.plot(x, data['Altitude'])
	plt.grid(True)
	plt.xlabel('Data points')
	plt.ylabel('Altitude (meters)')
	plt.title('Variation in estimated altitude')
	plt.show()
