#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import utm
import serial
from math import sin, pi
from std_msgs.msg import Float64
from gps_driver.msg import gps_msg

    
def DMm2Dd(a,b,la,lo):                                                #Function to convert from DegMin.m to Deg.deg
    latD=float(a[:2])
    #print(latD)
    lonD=float(b[:3])
    latMm=float(a[2:])
    #print(latMm)
    lonMm=float(b[3:])
    latd=latMm/60
    lond=lonMm/60
    DecDeg=[latD+latd, lonD+lond]
    if la=='S':
        DecDeg[0] = -DecDeg[0]
    if lo=='W':
        DecDeg[1] = -DecDeg[1]
    return DecDeg

def convertTime(a):
    hr=float(a[:2])
    min=float(a[2:4])
    sec=float(a[4:6])
    totSeconds=int(hr*3600+min*60+sec)
    ns=int(float(a[6:])*(10**9))
    print(hr, min, sec, ns/(10**9))
    return [totSeconds,ns]
    


def gpsNode():
    pub = rospy.Publisher('gps', gps_msg, queue_size=10)
    rospy.init_node('gpsNode', anonymous=True)                     #node inititalization
    rate = rospy.Rate(10) # 10hz
    serial_port=rospy.get_param("/gps_port")
    serial_baud=4800
    gps=gps_msg()                                                   #inititalize gps_msg classs as an ovject to call it
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    
    while not rospy.is_shutdown():
        line=port.readline().decode("utf-8")
        charnum=line.find("$")
        gpsdata=line[charnum+1:]
        #print(gpsdata)
        if gpsdata.startswith('GPGGA'):
            #print(gpsdata)
            gpsls=gpsdata.split(',')
            #print(gpsls)
            lla=[gpsls[2],gpsls[3],gpsls[4],gpsls[5],gpsls[9]]                              #Extract Lat,Long,Alt
            print(lla)
            deglla=DMm2Dd(lla[0],lla[2], str(lla[1]), str(lla[3]))
            print(deglla)
            utmll=utm.from_latlon(deglla[0], deglla[1])                   #convert lat long from degrees to utm
            print('Latitude, Longitude in UTM',utmll)
            time=gpsls[1]
            print("Timestamp in GPGGA = ",time)
            headerTime=convertTime(time)                                 #convert timestamp from GPS in hhmmss.ss format to seconds and nanoseconds format for the ros header file
            print('Timestamp in header format = ',headerTime)
            print('\n')
        

            gps.Header.stamp.secs=headerTime[0]
            gps.Header.stamp.nsecs=headerTime[1]                         #sending the converted data to the header file
            gps.Header.frame_id='GPS1_FRAME'
            gps.Latitude=deglla[0]
            gps.Longitude=deglla[1]
            gps.Altitude=float(lla[4])
            gps.UTM_easting=utmll[0]
            gps.UTM_northing=utmll[1]
            gps.Zone=int(utmll[2])
            gps.Letter=str(utmll[3])
            
            print('===================== Data Sent ====================\n',gps)
            print("\n\n\n")

            pub.publish(gps)
        
gpsNode()
