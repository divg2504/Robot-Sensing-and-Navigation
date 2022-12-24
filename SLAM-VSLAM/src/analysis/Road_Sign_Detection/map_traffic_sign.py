#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import pickle
import numpy as np

def extract_traj(poses_file):
    trajectory = open(poses_file, 'r')
    count = 0
    lines = trajectory.readlines()
    trajectory_x = []
    trajectory_y = []
    while count < len(lines):
        # Get next line from file
        data = lines[count].split(' ')
        trajectory_x.append(float(data[1]))
        trajectory_y.append(float(data[3]))
        count += 1
    trajectory.close()
    return trajectory_x, trajectory_y

def map_traf_sign(sign_dict, trajectory_x, trajectory_y):
    traf_sign = pickle.load( open( sign_dict, "rb" ))
    speed_sign = traf_sign['speed_limit']
    speed = np.zeros((len(speed_sign),2))
    for i in range(len(speed_sign)):
        item = speed_sign[i]
        key = item.split('.')
        x = key[0][-5:]
        speed[i,0]=trajectory_x[int(x)]
        speed[i,1]=trajectory_y[int(x)]

    yield_sign = traf_sign['yield']
    yield_p = np.zeros((len(yield_sign),2))
    for i in range(len(yield_sign)):
        item = yield_sign[i]
        key = item.split('.')
        x = key[0][-5:]
        yield_p[i,0]=trajectory_x[int(x)]
        yield_p[i,1]=trajectory_y[int(x)]
    

    yield_traf_sign = traf_sign['yeild_traffic']
    yield_traf = np.zeros((len(yield_traf_sign),2))
    for i in range(len(yield_traf_sign)):
        item = yield_traf_sign[i]
        key = item.split('.')
        x = key[0][-5:]
        yield_traf[i,0]=trajectory_x[int(x)]
        yield_traf[i,1]=trajectory_y[int(x)]

    mandatory_sign = traf_sign['mandatory']
    mandatory = np.zeros((len(mandatory_sign),2))
    for i in range(len(mandatory_sign)):
        item = mandatory_sign[i]
        key = item.split('.')
        x = key[0][-5:]
        mandatory[i,0]=trajectory_x[int(x)]
        mandatory[i,1]=trajectory_y[int(x)]
    return speed, yield_p, yield_traf, mandatory


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("poses_file", help="pose path file in TUM format")
    parser.add_argument("sign_dict", help="dictionary of traffic signs in pickle format")
    args = parser.parse_args()
    trajectory_x, trajectory_y = extract_traj(args.poses_file)
    speed, yield_p, yield_traf, mandatory = map_traf_sign(args.sign_dict, trajectory_x, trajectory_y)
    plt.plot(trajectory_x,trajectory_y)
    plt.scatter(speed[:,0],speed[:,1])
    plt.scatter(yield_p[:,0],yield_p[:,1])
    plt.scatter(yield_traf[:,0],yield_traf[:,1])
    plt.scatter(mandatory[:,0],mandatory[:,1])
    plt.legend(['Trajectory','Speed sign','Yield Pedestrian','Yield Traffic','Mandatory'])
    plt.axis('equal')
    plt.show()