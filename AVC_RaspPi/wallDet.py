#!/usr/bin/env python
""" Simple occupancy-grid-based mapping.

Author: David Gutow, Rich Paasch
Version: 8/2018
"""

import sys
import socket
import struct
import time
import numpy as np
import vehicleState as vs
import os as os
import constants as ct
import h5py

from scipy import ndimage
from scipy import misc                      # scipy
#from skimage import *  # scikit-image

import matplotlib.pyplot as plt             # matplotlib
import   math as math

def plotBuffer(lidarBuffer):
        #print("start")
    angles= np.expand_dims(np.linspace(0,2*math.pi,360),1)
    distances = np.expand_dims(lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE],1)
    points = np.concatenate((np.sin(angles)*distances, \
                             np.cos(angles)*distances \
                             ),axis=1)
    #for i in range(360):
    #    points[i,:]= points[i,:]*lidarBuffer[i,ct.LIDAR_BUFFER_DISTANCE]
    #print("break")
    #print(points)

    #plt.close()
    plt.cla()
    plt.plot(points[:,0],points[:,1],linestyle=' ',marker='.',markersize=5,color='k')
    plt.xlim((-12,12))
    plt.ylim((-12,12))
    plt.grid(True)

    #plt.show(block=False)
    #plt.show()
    #plt.draw()
    #plt.pause(0.001)

def processBuffer(vehState):
    VEHICLE_WIDTH = 1.5*ct.METERS_PER_FOOT

    plotBuffer(vehState.lidarBuffer)
    
    # okay our goal here is primarily to test wall detection
    
    # first things first lets convert the lidarbuffer points into cartesian coordinates
    allAngles= np.expand_dims(np.linspace(0,2*math.pi,360),1)
    allDistances = np.expand_dims(vehState.lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE],1)
    points = np.concatenate((np.sin(allAngles)*allDistances, \
                             np.cos(allAngles)*allDistances \
                             ),axis=1)
    
    # now let's try linking all of the points that are within the vehicle width
    groups = np.zeros(allDistances.shape)
    for i in range(points.shape[0]):
        if not (groups[i] == 0) or (allDistances[i]==12):
            continue
        # this is an unallocated point of a distance shorter than 12

        # allocate this point
        groups[i]=np.max(groups)+1
        # find the unallocate points
        indices = np.nonzero((groups==0) * (allDistances < 12))[0]
        # calculate the distances from this point
        distances = np.sqrt(np.sum(np.square(points[indices,:]-points[i,:]),axis=1))
        # get a list of all unallocated points near this one
        toBeAdded = indices[np.where(distances<=VEHICLE_WIDTH)].tolist()
        # allocate them to prevent looping
        groups[toBeAdded]=groups[i]
        while len(toBeAdded):
            curIndex = toBeAdded.pop()
            # find the unallocate points
            indices = np.nonzero((groups==0) * (allDistances < 12))[0]
            # calculate the distances from this point
            distances = np.sqrt(np.sum(np.square(points[indices,:]-points[curIndex,:]),axis=1))
            # get a list of all unallocated points near this one
            newAdditions = indices[np.where(distances<=VEHICLE_WIDTH)].tolist()
            # allocate them to prevent looping
            groups[newAdditions]=groups[i]
            # add these elements to the existing list
            toBeAdded.extend(newAdditions)
            
    # now display the results
    for i in range(1,np.max(groups).astype(int).item()+1):
        indices = np.nonzero(groups==i)[0]
        if len(indices)>5:
            plt.plot(points[indices,0],points[indices,1],linestyle=' ',marker='o',markersize=10,color='g')
    plt.show()
    #plt.draw()
    plt.pause(0.001)        
    groups
        
        
###############################################################################
# Test code
###############################################################################
TEST = 1
NPY_DIR = "."
MAT_DIR = "./matlabCourse/"
if __name__ == '__main__':
    import sys
    plt.ion()
    #print(sys.paRth)

    ##########################################################################
    vehState = vs.vehicleState()
    


    if TEST==0: # run our manual points
        processBuffer(vehState)
    elif TEST == 1:
        # get a sorted listing of the .npy files
        dir = os.listdir(NPY_DIR)
        for i in range(len(dir)-1,-1,-1):
            if len(dir[i])<4:
                del dir[i]
            elif ".npy" not in dir[i]:
                del dir[i]
        # sort by digits
        dir.sort()
        # sort by length
        dir = sorted(dir, key=len)   
        
        # now load, display and run them
        print ("Number of npy files - ", len(dir))
        for file in dir:
            print(file)
            vehState.lidarBuffer = np.load(file)
            processBuffer(vehState)
            #plotBuffer(vehState.lidarBuffer)
            plt.pause(1)
    elif TEST == 2:
        # get a sorted listing of the .npy files
        dir = os.listdir(MAT_DIR)
        for i in range(len(dir)-1,-1,-1):
            if len(dir[i])<4:
                del dir[i]
            elif ".mat" not in dir[i]:
                del dir[i]
        # sort by digits
        dir.sort()

        # now load, display and run them
        print ("Number of mat files - ", len(dir))
        for file in dir:
            print(MAT_DIR+file)
            f = h5py.File(MAT_DIR+file,'r') 
            data = np.transpose(np.array(f.get('temp')))
            vehState.lidarBuffer = np.zeros((360,5))
            vehState.lidarBuffer[:,ct.LIDAR_BUFFER_ANGLE] = data[:,0]
            vehState.lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE] = data[:,1]
            
            processBuffer(vehState)
            #plotBuffer(vehState.lidarBuffer)
            plt.pause(.1)
    #vehState.lidarBuffer = np.load(dir[0])

# end