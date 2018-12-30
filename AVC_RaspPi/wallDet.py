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

def plotBuffer(lidarBuffer, file):
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
    plt.title(file)

    #plt.show(block=False)
    plt.show()
    #plt.draw()
    plt.pause(0.001)

def processBuffer(vehState):
    plotBuffer(vehState.lidarBuffer)
    
    # okay our goal here is primarily to test if we can do quadratic fitting of points
    
    # first things first lets convert the lidarbuffer points into cartesian coordinates
    #angles= np.expand_dims(np.linspace(0,2*math.pi,360),1)
    #distances = np.expand_dims(vehState.lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE],1)
    #points = np.concatenate((np.sin(angles)*distances, \
    #                         np.cos(angles)*distances \
    #                         ),axis=1)
    
    # now let's try a toy problem, create a quadratic fit to the first 5 points
    # set the mean angle to 90 deg so that we get an optimally 
    
###############################################################################
# Test code
###############################################################################
TEST = 2
NPY_DIR = "."
MAT_DIR = "./matlabCourse/"
if __name__ == '__main__':
    import sys
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
        plt.ion()
        for file in dir:
            print(MAT_DIR+file)
            f = h5py.File(MAT_DIR+file,'r') 
            data = np.transpose(np.array(f.get('temp')))
            vehState.lidarBuffer = np.zeros((360,5))
            vehState.lidarBuffer[:,ct.LIDAR_BUFFER_ANGLE] = data[:,0]
            vehState.lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE] = data[:,1]
            
            #processBuffer(vehState)
            plotBuffer(vehState.lidarBuffer, file)
            plt.pause(0.1)
    #vehState.lidarBuffer = np.load(dir[0])

# end
