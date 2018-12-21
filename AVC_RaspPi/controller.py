"""
 controller.py 
 Class to store the current state of the controller
 
 Written by Faye Cameron 11/2018
"""

import numpy as np
import math as math
import matplotlib.pyplot as plt
import time as time
#from rangeClass      import Range

import constants as ct        # Vehicle and course constants
import vehicleState as vs          # Everything we know about the vehicle

###############################################################################
# class controller - 
###############################################################################
class controller (object):
    timeSinceStart     = 0.0   # Seconds since the start signal arrived    
    timeAtStart        = 0.0   # Time when start signal arrived
    distAtStart        = 0.0   # Distance measured when start signal arrived
    compassAtStart     = 0.0   # Compass angle when start signal arrived
    
    # map state information
    #currentAngleLock   = threading.lock() # a lock to prevent race conditions
    currentAngle       = 0.0 # the angle between the global frame and the body's
    # the offset distance (global frame) between global and body
    currentOffset      = np.zeros((1,2)) 
    
    # The last seconds worth of scan ranges are stored in this buffer.   
    #iopRanges           = Range(40)
    # a circular buffer of the LIDAR readings
    #lidarBuffer = 12*np.ones((360,5))
    #lidarBufferLock = threading.lock() # a lock to prevent race conditions
    cosines = np.zeros((181,1))
    sines = np.zeros((181,1))

    # Error handling
    errorString        = ""
    
    def translateCommand(self, angle, speed):
        # turn rate = speed / (wheel-base length) * sin(wheel rotation)

        # target a turn period of .5 sec
        # desired turn rate
        dTR = angle/.5
        # desired wheel rotation
        dWR = math.asin(dTR*ct.DEG_TO_RAD * ct.wheelBase / speed)
        return dWR


    ###########################################################################
    # calcTargetAngle -
    ###########################################################################
    def calcTargetAngle(self, vehState, leftAngle, rightAngle):
        # this algorithm has one stage, find the farthest distance that we can
        # go within the bounds represented by left angle and right angle without
        #  colliding with stuff.

        # get a local copy of the current angle
        currentAngle = self.currentAngle

        # now copy out the lidar readings, recentering on our current direction
        localLidar=np.roll(vehState.lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE],180)

        angles = np.arange(rightAngle,leftAngle+1,1).astype(int)

        # FIND THE distances
        # convert the localLidar to max distance 
        #print("angles: ",angles)
        #print("localLidar: ", localLidar)
        #print("self.sineMatrix.shape: ",self.sineMatrix.shape)
        obstaclePerpDistance = self.sineMatrix[90+angles,:]*localLidar
        obstacleTangDistance = self.cosineMatrix[90+angles,:]*localLidar
        # set all values where we wouldn't run into a specific obstacle to 12 m
        #print(obstacleTangDistance.shape)
        obstacleTangDistance = np.where(np.abs(obstaclePerpDistance)>=ct.vehicleWidth, 12, obstacleTangDistance)
        
        #print(obstacleTangDistance.shape)
        obstacleTangDistance = np.nanmin(obstacleTangDistance, axis=1)
        
        # find the direction that would get us the farthest with the least turning, assuming that we are a particle
        maxDistance = 0.
        outputAngle = 0
        #print(obstacleTangDistance)
        # get the indices that have the maximum length
        indices = np.nonzero(obstacleTangDistance==np.nanmax(obstacleTangDistance))[0]
        #print(type(indices), "indices: ",indices)
        #print("angles[indices]: ",(np.abs(angles[indices])==np.nanmin(np.abs(angles[indices]))))
        #print("angles[indices]: ",np.nonzero(np.abs(angles[indices])==np.nanmin(np.abs(angles[indices])))[0].astype(int).item(0))
        bestIndex = np.nonzero(np.abs(angles[indices])==np.nanmin(np.abs(angles[indices])))[0].astype(int).item(0)
        # get the maximum length index that is closest to forward
        bestIndex = indices[bestIndex]
        
        # record for plotting and output
        maxDistance=obstacleTangDistance[bestIndex]
        outputAngle=angles[bestIndex]
        
        np.save(str(time.clock())+".npy",vehState.lidarBuffer)        

        print("maxDistance: ",maxDistance, " outputAngle: ",outputAngle) 
        if ct.DEVELOPMENT:
            bestAngle=(outputAngle*ct.DEG_TO_RAD) % (2*math.pi)
            #print("maxDistance: ",maxDistance, " bestAngle: ",bestAngle) 
            plt.plot((0,maxDistance*math.cos(bestAngle)),(0,maxDistance*math.sin(bestAngle)),linestyle='-',color='b')
            #plt.plot((0,12),(0,0),linestyle=':')
            plt.title(("maxDistance: ",maxDistance, " outputAngle: ",outputAngle))
            plt.show()
            plt.pause(0.001)

        return outputAngle // 3 #self.translateCommand(outputAngle, ct.speedMax)
    # end
    
    def __init__(self):
        for i in range(181):
            self.cosines[i,0] = math.cos((i-90)*ct.DEG_TO_RAD)
            self.sines[i,0] = math.sin((i-90)*ct.DEG_TO_RAD)
        self.cosineMatrix = np.zeros((181,360))
        self.sineMatrix = 1E2 * np.ones((181,360))
        for i in range(180):
            #print(i)
            #print("range: ",self.sineMatrix[i, 360])
            self.sineMatrix[i, i:(181+i)] = np.squeeze(self.sines)
            self.cosineMatrix[i, i:(181+i)] = np.squeeze(self.cosines)
        # the last line overlaps with the first entry, so do it by hand
        self.sineMatrix[180, 180:360] = np.squeeze(self.sines[0:180,0])
        self.cosineMatrix[180, 180:360] = np.squeeze(self.cosines[0:180,0])
        self.sineMatrix[180, 0] = np.squeeze(self.sines[180,0])
        self.cosineMatrix[180, 0] = np.squeeze(self.cosines[180,0])
        #print("sine row 1: ",self.sineMatrix[180,:])
        #print("cosine row 1: ",self.cosineMatrix[180,:])
    #end   
    
# end class    
###############################################################################
