"""
 controller.py 
 Class to store the current state of the controller
 
 Written by Faye Cameron 11/2018
"""

import numpy as np
import math
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
    # the offset (global frame) between global and body
    currentOffset      = np.zeros((1,2)) 
    
    # The last seconds worth of scan ranges are stored in this buffer.   
    #iopRanges           = Range(40)
    # a circular buffer of the LIDAR readings
    lidarBuffer = 12*np.ones((180,5))
    #lidarBufferLock = threading.lock() # a lock to prevent race conditions
    cosines = np.zeros((181,1))
    sines = np.zeros((181,1))

    # Error handling
    errorString        = ""
    
    ###########################################################################
    # calcTargetAngle -
    ###########################################################################
    def calcTargetAngle(self, vehState):
        if self.cosines[0,0]==0:
            for i in range(181):
                self.cosines[i,0] = math.cos((i-90)/180.8*math.pi)
                self.sines[i,0] = math.cos((i-90)/180.8*math.pi)
            
        # this algorithm has two stages, the first finds the angles within +- 
        # 45 deg of our current heading that have the largest distance reading, 
        # and prefers angles that are closest to our current angle. The second 
        # searches within +- 45 deg of the first angle for the direction that 
        # we can go the farthest and not collide with stuff

        # get a local copy of the current angle
        #vehState.currentAngleLock.acquire()
        currentAngle = self.currentAngle
        #vehState.currentAngleLock.release()

        # now copy out the lidar readings, recentering on our current direction
        localLidar = np.zeros((180,1))
        #lidarBufferLock.acquire()
        for index in range(180):
            bufferIndex = int(round( \
                ((currentAngle - math.pi+(index-1)*math.pi*2/180) % \
                (2*math.pi))/(2*math.pi)*(len(vehState.lidarBuffer)-1)))
            #print("bufferIndex: ",bufferIndex," LocalIndex: ",index, "distance: ", \
            #    vehState.lidarBuffer[bufferIndex,ct.LIDAR_BUFFER_DISTANCE])
            #print(index,", ",bufferIndex,", ",ct.LIDAR_BUFFER_DISTANCE)
            localLidar[index]=vehState.lidarBuffer[bufferIndex,ct.LIDAR_BUFFER_DISTANCE]
        #lidarBufferLock.release()        
        
        # FIND THE FIRST ANGLE

        # find the direction that would get us the farthest with the least turning, assuming that we are a particle
        maxDistance = 0.
        bestDistanceIndex = 0
        for index in range(68,114): # 91 is the 0 angle, 91+- 23 is ~ +-45 deg
            if localLidar[index]>maxDistance:
                maxDistance=localLidar[index].astype(float)
                bestDistanceIndex=index
                continue
            if (localLidar[index]==maxDistance) and (abs(index-91)<abs(bestDistanceIndex-91)):
                bestDistanceIndex=index
                continue

        #print("maxDistance: ",maxDistance, " bestDistanceIndex: ",bestDistanceIndex) 
        # FIND THE SECOND ANGLE
        # convert the localLidar to max distance 
        obstacleDistance = localLidar.copy()
        # iterate over the potential directions (+-45 deg)
        for index in range(bestDistanceIndex-23,bestDistanceIndex+23):
            # iterate over the potential obstacles (+-90 deg)
            for subIndex in range(index-45,index+45):
                #print("sideDist: ",((localLidar[subIndex].astype(float))* \
                #    math.sin(2.*abs(index-subIndex)/180.*math.pi))," dist ", \
                #    (localLidar[subIndex].astype(float))," angle ",(2.*abs(index-subIndex)))
                if localLidar[subIndex]*self.sines[2*abs(index-subIndex)]< (9/39.3): # 9 inches
                    obstacleDistance[index]=min(obstacleDistance[index], \
                        localLidar[subIndex]*self.cosines[2*abs(index-subIndex)])

        # find the direction that would get us the farthest with the least turning, assuming that we are a particle
        maxDistance = 0.
        outputAngleIndex = 0
        for index in range(bestDistanceIndex-23,bestDistanceIndex+23): # 91 is the 0 angle, 91+- 23 is ~ +-45 deg
            #print("distToGo[",index,"] = ",obstacleDistance[index].astype(float));
            if obstacleDistance[index]>maxDistance:
                maxDistance=obstacleDistance[index]
                outputAngleIndex=index
                continue
            if (obstacleDistance[index]==maxDistance) and (abs(index-bestDistanceIndex)<abs(bestDistanceIndex-bestDistanceIndex)):
                outputAngleIndex=index
                continue

        print("maxDistance: ",maxDistance, " bestDistanceIndex: ",outputAngleIndex) 

        return -math.pi+(outputAngleIndex-1)*math.pi*2/180
    # end
    
    
# end class    
###############################################################################

###############################################################################
# Global variables 
###############################################################################
cont       = controller()
