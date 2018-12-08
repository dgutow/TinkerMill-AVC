"""
 controller.py 
 Class to store the current state of the controller
 
 Written by Faye Cameron 11/2018
"""

import numpy as np
import math
import matplotlib.pyplot as plt
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
    
    ###########################################################################
    # calcTargetAngle -
    ###########################################################################
    #@profile
    def calcTargetAngle(self, vehState):
        if self.cosines[0,0]==0:
            for i in range(181):
                self.cosines[i,0] = math.cos((i-90)*ct.DEG_TO_RAD)
                self.sines[i,0] = math.sin((i-90)*ct.DEG_TO_RAD)
            
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
        #localLidar = np.zeros((360))
        #lidarBufferLock.acquire()
        #for index in range(360):
            #bufferIndex = int(round( \
            #    ((currentAngle - 180 + index) % 360) ))
            #print("bufferIndex: ",bufferIndex," LocalIndex: ",index, "distance: ", \
            #    vehState.lidarBuffer[bufferIndex,ct.LIDAR_BUFFER_DISTANCE])
            #print(index,", ",bufferIndex,", ",ct.LIDAR_BUFFER_DISTANCE)
            #localLidar[index]=vehState.lidarBuffer[bufferIndex,ct.LIDAR_BUFFER_DISTANCE]
        #print(indices)
        localLidar=np.roll(vehState.lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE],180)
        #lidarBufferLock.release()        
        
        # FIND THE FIRST ANGLE

        # find the direction that would get us the farthest with the least turning, assuming that we are a particle
        maxDistance = 0.
        bestDistanceIndex = 0
        # 180 is the 0 angle, 180+- 45 is the range we want to consider
        for index in range(180-45,180+45): 
            if localLidar[index]>maxDistance:
                maxDistance=localLidar[index].astype(float)
                bestDistanceIndex=index
                continue
            if (localLidar[index]==maxDistance) and (abs(index-180)<abs(bestDistanceIndex-180)):
                bestDistanceIndex=index
                continue

        print("maxDistance: ",maxDistance, " bestDistanceIndex: ",bestDistanceIndex) 
        # FIND THE SECOND ANGLE
        # convert the localLidar to max distance 
        obstacleDistance = localLidar.copy()
        vehicleWidth=9/12*ct.METERS_PER_FOOT
        # iterate over the potential directions (+-45 deg)
        for index in range(bestDistanceIndex-45,bestDistanceIndex+45):
            # iterate over the potential obstacles (+-90 deg)
            closeness = np.abs(np.multiply(np.squeeze(self.sines),localLidar[index-90:index+91]))
            dist2close = np.multiply(np.squeeze(self.cosines),localLidar[index-90:index+91])
            collisions=np.where(closeness<vehicleWidth)
            #print("shapes")
            #print(self.sines.shape)
            #print(localLidar[index-90:index+90].shape)
            #print(closeness.shape)
            #print(dist2close.shape)
            obstacleDistance[index]=np.amin(dist2close[collisions])
            """
            for subIndex in range(index-90,index+90):
                if index==subIndex:
                    continue
                #print("sideDist: ",((localLidar[subIndex].astype(float))* \
                #    math.sin(abs(index-subIndex)*ct.DEG_TO_RAD))," dist ", \
                #    (localLidar[subIndex].astype(float))," angle ",(abs(index-subIndex)))
                # if we would run into the obstacle sooner than our 
                # distance, then lower our distance
                #print("index: ",index,"subIndex: ",subIndex,"sideDist: ",localLidar[subIndex],"sin ",self.sines[90-(index-subIndex)], " cos ",self.cosines[90-abs(index-subIndex)]," curDist: ",obstacleDistance[index]," angle: ",(index-subIndex))
                if (localLidar[subIndex]*abs(self.sines[90-(index-subIndex)]) < \
                    (9/12*ct.METERS_PER_FOOT)) and \
                    (obstacleDistance[index] > localLidar[subIndex]*self.cosines[90-(index-subIndex)]): 
                    #print("Yindex: ",index,"subIndex: ",subIndex,"sideDist: ",localLidar[subIndex],"sin ",self.sines[90-(index-subIndex)], " cos ",self.cosines[90-abs(index-subIndex)]," curDist: ",obstacleDistance[index]," angle: ",(index-subIndex))
                    obstacleDistance[index]= \
                        localLidar[subIndex]*self.cosines[90-(index-subIndex)]
                #else:
                #    if (171==index):
                #        print("Nindex: ",index,"subIndex: ",subIndex,"sideDist: ",localLidar[subIndex],"sin ",self.sines[90-(index-subIndex)], " cos ",self.cosines[90-abs(index-subIndex)]," curDist: ",obstacleDistance[index]," angle: ",(index-subIndex))
            """
                            

        #print("localLidar")
        #print(localLidar[bestDistanceIndex-45:bestDistanceIndex+45])
        #print("obstacleDistance")
        #print(obstacleDistance[bestDistanceIndex-45:bestDistanceIndex+45])

        # find the direction that would get us the farthest with the least turning, assuming that we are a particle
        maxDistance = 0.
        outputAngleIndex = 0
        # bestDistanceIndex is the 0 angle, bestDistanceIndex+- 45
        for index in range(bestDistanceIndex-45,bestDistanceIndex+45): 
            #print("distToGo[",index,"] = ",obstacleDistance[index].astype(float));
            if obstacleDistance[index]>maxDistance:
                maxDistance=obstacleDistance[index]
                outputAngleIndex=index
                continue
            if (obstacleDistance[index]==maxDistance) and (abs(index-bestDistanceIndex)<abs(outputAngleIndex-bestDistanceIndex)):
                outputAngleIndex=index
                continue

        print("maxDistance: ",maxDistance, " outputAngleIndex: ",outputAngleIndex) 

        angle = (outputAngleIndex-180)
        if ct.DEVELOPMENT:
            bestAngle=(bestDistanceIndex-180)*ct.DEG_TO_RAD
            plt.plot((0,12*math.cos(bestAngle)),(0,12*math.sin(bestAngle)),linestyle=':',color='b')
            plt.plot((0,maxDistance*math.cos(angle*ct.DEG_TO_RAD)),(0,maxDistance*math.sin(angle*ct.DEG_TO_RAD)),linestyle='-',color='b')
            #plt.plot((0,12),(0,0),linestyle=':')
            plt.show()
            plt.pause(0.1)
        return -angle
    # end
    
    
# end class    
###############################################################################

###############################################################################
# Global variables 
###############################################################################
cont       = controller()
