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
from  Timeit import Timeit

from scipy import ndimage
from scipy import misc                      # scipy
#from skimage import *  # scikit-image
import h5py
import matplotlib.pyplot as plt             # matplotlib
from   math     import *
import vehicleState     as vs

###############################################################################
# Class Grid
###############################################################################

class Grid(object):
    """
    The Grid class stores an occupancy grid as a two dimensional array.
    Each cell in the grid can be thought of as a bin holding a position
    of an obstacle as detected by the sensor.  Each cell only holds one
    position; if another obstacle is detected in the same cell (in
    same vicinity) it is assumed to be the same obstacle.  The newer
    position replaces the older one, on the assumption that the vehicle
    is closer to the obstacle and thus the measurement is more accurate.

    (Possible future enhancement - look at the 8 surrounding cells to
    see if there is an entry within a set distance say 'resolution' and
    if so, assume that is the original for the current point)

    The origin (0,0) of the grid is considered to be the lower left
    corner with 'x' increasing to the right (increasing column index)
    and 'y' increasing going up (increasing row index).

    The grid is dynamic in that it is referenced to the vehicle.  As the
    vehicle moves the grid moves with it.  The distance and angle variables
    are the vehicle cumulative distance and angle which correspond to the
    current grid. Angle is assumed to be an absolute angle in world
    reference frame, something like we might obtain from a compass.

    Public instance variables:
        nCols      --  Number of columns in the occupancy grid.
        nRows      --  Number of rows in the occupancy grid.
        resolution --  Width & height of each grid square in cm.
        distance   --  The distance (position) this grid is referenced from
                       (The cumulative distance the car has travelled)
        angle      --  The angle this grid is referenced from
        grid       --  integer array with nRows rows and nCols columns.
        Xpos       --  The position of the car which the map is relative to
        Ypos       --  The position of the car which the map is relative to

    """
 
    ###########################################################################
    # __init__   Note - the position of the car when this map was created or
    # updated is always in the center and at Y = 0.0
    ###########################################################################
    def __init__(self, resolution=10, nRows=40, nCols=60, distance=0, angle=0):
        self.resolution = resolution
        self.nCols      = nCols
        self.nRows      = nRows
        self.Xpos       = nCols * resolution / 2
        self.Ypos       = 0.0

        self.host       = None          # Used for UDP telemetry
        self.port       = None
        self.sock       = None

        self.scanAngle  = 30
        self.angDelta   = 5
        self.minAngle   = -1 * self.scanAngle
        self.maxAngle   = self.scanAngle
        self.origin     = [0.5 * self.nCols * self.resolution, 0]
        self.maxDist    = (sqrt( (self.nRows/2)**2 + (self.nCols)**2 ) )
        
        # Create the binary grid to store the points
        self.binGrid = np.zeros( (self.nRows, self.nCols), dtype=np.uint16 )
        
        self.createStructElems()
        
        self.tmr = Timeit(0)
        
        # Create the binary array to send binGrid over UDP
        self.binArr =  [ 0 for x in range(self.nCols * self.nRows)] 
        
        # Create the histogram array
        self.histSize = int((self.scanAngle * 2) / self.angDelta) + 1
        self.histArr =  [ 0 for x in range(self.histSize)]

        self.clear (distance, angle)
    # end
    
    ###########################################################################
    # details   
    ###########################################################################
    def details (self):
        print ("BinGrid: Resolution %d cm, nRows %d, nCols %d\n" % 
                            (self.resolution, self.nRows, self.nCols), end='') 
    
    ###########################################################################
    # clear   Clears and re-initializes the grid
    ###########################################################################
    def clear (self, distance=0, angle= 0):
        self.distance   = distance
        self.angle      = angle
        
        # Create the binary grid to send as telemetry
        self.binGrid = np.zeros( (self.nRows, self.nCols), dtype=np.uint16 )
        #self.binGrid =  [[ False for x in range(self.nCols)] for y in range(self.nRows)]
    # end

    ###########################################################################
    # enterRange - enters a point into the map.  The vehicle only knows about
    # it's cumulative, and it's currAngle.  The scanner knows it's scan angle
    # and range distance.
    # Parameters:
    #   carCumDist  - cumulative distance the car has traveled
    #   carCurrAngle- current angle of the car relative to left/right wall (deg).
    #                 positive values are angled to the right
    #                 negative values are angled to the left
    #   scanDist    - range of the object (cm)
    #   scanAngle   - angle of scanner relative to the heading of the car (deg)
    #                 positive values are to the right
    #                 negative values are to the left
    ###########################################################################
    def enterRange (self,  carCumDist, carCurrAngle, scanDist, scanAngle):
        distTravelled = 0       # carCumDist   - self.distance
        angleDiff     = 0       # carCurrAngle - self.angle
        carXpos       = (sin(radians(angleDiff)) * distTravelled)
        carYpos       = (cos(radians(angleDiff)) * distTravelled)
        carXpos       += self.Xpos
        carYpos       += self.Ypos

        scanAngle     = angleDiff + scanAngle
        rangeXpos     = sin(radians(scanAngle)) * scanDist
        rangeYpos     = cos(radians(scanAngle)) * scanDist

        Xpos          = rangeXpos + carXpos
        Ypos          = rangeYpos + carYpos
        
        col = int (Xpos / self.resolution)
        row = int (Ypos / self.resolution)
        self.enterPoint (row,   col)
    
    # end

    ###########################################################################
    # enterPoint - enters an objects position into the grid
    ###########################################################################
    def enterPoint(self, row, col):
        #col = int (x / self.resolution)
        #row = int (y / self.resolution)

        if (col < 0 or col >= self.nCols):
            return
        if (row < 0 or row >= self.nRows):
            return

        self.binGrid[row, col] = 1
    # end

    ###########################################################################
    # printGrid -
    ###########################################################################
    def printGrid (self, str=""):
        print (str, end='\n')
        print ("       ", end='')
        for col in range (self.nCols):
            print ("%d" % (col % 10), end='')
        print("\n", end='')
        
        for row in range (self.nRows-1, -1, -1):
            print ("Row %2d:" % (row), end=''),
            for col in range (self.nCols):
                if (self.binGrid[row, col]):
                    print ("%s" % ("O"), end='')
                else:
                    print ("%s" % (" "), end='')
                # end if
            # end for col
            print ("\n", end='')
        # end for row
        print("\n", end='')
    # end
    
    ################################################################################
    # sendUDP()
    ################################################################################
    def sendUDP_init(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # end

    ################################################################################
    # sendUDP()
    ################################################################################
    def sendUDP(self, currTime, angle):
        PktId   = 0x55555555              # packet ID for the occupancy grid msg
        nRows   = self.nRows
        nCols   = self.nCols
        carXpos = int(self.Xpos)
        carYpos = int(self.Ypos)

        # Fill the binary array from the occupancy grid. At the same time
        # calculate the total number of non-zero cells and the (partial) checksum
        total       = 0               # total number of non-zero cells
        checksum    = 0
        for row in range (self.nRows):
            for col in range (self.nCols):
                if (self.isZero(row, col)):
                    val = 0
                else:
                    val = 1
                    total += 1                    
                # End if
                self.binArr[row * self.nCols + col] = val                    
                checksum += val
        # end for

        # Finish off the checksum
        checksum = PktId + currTime + nRows + nCols + carXpos + carYpos + angle

        # Create the packet to send
        # 'L' - ulong, i - int, 'h' - short, 'B' - uchar,
        # 'x' - char, 's', 'p' - string, pascal char[]
        #packetDesc = ( '>LLLLiii%dH' % (nRows * nCols) )
        #packetDesc = ( '>LLLLiii%dx' % (nRows * nCols) ) dnw expected 7
        packetDesc = ( '>LLLLiii%dB' % (nRows * nCols) )
        packetData = struct.pack(packetDesc, PktId, currTime, nRows, nCols,
                           carXpos, carYpos, angle, *self.binArr)

        # Send the packet!
        if (self.sock != None):
            self.sock.sendto(packetData, (self.host, self.port))
            print ('sendUDP - number of non-0 entries', total)         
    # end

    ###############################################################################
    # ProcessBuffer with Morphology
    ###############################################################################
    def createStructElems(self):

        # Structuring array for the dilation
        self.structElem50 = np.array ( [[ 0,  1,  1,  1,  0],
                                        [ 1,  1,  1,  1,  1],
                                        [ 1,  1,  1,  1,  1],
                                        [ 1,  1,  1,  1,  1],
                                        [ 0,  1,  1,  1,  0]] ) #, dtype=np.uint16) 
        self.structElem51 = np.array ( [[ 1,  1,  1,  1,  1],
                                        [ 1,  1,  1,  1,  1],
                                        [ 1,  1,  1,  1,  1],
                                        [ 1,  1,  1,  1,  1],
                                        [ 1,  1,  1,  1,  1]] ) #, dtype=np.uint16)                               
        self.structElem3 = np.ones( (3, 3), dtype=np.uint16 )
        
        # Second deriv operators    
        self.secderiv7 = np.array ( [[-2,  0,  0,  0,  0,  0, -2],
                                     [-2,  0,  0,  0,  0,  0, -2],    
                                     [-2,  0,  0,  0,  0,  0, -2],
                                     [-2,  0,  0, 28,  0,  0, -2],
                                     [-2,  0,  0,  0,  0,  0, -2],
                                     [-2,  0,  0,  0,  0,  0, -2],                            
                                     [-2,  0,  0,  0,  0,  0, -2]])
        self.secderiv5_1 = np.array ( [[-1, -2,  6, -2, -1],
                                       [-1, -2,  6, -2, -1],
                                       [-1, -2,  6, -2, -1],
                                       [-1, -2,  6, -2, -1],
                                       [-1, -2,  6, -2, -1]])                            
        self.secderiv5_2 = np.array ( [[-1,  0,  0,  0, -1],
                                       [-1, -2,  6, -2, -1],
                                       [-1, -2, 10, -2, -1],
                                       [-1, -2,  6, -2, -1],
                                       [-1,  0,  0,  0, -1]])                       
        self.secderiv3 = np.array ( [[-1,  2, -1],
                                     [-1,  2, -1],
                                     [-1,  2, -1] ])                           
        self.secderiv3h= np.array ( [[ 0,  0,  0],
                                     [-1,  2, -1],
                                     [ 0,  0,  0] ])                           
        # Diagonal Deriv operators
        self.diag0 = np.array ([ [-1, -2,  0],
                                 [-2,  0, +2],
                                 [ 0, +2, +1] ])
        self.diag1 = np.array ([ [ 0, +2, +1],
                                 [-2,  0, +2],
                                 [-1, -2,  0] ])   
    # end createStructElems
    
    ###############################################################################
    # ProcessBuffer with Morphology
    ###############################################################################
    def processGrid(self, vehState):

        ###########################################################################
        # Dilation
        self.tmr.start(0)
        self.tmr.start(10)
        self.dilatedgrid = ndimage.binary_dilation(self.binGrid, structure=self.structElem50, border_value=0) 
        self.tmr.stop(0)
        
        # invert
        self.tmr.start(1)
        self.notGrid = ~self.dilatedgrid
        self.tmr.stop(1)
        
        # skeletonize   
        self.tmr.start(2)
        self.distance = ndimage.distance_transform_cdt(self.notGrid, metric='taxicab', return_distances=True)
        self.tmr.stop(2)
    
        self.tmr.start(3)
        self.deriv = ndimage.convolve(self.distance, weights=self.secderiv5_1)
        self.derivThresh = np.where(self.deriv >= 2, 1, 0)    
        self.tmr.stop(3)
        self.tmr.stop(10)
    # end processBuffer    
    
    ###############################################################################   
    # printTimers - 
    ############################################################################### 
    def printTimers(self, title): 
        self.tmr.printAll(title)      
    
    ###############################################################################   
    # plotGrid - Consolidate all the plotting here...
    ############################################################################### 
    def plotGrid(self, title, binary=False, dilated=False, distance=False, 
                              deriv=False,  threshold=False):    
        # don't stall on putting up each figure - interactive mode on  
        plt.ioff()    
        
        if (binary):            # the original Grid
            plt.figure(0)
            plt.cla()
            plt.title(title + " - Grid")
            plt.imshow(self.binGrid, origin='lower') 
        
        if (dilated):           # the dilated grid
            plt.figure(1)
            plt.cla()
            plt.title(title + " - Dilated grid")
            plt.imshow(self.dilatedgrid, origin='lower')
                   
        if (distance):          # The distance transform
            plt.figure(2)
            plt.cla()        
            plt.title(title + " - Distance transform")
            plt.imshow(self.distance, origin='lower')
        
        if (deriv):             # the derivative
            plt.figure(3)
            plt.cla()
            plt.title(title + " - Derivative")
            plt.imshow(self.deriv, origin='lower')
        
        if (threshold):         # thesholded
            plt.figure(4)
            plt.cla()            
            plt.title(title + " - Threshold")
            plt.imshow(self.derivThresh, origin='lower')
            
        """         
        # the rows
        nRows = 10
        #plt.figure(4)
        figure, plots = plt.subplots(nRows)
        for i in range(nRows):
            row = 15 + 5*i
            plots[nRows-i-1].plot(distance[row])
            plots[nRows-i-1].set(ylabel='row ' + str(row))
            #plt.title("Distance Transform")
        plt.show()  
        plt.ion()            # Now wait till the user kills the plots 
        """
        
    ###############################################################################   
    # enterBufferPnts - enter the grid points from the vehState buffer
    ############################################################################### 
    def enterBufferPnts(self, vehState):
        distances = vehState.lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE]
        angles    = ct.RAD_TO_DEG * vehState.lidarBuffer[:,ct.LIDAR_BUFFER_ANGLE]
        
        for i in range(len(vehState.lidarBuffer)):
            dist = distances[i] * 100
            angle= angles[i]
            self.enterRange (0, 0, dist, angle)
            #print ("Entering Pt %3d: Distance: %6.2f Angle: %5.1f" % (i, dist, angle))
        
        """
        points = np.expand_dims(distances, 1) / ct.METERS_PER_FOOT * 3 * \
                np.transpose([np.cos(angles * ct.DEG_TO_RAD), np.sin(angles * ct.DEG_TO_RAD)])              
        for point in points:
            grid.enterPoint(point[0].astype(int),point[1].astype(int)+80)            
        """
    # end enterBufferPnts      
    
# end class
    
###############################################################################
# Test code
###############################################################################
TEST = 1
NPY_DIR = "."
MAT_DIR = "./matlabCourse/"
EXTENSION = ".mat"

if __name__ == '__main__': 

    ###############################################################################   
    # enterManualPnts0 - create a grid of points the hard way - earn it!
    ###############################################################################    
    def enterManualPnts0(grid):
        print ("enterManualPnts: Entering points manually") 
        lWall = 40
        rWall = 120
        
        if (1):
            for row in range(99): 
                # create a bit of uneveness.  Every 10 columns move over and 
                # every 25 columns move over some more
                if (row % 10 == 0):
                    lWall += 1
                    rWall += 1
                if (row % 25 == 0):
                    lWall += 2
                    rWall += 2   
                
                # Only enter every third point
                if (row % 3 == 0):
                    if (row < 60):
                        grid.enterPoint(row, lWall)
                        grid.enterPoint(row, rWall) 
                    else:
                        grid.enterPoint(row, rWall)            
                    
            # The horiz wall
            for col in range(55, 80, 2):
                grid.enterPoint(60, col)
            
            # the barrels
            grid.enterPoint(70, 95)
            grid.enterPoint(68, 96) 
            grid.enterPoint(69, 97)
            grid.enterPoint(69, 98) 
            grid.enterPoint(70, 99)     
        
            grid.enterPoint(68, 110)
            grid.enterPoint(67, 111) 
            grid.enterPoint(67, 112)
            grid.enterPoint(68, 113)   
            grid.enterPoint(69, 114)  
        else:
            lWall = 65
            rWall = 95    
            for row in range(99): 
                # create a bit of uneveness.  Every 10 columns move over and 
                # every 25 columns move over some more
                if (row % 10 == 0):
                    lWall += 1
                    rWall += 1
                if (row % 25 == 0):
                    lWall += 2
                    rWall += 2   
                
                # Only enter every third point
                if (row % 3 == 0):
                    if (row > 40 and row < 60):
                        continue
                    if (row < 80):
                        grid.enterPoint(row, lWall)
                        grid.enterPoint(row, rWall) 
                    else:
                        grid.enterPoint(row, rWall)            
                    
            # The horiz wall
            for col in range(55, 80, 1):
                if (col % 4 == 0):
                    grid.enterPoint(80, col)
                elif (col % 2 == 0):
                    grid.enterPoint(81, col)            
            
            # the barrels
            grid.enterPoint(85, 95)
            grid.enterPoint(83, 96) 
            grid.enterPoint(84, 97)
            grid.enterPoint(84, 98) 
            grid.enterPoint(85, 99)     
        
            grid.enterPoint(88, 110)
            grid.enterPoint(87, 111) 
            grid.enterPoint(87, 112)
            grid.enterPoint(88, 113)   
            grid.enterPoint(89, 114)      
    # end enterManualPnts 

    ###############################################################################   
    # enterManualPnts1 - create a grid of points the hard way - earn it!
    ###############################################################################
    def enterManualPnts1(grid):
        print ("enterManualPnts: Entering points manually") 
        lWall = 65
        rWall = 95    
        for row in range(99): 
            # create a bit of uneveness.  Every 10 columns move over and 
            # every 25 columns move over some more
            if (row % 10 == 0):
                lWall += 1
                rWall += 1
            if (row % 25 == 0):
                lWall += 2
                rWall += 2   
            
            # Only enter every third point
            if (row % 3 == 0):
                if (row > 40 and row < 60):
                    continue
                if (row < 80):
                    grid.enterPoint(row, lWall)
                    grid.enterPoint(row, rWall) 
                else:
                    grid.enterPoint(row, rWall)            
                
        # The horiz wall
        for col in range(80, 85, 1):
            if (col % 4 == 0):
                grid.enterPoint(80, col)
            elif (col % 2 == 0):
                grid.enterPoint(81, col)            
        
        # the barrels
        grid.enterPoint(85, 95)
        grid.enterPoint(83, 96) 
        grid.enterPoint(84, 97)
        grid.enterPoint(84, 98) 
        grid.enterPoint(85, 99)     
        
        grid.enterPoint(88, 105)
        grid.enterPoint(87, 106) 
        grid.enterPoint(87, 107)
        grid.enterPoint(88, 108)   
        grid.enterPoint(89, 109)      
    # end enterManualPnts1 
      
    ###############################################################################
    # getFileList
    ###############################################################################
    def getFileList(directory, extension):
        # get a sorted listing of the .<extension> files
        dir = os.listdir(directory)
        for i in range(len(dir)-1,-1,-1):
            if len(dir[i])<4:
                del dir[i]
            elif extension not in dir[i]:
                del dir[i]
        # sort by digits
        dir.sort()
        return dir
    # end getFileList       

    ###############################################################################
    # TEST 1 - Get all the data from the .mat files
    ###############################################################################    
    if (TEST == 1):
        import os as os
        import sys
    
        ##########################################################################
        vehState = vs.vehicleState()
        grid = Grid(10, nRows=100, nCols=180, distance=0, angle=0)
        grid.details()
        
        # get a sorted listing of the .npy files
        dir = getFileList(MAT_DIR, EXTENSION)
               
        # now load, display and run them
        print ("Number of %s files: %d " % (EXTENSION, len(dir)) )
        for file in dir:           
            print(MAT_DIR+file)
            f = h5py.File(MAT_DIR+file,'r') 
            data = np.transpose(np.array(f.get('temp')))
            vehState.lidarBuffer = np.zeros((360,5))
            vehState.lidarBuffer[:,ct.LIDAR_BUFFER_ANGLE] = data[:,0]
            vehState.lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE] = data[:,1]
            
            grid.enterBufferPnts(vehState)
            grid.processGrid(vehState)
            grid.plotGrid(file, binary=True, dilated=False, distance=False, 
                                deriv=True, threshold=True)
                                
            # slow down at the hard parts of the track
            fileNo = int(file[0:3]) 
            if ( (fileNo > 60  and fileNo <  70) or
                 (fileNo > 105 and fileNo < 120) or
                 (fileNo > 170 and fileNo < 185) or
                 (fileNo > 195 and fileNo < 200) ):
                plt.pause(1.5)
            else: 
                plt.pause(.1)            
            grid.clear()     
        
        grid.printTimers("Grid Processing:")
    # end TEST 1
    
    ###############################################################################
    # TEST 2 -  Use the manually generated points
    ###############################################################################     
    elif (TEST == 2):
        enterManualPnts1 (grid)
        processGrid(vehState, grid)    
        
    ###############################################################################
    # TEST ERROR 
    ###############################################################################         
    else:
        print ("ERROR - no valid TEST defined")
    
