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

from scipy import ndimage
from scipy import misc                      # scipy
#from skimage import *  # scikit-image

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
        #self.binGrid =  [[ False for x in range(self.nCols)] for y in range(self.nRows)]
        
        # Create the binary array to send binGrid over UDP
        self.binArr =  [ 0 for x in range(self.nCols * self.nRows)] 
        
        # Create the histogram array
        self.histSize = int((self.scanAngle * 2) / self.angDelta) + 1
        self.histArr =  [ 0 for x in range(self.histSize)]

        # Create the cost and angle arrays and calculate their values
        self.costArr =  [[ 0 for x in range(self.nCols)] for y in range(self.nRows)]
        self.angArr  =  [[ 0 for x in range(self.nCols)] for y in range(self.nRows)]

        # NOTE - don't do row 0 or there will be a divide by 0 at the origin
        for row in range(1, self.nRows):
            for col in range(self.nCols):
                angIndex = self.getAngleIndex(row, col)
                if (angIndex == -1):
                    self.angArr[row][col]  = 0
                    self.costArr[row][col] = 0
                else:
                    self.angArr[row][col]  = angIndex
                    self.costArr[row][col] = self.getCost(row, col)

        # Now fill in row 0 with all 0 values
        for col in range(self.nCols):
            self.costArr[0][col] = 0
            self.angArr[0][col]  = 0

        self.clear (distance, angle)
    # end
    
    ###########################################################################
    # details   
    ###########################################################################
    def details (self):
        print ("Resolution %d, nRows %d, nCols %d\n" % (self.resolution, self.nRows, self.nCols), end='') 
    
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
        
        #self.enterPoint (row+1, col+1)
        #self.enterPoint (row,   col+1)
        #self.enterPoint (row-1, col+1)
        #self.enterPoint (row+1, col)
        self.enterPoint (row,   col)
        #self.enterPoint (row-1, col)
        #self.enterPoint (row+1, col-1)
        #self.enterPoint (row,   col-1)
        #self.enterPoint (row-1, col-1)        
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

    
    ###########################################################################
    # getCost calculates the cost from the specified row/col to the origin
    ###########################################################################
    def getCost(self, row, col):
        dist = sqrt ( (row) ** 2 + (self.nCols/2 - col) ** 2 )
        cost = self.maxDist * (1 - dist/self.maxDist)
        return cost
    # end

    ###########################################################################
    # getAngleIndex calculates the angle from the specified row/col to the origin
    ###########################################################################
    def getAngleIndex(self, row, col):
        angle = degrees( atan( (col - self.nCols/2)/  row ) )
        angIndex = int( (angle - self.minAngle) / self.angDelta )
        if angIndex < 0 or angIndex >= self.histSize:
            return -1
        else:
            return angIndex
    # end
    
    ###########################################################################
    # getNearestAngle -
    ###########################################################################
    def getNearestAngle(self):
        # Zero out any old results in the histogram
        self.histArr =  [ 0 for x in range(self.histSize)]

        # Fill the histArr with the cost of each grid point
        for row in range(self.nRows):
            for col in range(self.nCols):
                if not (self.isZero(row, col)):
                    cost = self.costArr[row][col]
                    angleIndex = self.angArr[row][col]
                    self.histArr[angleIndex] += cost

        self.printHistArr()
        self.lowPassFilter(3)               # Not really necessary
        
        minCost = min(self.histArr)         # Find the minimum(s)      
        runs = self.getRuns(minCost)        # Find all the runs at this minimum
        
        return self.findBestAngle(minCost)
    # end

        
    ###########################################################################
    # lowPassFilterDag - to eliminate naughty zeros
    ###########################################################################
    def lowPassFilter(self, size):
        histArrayFiltered =  [ 9.0 for x in range(self.histSize)]
        halfSize = int(floor(size/2))
        
        for index in range(halfSize, self.histSize - halfSize):
            min = (index - halfSize)
            max = (index + halfSize + 1)
            ave = sum( self.histArr[min:max] ) / size    
            histArrayFiltered[index] =  ave            
            
        for index in range (0, halfSize):      
            histArrayFiltered[index] = histArrayFiltered[halfSize] 
            
        for index in range ( (self.histSize - halfSize), self.histSize):
            histArrayFiltered[index] = histArrayFiltered[self.histSize - halfSize - 1]
            
        self.histArr = histArrayFiltered        

    ###########################################################################
    # getRuns - finds all the runs of minimums in the hist array
    ###########################################################################
    def getRuns(self, minCost):
        inRun = False           # are we in a run of minimums?
        run   = []              # a single run.  elem 0 start of run, elem 1 end
        runs  = []              # the collection of runs
        
        for index in range(self.histSize):
            if not inRun and self.histArr[index] != minCost: 
                 # we not in a run and not at minimum cost?
                pass # don't do anything, we don't care!    
                                       
            elif inRun and self.histArr[index] == minCost: 
                # we in a run and still at minimum cost so still in the run
                pass 
                
            elif inRun and self.histArr[index] != minCost: 
                # we were in a run but now we're not anymore
                run.append(index-1) # Record the last index of the run
                runs.append(run)    # then stash this run to the collection
                run = []            # clear out the last collected run
                inRun = False       # we're not in a run anymore
                
            elif not inRun and self.histArr[index] == minCost: 
                # We weren't in a run but this is the start of one
                run.append(index)   # record the start index of this run
                inRun = True        # we're in a run now
        # end for
        
        if inRun: 
            # If we're done iterating but we're still in a run
            run.append(self.histSize-1)
            runs.append(run)        # stash this last run to the collection
            
        return runs
    #end getRuns
    
    ###########################################################################
    # getRuns - finds the 'best' run of minimums and then the actual angle
    ###########################################################################    
    def getBestRun(self, runs):
        longestRun      = 0
        longestRunIndex = 0
        for index, run in enumerate(runs):
            runLength = run[1] - run[0] + 1
            if runLength > longestRun:
                longestRun = runLength
                longestRunIndex = index
        
        bestRun     = runs[longestRunIndex]
        bestIndex   = int( (bestRun[0] + bestRun[1]) / 2)
        anglePos    = self.minAngle + (self.angDelta * bestIndex)        
        return anglePos

    ###########################################################################
    # printHistArr -
    ###########################################################################
    def printHistArr(self):
        for index in range(self.histSize):
            sys.stdout.write("%5.1f" % (self.histArr[index]))
        print ("\n")
        for index in range(self.histSize):
            sys.stdout.write("%5d" % (index))
        print ("\n")
    # end printHistArr
    ## old Histogram functions end

# end class



###############################################################################
# Test code
###############################################################################
TEST = 1
NPY_DIR = "."

if __name__ == '__main__':
    import os as os
    import sys
    #print(sys.path)

    ##########################################################################
    vehState = vs.vehicleState()
    
    # get a sorted listing of the .npy files
    dir = os.listdir(NPY_DIR)
    for i in range(len(dir)-1,-1,-1):
        # print (dir[i])
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
        plotBuffer(vehState.lidarBuffer)
    # end   
    
    """
    plt.cla - clear current axes
    plt.close - close a figure
    plt.pause(0.5)
    """

def processBuffer(vehState):
    ###########################################################################
    # Structuring array for the dilation
    structElem5 = np.array ( [[ 0,  1,  1,  1,  0],
                              [ 1,  1,  1,  1,  1],
                              [ 1,  1,  1,  1,  1],
                              [ 1,  1,  1,  1,  1],
                              [ 0,  1,  1,  1,  0]], dtype=np.uint16)        
    structElem3 = np.ones( (3, 3), dtype=np.uint16 )
    
    # Second deriv operators    
    secderiv7 = np.array ( [[-2,  0,  0,  0,  0,  0, -2],
                            [-2,  0,  0,  0,  0,  0, -2],    
                            [-2,  0,  0,  0,  0,  0, -2],
                            [-2,  0,  0, 28,  0,  0, -2],
                            [-2,  0,  0,  0,  0,  0, -2],
                            [-2,  0,  0,  0,  0,  0, -2],                            
                            [-2,  0,  0,  0,  0,  0, -2]])
    secderiv5 = np.array ( [[ 0,  0,  0,  0,  0],
                            [-2,  0,  4,  0, -2],
                            [-2,  0,  4,  0, -2],
                            [-2,  0,  4,  0, -2],
                            [ 0,  0,  0,  0,  0]])
    secderiv3 = np.array ( [[-1,  2, -1],
                            [-2,  4, -2],
                            [-1,  2, -1] ])                           
    secderiv3 = np.array ( [[0,  0, 0],
                            [-1,  2, -1],
                            [0,  0, 0] ])                           
    # Diagonal Deriv operators
    diag0 = np.array ([ [-1, -2,  0],
                        [-2,  0, +2],
                        [ 0, +2, +1] ])
    diag1 = np.array ([ [ 0, +2, +1],
                        [-2,  0, +2],
                        [-1, -2,  0] ])   
    ###########################################################################
    
    if (0):
        plt.ion()
        plt.plot([1.6, .27])
        plt.show()
        plt.pause(1.5)
        sys.exit()       
        
    if (0):        
        f = misc.face()
        plt.imshow(f)
        plt.show()    
        sys.exit()
    # endif

    g = Grid(10, nRows=100, nCols=160, distance=0, angle=0)
    g.details()
    
    if np.sum(vehState.lidarBuffer)<1: # manually enter a set of lidar readings
        lWall = 40
        rWall = 120
        
        for row in range(99): 
            if (row % 10 == 0):
                lWall += 1
                rWall += 1
            if (row % 25 == 0):
                lWall += 2
                rWall += 2   
            if (row % 3 == 0):
                if (row < 60):
                    g.enterPoint(row, lWall)
                    g.enterPoint(row, rWall) 
                else:
                    g.enterPoint(row, rWall)            
                
        for col in range(55, 80, 2):
            g.enterPoint(60, col)
    
        g.enterPoint(70, 95)
        g.enterPoint(68, 96) 
        g.enterPoint(69, 97)
        g.enterPoint(69, 98) 
        g.enterPoint(70, 99)     
        
        g.enterPoint(68, 110)
        g.enterPoint(67, 111) 
        g.enterPoint(67, 112)
        g.enterPoint(68, 113)   
        g.enterPoint(69, 114)       
    else:
        points = np.expand_dims(vehState.lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE],1) / ct.METERS_PER_FOOT * 3 * \
                np.transpose([np.cos(vehState.lidarBuffer[:,ct.LIDAR_BUFFER_ANGLE]*ct.DEG_TO_RAD),np.sin(vehState.lidarBuffer[:,ct.LIDAR_BUFFER_ANGLE]*ct.DEG_TO_RAD)])
        for point in points:
            g.enterPoint(point[0].astype(int),point[1].astype(int)+80)
        plt.figure(5)
        plt.cla()
        plt.plot(points[:,0] / 3 * ct.METERS_PER_FOOT, points[:,1] / 3 * ct.METERS_PER_FOOT,linestyle=' ',marker='.',markersize=5,color='k')

        plt.xlim((0,12))
        plt.ylim((-12,12))
        plt.grid(True)

        plt.show()
        plt.pause(.1)
        
    
    ###########################################################################
    plt.ion()            # don't stall on plt.show() - interactive mode on
    fig, ax = plt.subplots()    
    plt.imshow(g.binGrid, origin='lower')
    plt.show()  
    plt.pause(1.5)    

    # Dilation
    start_time = time.perf_counter()
    g.binGrid = ndimage.binary_dilation(g.binGrid, structure=structElem5, border_value=0) 
    plt.figure(1)
    plt.imshow(g.binGrid, origin='lower')
    plt.show()
    plt.pause(0.001)

    
    # invert
    notGrid = ~g.binGrid
    #plt.imshow(notGrid, origin='lower')
    #plt.show()    
    
    # skeletonize    
    distance = ndimage.distance_transform_cdt(notGrid, metric='taxicab', return_distances=True)
    end_time = time.perf_counter()
    print ("processing time  ", end_time - start_time)
    
    plt.figure(2)
    plt.imshow(distance, origin='lower')
    plt.show()
    plt.pause(0.001)
    
    deriv = ndimage.convolve(distance, weights=secderiv3)
    #deriv = np.where(distance > 4, 1, 0)
    plt.figure(3)
    plt.imshow(deriv, origin='lower')
    plt.show()    
    plt.pause(0.001)

 
    g.histSize = 12
    g.histArr =  [ 0.0, 4.0, 4.0, 5.0, 6.0, 4.0, 0.0, 0.0, 0.0, 5.0, 5.0, 0.0]
    #g.histArr  =  [ 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
    #g.lowPassFilterDag(5)   
    
    g.printHistArr
    minCost = min(g.histArr)         # Find the minimum(s)      
    runs = g.getRuns(minCost)        # Find all the runs at this minimum
    print (runs)
    angle = g.getBestRun(runs)
    print ("angle = ", angle)

###############################################################################
# Test code
###############################################################################
TEST = 1
NPY_DIR = "."

if __name__ == '__main__':
    import sys
    #print(sys.paRth)

    ##########################################################################
    vehState = vs.vehicleState()
    
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

    if 0: # run our manual points
        processBuffer(vehState)
    else:
        # now load, display and run them
        print ("Number of npy files - ", len(dir))
        for file in dir:
            print(file)
            vehState.lidarBuffer = np.load(file)
            processBuffer(vehState)
            #plotBuffer(vehState.lidarBuffer)
            plt.pause(1)
    #vehState.lidarBuffer = np.load(dir[0])

# end  
    
    
    
    
    
    
    
    

"""
    import time

    np.set_printoptions(precision=3, linewidth=2000, threshold=np.nan, suppress=True)

    if (False):
        # RPLIDAR A2 scanner radius is 12 meters max
        maxWidth = 16 # meters
        desired_columns = 60
        res = (maxWidth * 100) / desired_columns # cm/Grid
 
        g = Grid(res, nRows=desired_columns, nCols=desired_columns, distance=0, angle=0)

        # # TEST SCRIPT - Draw a left wall
        # for y in range(3000):
        #     g.enterPoint(y*tan(radians(40)), y)

        s = LIDAR(portname='/dev/ttyUSB0')

        for j in range(10):
            obstacles = s.scan() # returns an array of one rotation of obstacles

            for i in obstacles:
                g.enterRange(0, 0, i[1] / 10, i[0])

        # scanAngle or "Cone": +/- (deg)
        # angDelta or "Slice": (deg)
        # minCost - smallest cost to display representing no object detected (recommended set to 0)
        # maxCost - largest cost to display representing imminent collision (recommended set to 9 max)
        h = Histogram(origin=[0.5 * g.nCols * g.resolution, 0], scanAngle=45, angDelta=3)

        # print(h.printSimpleGrid(g))
        costArray = h.getCostArray(g, maxDist, h.scanAngle, h.angDelta)
        # print(costArray)
        # OUTPUT
        output = h.getAngle(costArray, 0)

        print(output)
        pass
    else:           # DAG
        rows = 100
        cols = 160
        res  = 10   # resolution 10 cm

        grid = Grid(res, rows, cols, 0, 0)

        #initGraphGrid (self, str, nPix, borders = False, circle = False):
        #graphGrid (self, color="red"):
        #enterRange (self,  carCumDist, carCurrAngle, scanDist, scanAngle):
        #enterPoint(self, x, y):

        multiplier = 1
        middle = (cols / 2) * res

        for y in range(0, rows * res, res):
            if y <= rows * res * 0.5:
                grid.enterPoint((0 * y) + (middle-300), y)
                grid.enterPoint((0 * y) + (middle+300), y)
            else:
                grid.enterPoint((multiplier * y) + (middle-200), y)
                grid.enterPoint((-multiplier * y) + (middle+200), y)

        for y in range(0, rows * res, res):
            grid.enterPoint((-multiplier * y) + (middle+100), y + 800)
            grid.enterPoint((multiplier * y) + (middle+100), y + 800)

        grid.initGraphGrid ("Testing", 4, borders = False, circle = False)
        grid.graphGrid (color="red")

        time.sleep(3)
        #exit()

        # OUTPUT
        output = grid.getNearestAngle(0)

        print(output)
    # end if
"""
