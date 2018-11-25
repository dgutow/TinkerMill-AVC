#!/usr/bin/env python
""" Simple occupancy-grid-based mapping.

Author: David Gutow, Rich Paasch
Version: 8/2018
"""

import sys
import socket
import struct
import numpy as np
from   math     import *


from LIDAR      import *
from graphics   import *        # dag - remove before flight
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
    def __init__(self, resolution=10, nRows=50, nCols=50, distance=0, angle=0):
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

        # Create the histogram array
        self.histSize = int((self.scanAngle * 2) / self.angDelta) + 1
        self.histArr =  [ 0 for x in range(self.histSize)]

        # Create the cost and angle arrays and calculate their values
        self.costArr =  [[ 0 for x in range(self.nCols)] for y in range(self.nRows)]
        self.angArr  =  [[ 0 for x in range(self.nCols)] for y in range(self.nRows)]
        
        # Create the binary array so send UDP
        self.binArr =  [ 0 for x in range(self.nCols * self.nRows)] 

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
    # clear   Clears and re-initializes the grid
    ###########################################################################
    def clear (self, distance=0, angle= 0):
        self.distance   = distance
        self.angle      = angle

        # Create the occupancy grid
        self.grid = [[ [0.0, 0.0] for x in range(self.nCols)] for y in range(self.nRows)]

        # Create the binary grid to send as telemetry
        self.binaryGrid =  [[ 0 for x in range(self.nCols)] for y in range(self.nRows)]
    # end

    ###########################################################################
    # enterRange - enters a point into the map.  The vehicle only knows about
    # it's cumulative, and it's currAngle.  The scanner knows it's scan angle
    # and range distance.
    # Parameters:
    #   carCumDist  - cumulative distance the car has travelled
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
        self.enterPoint (Xpos, Ypos)
    # end

    ###########################################################################
    # enterPoint - enters an objects position into the grid
    ###########################################################################
    def enterPoint(self, x, y):
        col = int (x / self.resolution)
        row = int (y / self.resolution)

        if (col < 0 or col >= self.nCols):
            return
        if (row < 0 or row >= self.nRows):
            return

        self.grid[row][col] = [x, y]
    # end

    ###########################################################################
    # recenterGrid - translates and rotates the map to the new distance
    # and angle.  NOTE - rather than creating a second grid and transferring
    # all the rotated/translated points to it, we do it within the same grid.
    # CAUTION though, this method only works if we are going forward, e.g.
    # moving the data in the grid generally downward.
    ###########################################################################
    def recenterGrid(self, dist, angle):
        deltaAngle = angle   #  - self.angle  dag?
        deltaDist  = dist  - self.distance
        deltaY     = deltaDist * cos(radians(deltaAngle))
        deltaX     = deltaDist * sin(radians(deltaAngle))

        for row in range(self.nRows):
            for col in range(self.nCols):
                if (not self.isZero(row, col)):
                    [x,y] = self.grid[row][col]
                    x = x - deltaX
                    y = y - deltaY

                    self.grid[row][col] = [0.0, 0.0]
                    self.enterPoint(x,y)

        # Save these last values
        self.angle    = angle
        self.distance = dist
        pass
    # end

    ###########################################################################
    # getValue
    ###########################################################################
    def getValue (self, xIndex, yIndex):
        return self.grid[xIndex][yIndex]
    # end

    ###########################################################################
    # isZero - checks if a map cell contains no data, taking into account
    # floating point roundoff
    ###########################################################################
    def isZero (self, xIndex, yIndex):
        point = self.grid[xIndex][yIndex]
        x = point[0]
        y = point[1]
        if (x > -0.001 and x < 0.001 and y > -0.001 and y < 0.001):
            return True
        return False
    # end

    ###########################################################################
    # printGrid -
    ###########################################################################
    def printGrid (self, str):
        print (str)
        for row in range (self.nRows-1, -1, -1):
            print ("Row %2d:" % row),
            for col in range (self.nCols):
                if (self.isZero(row, col)):
                    print ("%-11s" % "    ---   "),
                else:
                    [x, y] = self.getValue (row, col)
                    print ("(%4.1f,%4.1f)" % (x, y)),
                # end if
            # end for col
            print ("\n")
        # end for row
        print ""
    # end


    ###########################################################################
    # initGraphGrid -
    ###########################################################################
    def initGraphGrid (self, str, nPix, borders = False, circle = False):
        self.nPix      = nPix
        self.borders   = borders
        self.circle    = circle
        self.frame     = nPix / 2         # Width of frame around map
        xSize = self.nCols * nPix
        ySize = self.nRows * nPix

        self.win = GraphWin( str, xSize + (2*self.frame), ySize + (2*self.frame) )

        if borders:
            # Draw the vertical edges
            for col in range (self.nCols+1):
                Ptp = Point (self.frame + (col * nPix), self.frame)
                Pbt = Point (self.frame + (col * nPix), self.frame + (self.nRows * nPix))
                lin = Line (Ptp, Pbt)
                lin.setWidth(1)
                lin.setFill("gray")
                lin.draw(self.win)

            # Draw the horiz edges
            for row in range (self.nRows+1):
                Plt = Point (self.frame, self.frame + (row * nPix))
                Prt = Point (self.frame + (self.nCols * nPix), self.frame + (row * nPix))
                lin = Line (Plt, Prt)
                lin.setWidth(1)
                lin.setFill("gray")
                lin.draw(self.win)
        # end if borders

    ###########################################################################
    # clearGraphGrid -
    ###########################################################################
    def clearGraphGrid (self):
        self.win.dag_clear()


    ###########################################################################
    # graphGrid -
    #   nPix is the num of pixels to draw a single cell (nPixels per cell)
    #   borders (T/F) is whether to draw a grid surrounding all the cells
    #   circle (T/F) is whether to draw a circle or a line segment in each
    #       occupied cell. Drawing a line is faster than drawing a circle.
    ###########################################################################
    def graphGrid (self, color="red"):
        if (self.circle):
            # Draw a circle in each occupied cell (slow):
            for row in range (self.nRows):
                for col in range (self.nCols):
                    if (not self.isZero(row, col)):
                        pt = Point((col+1) * self.nPix, (self.nRows - row) * self.nPix)
                        cir = Circle(pt, self.frame/2)
                        cir.setFill(color)
                        cir.draw(self.win)
                # end for col
            # end for row
        else:
            # Draw a vertical line in each occupied cell (faster):
            for row in range (self.nRows):
                for col in range (self.nCols):
                    if (not self.isZero(row, col)):
                        Ptp = Point((col+1) * self.nPix, (self.nRows - row) * self.nPix - self.frame)
                        Pbt = Point((col+1) * self.nPix, (self.nRows - row) * self.nPix + self.frame)
                        lin = Line(Ptp, Pbt)
                        lin.setFill(color)
                        lin.setWidth(self.frame)
                        lin.draw(self.win)
                # end for col
            # end for row
        # end if circle

        #win.getMouse()
        #win.close
    # end graphGrid


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
            
        # print ('sendUDP - number of non-0 entries', total)         

    # end

    ###########################################################################
    # getGrid -
    ###########################################################################
    def getGrid(self):
        return (np.flip([[self.getValue(x, y) for y in range(self.nCols)] for x in range(self.nRows)], axis=0).tolist())
    # end
    
    ## old Histogram functions start
    
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
        angle = degrees( np.arctan2( (col - self.nCols/2), row ) )
        angIndex = int( (angle - self.minAngle) / self.angDelta )
        if angIndex < 0 or angIndex >= self.histSize:
            return -1
        else:
            return angIndex
    # end

    ###########################################################################
    # getCostArray - not used
    ###########################################################################
    def getCostArray(self, grid, maxDist, scanAngle, angDelta):
        angleBin = {}

        for col in range(grid.nCols):
            for row in range(grid.nRows):
                # if coordinate found, get cost and angle
                if not (grid.isZero(row, col)):
                    cost = maxDist * (1 - (self.getDist(self.origin, grid.grid[row][col]) / maxDist))
                    angle = degrees(np.arctan((grid.grid[row][col][0]-self.origin[0]) / (grid.grid[row][col][1]-self.origin[1])))

                    # bin angle into degree buckets
                    for slice in range(-scanAngle, scanAngle, angDelta):
                        if angle >= slice and angle < slice + angDelta:
                            # initialize bucket
                            if not angleBin.has_key(slice):
                                angleBin[slice] = cost
                            # add cost to existing bucket
                            else:
                                newValue = angleBin[slice] + cost
                                angleBin[slice] = newValue
                else:
                    cost = 0

        # for buckets that did not any cost, add empty buckets
        for slice in range(-scanAngle, scanAngle, angDelta):
            if not angleBin.has_key(slice):
                angleBin[slice] = 0

        costArray = np.array(angleBin.items())

        return costArray

    ###########################################################################
    # calcTargetAngle -
    ###########################################################################
    def calcTargetAngle(self):
        # this algorithm has two stages, the first finds the angles within +- 
        # 45 deg of our current heading that have the largest distance reading, 
        # and prefers angles that are closest to our current angle. The second 
        # searches within +- 45 deg of the first angle for the direction that 
        # we can go the farthest and not collide with stuff

        # get a local copy of the current angle
        vehState.currentAngleLock.acquire()
        currentAngle = vehState.currentAngle
        vehState.currentAngleLock.release()

        # now copy out the lidar readings, recentering on our current direction
        localLidar = []
        lidarBufferLock.acquire()
        for index in range(180)]
            bufferIndex = round(((currentAngle - math.pi+(index-1)*math.pi*2/180) % (2*math.pi))/(2*math.pi)*len(vehState.lidarBuffer)+.5)
            localLidar[index]=vehState.lidarBuffer[bufferIndex][LIDAR_DISTANCE]
        lidarBufferLock.release()        
        
        # FIND THE FIRST ANGLE

        # find the direction that would get us the farthest with the least turning, assuming that we are a particle
        maxDistance = 0
        bestDistanceIndex = 0
        for index in range(68,114) # 91 is the 0 angle, 91+- 23 is ~ +-45 deg
            if localLidar[index]>maxDistance
                maxDistance=localLidar[index]
                bestDistanceIndex=index
                continue
            if (localLidar[index]==maxDistance)AND(abs(index-91)>abs(bestDistanceIndex-91))
                bestDistanceIndex=index
                continue

        # FIND THE SECOND ANGLE
        # convert the localLidar to max distance 
        obstacleDistance = List(localLidar)
        for index in range(bestDistanceIndex-45-23,bestDistanceIndex+45+23)
            for subIndex in range(max(bestDistanceIndex-23,subIndex-45),min(bestDistanceIndex-23,subIndex+45))
                obstacleDistance[subIndex]=min(obstacleDistance[subIndex],9*2.54/localLidar[index]*sin(2*abs(index-subIndex)/180*math.pi))

        # find the direction that would get us the farthest with the least turning, assuming that we are a particle
        maxDistance = 0
        bestDistanceIndex = 0
        for index in range(bestDistanceIndex-23,bestDistanceIndex+23) # 91 is the 0 angle, 91+- 23 is ~ +-45 deg
            if obstacleDistance[index]>maxDistance
                maxDistance=obstacleDistance[index]
                bestDistanceIndex=index
                continue
            if (obstacleDistance[index]==maxDistance)AND(abs(index-bestDistanceIndex)>abs(bestDistanceIndex-bestDistanceIndex))
                bestDistanceIndex=index
                continue

        return -math.pi+(bestDistanceIndex-1)*math.pi*2/180
    # end

    ###########################################################################
    # getNearestAngle -
    ###########################################################################
    def getNearestAngle(self, nearest):
        # Zero out any old results in the histogram
        self.histArr =  [ 0 for x in range(self.histSize)]

        # Fill the histArr with the cost of each grid point
        for row in range(self.nRows):
            for col in range(self.nCols):
                if not (self.isZero(row, col)):
                    cost = self.costArr[row][col]
                    angleIndex = self.angArr[row][col]
                    self.histArr[angleIndex] += cost

        #debug
        #print("Before Low Pass Filter...")
        self.printHistArr()
        
        self.lowPassFilter(3)

        minCost = min(self.histArr)

        #print("After Low Pass Filter...")
        #print("MinCost is", minCost)        
        #self.printHistArr()

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
    # findBestAngle - find the widest path and go towards the center of it
    ###########################################################################
    def findBestAngle(self, minCost):
        inRun = False # are we in a run of minimums?
        anglePaths = [] # the run of minimums
        collectPaths = [] # the collections of minimum runs
        for index in range(self.histSize):
            if inRun and self.histArr[index] == minCost: # are we in a run and at minimum cost?
                anglePaths.append(index) # then we are still in a run, collect the index
            elif inRun and self.histArr[index] != minCost: # are we in a run and not a minimum cost?
                collectPaths.append(anglePaths) # then stash the runs to the collection
                anglePaths = [] # clear out the last collected run
                inRun = False # now we're not in a run, set to False
            elif not inRun and self.histArr[index] == minCost: # are we not in a run but at minimum cost?
                anglePaths.append(index) # then we're actually in a run so collect the index
                inRun = True # now we're in a run, set to True
            elif not inRun and self.histArr[index] != minCost: # are we not in a run and not at minimum cost?
                pass # don't do anything, we don't care!
        
        if inRun: # if the very last item in the histArr is still in a run...
            collectPaths.append(anglePaths) # collect the very last run!
            
        widestPath = 0
        pathIndex = 0
        for index, path in enumerate(collectPaths):
            if len(path) > widestPath:
                widestPath = len(path)
                pathIndex = index
        
        # debug
        #print(collectPaths)
        bestIndex = int(sum(collectPaths[pathIndex]) / widestPath)

        anglePos = self.minAngle + (self.angDelta * bestIndex)        
        
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
if __name__ == '__main__':

    g = Grid(10, nRows=100, nCols=160, distance=0, angle=0)
    g.histSize = 9
    #g.histArr =  [ 0.0, 4.0, 4.0, 5.0, 6.0, 0.0, 0.0, 5.0, 0.0]
    g.histArr  =  [ 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
    
    g.printHistArr()
    g.lowPassFilterDag(5)
    g.printHistArr()    
    #g.histArr
    

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
