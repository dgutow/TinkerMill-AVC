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
        distTravelled = carCumDist   - self.distance
        angleDiff     = carCurrAngle - self.angle
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
        deltaAngle = angle - self.angle
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

        #Fill the binary grid from the occupancy grid
        binaryGrid = []
        for row in range (self.nRows):
            for col in range (self.nCols):
                if (self.isZero(row, col)):
                    binaryGrid.append (False)
                else:
                    binaryGrid.append (True)
                # End if
        # end for

        # Calculate our checksum
        checksum = PktId + currTime + nRows + nCols + carXpos + carYpos + angle
        for x in range (len(binaryGrid)):
                checksum += binaryGrid[x]
        # end for

        # Create the packet to send
        # 'L' - ulong, i - int, 'h' - short, 'B' - uchar,
        # 'x' - char, 's', 'p' - string, pascal char[]
        #packetDesc = ( '>LLLLiii%dH' % (nRows * nCols) )
        #packetDesc = ( '>LLLLiii%dx' % (nRows * nCols) ) dnw expected 7
        packetDesc = ( '>LLLLiii%dB' % (nRows * nCols) )
        packetData = struct.pack(packetDesc, PktId, currTime, nRows, nCols,
                            carXpos, carYpos, angle, *binaryGrid )

        # Send the packet!
        if (self.sock != None):
            self.sock.sendto(packetData, (self.host, self.port))

    # end

    ###########################################################################
    # getGrid -
    ###########################################################################
    def getGrid(self):
        return (np.flip([[self.getValue(x, y) for y in range(self.nCols)] for x in range(self.nRows)], axis=0).tolist())
    # end

# end class

###############################################################################
# Class Histogram
###############################################################################
class Histogram(object):
    def __init__(self, grid, origin, scanAngle, angDelta):
        self.grid       = grid
        self.nRows      = grid.nRows
        self.nCols      = grid.nCols
        self.scanAngle  = scanAngle
        self.angDelta   = angDelta
        self.minAngle   = -1 * scanAngle
        self.maxAngle   = scanAngle
        self.origin     = origin
        self.maxDist    = (sqrt( (grid.nRows/2)**2 + (grid.nCols)**2 ) )

        # Create the histogram array
        self.histSize = int((scanAngle * 2) / angDelta) + 1
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
        angle = degrees( np.arctan2( (col - self.nCols/2), row ) )
        angIndex = int( (angle - self.minAngle) / self.angDelta )
        if angIndex < 0 or angIndex >= self.histSize:
            return -1
        else:
            return angIndex
    # end

    ###########################################################################
    # getCostArray -
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
        #self.printHistArr()

        minCost = min(self.histArr)
        print("MinCost is", minCost)

        self.lowPassFilter(3)

        #debug
        self.printHistArr()

        # newDiff = 90

        return self.findBestAngle(minCost)

        # for index in range(self.histSize):
        #     anglePos = self.minAngle + self.angDelta * index
        #     if self.histArr[index] == minCost:
        #         diff = abs(anglePos - nearest)
        #         if diff < newDiff:
        #             newDiff = diff
        #             closestAngle = anglePos

        # return closestAngle
    # end

    ###########################################################################
    # lowPassFilter - to eliminate naughty zeros
    ###########################################################################
    def lowPassFilter(self, size):
        histArrayFiltered = []

        for index in range(self.histSize):
            histArrayFiltered.insert(index, self.histArr[index])
            if index >= size:
                meanCost = sum(self.histArr[index - size:index]) / size
                histArrayFiltered[int(index-floor(size/2)) - 1] = meanCost

        self.histArr = histArrayFiltered

    ###########################################################################
    # findBestAngle - find the widest path and go towards the center of it
    ###########################################################################
    def findBestAngle(self, minCost):
        anglePaths = []
        collectPaths = []
        for index in range(self.histSize):
            if self.histArr[index] == minCost:
                anglePaths.append(index)
            if self.histArr[index] != minCost and self.histArr[index-1] == minCost:
                collectPaths.append(anglePaths)
                anglePaths = []

        widestPath = 0
        pathIndex = 0
        for index, path in enumerate(collectPaths):
            if len(path) > widestPath:
                widestPath = len(path)
                pathIndex = index

        bestIndex = sum(collectPaths[pathIndex]) / widestPath

        for index in range(self.histSize):
            anglePos = self.minAngle + self.angDelta * index
            if index == bestIndex:
                return anglePos

    ###########################################################################
    # printHistArr -
    ###########################################################################
    def printHistArr(self):
        for index in range(self.histSize):
            sys.stdout.write("%5d" % (self.histArr[index]))
        print ("\n")
        for index in range(self.histSize):
            sys.stdout.write("%5d" % (index))
        print ("\n")
    # end printHistArr

###############################################################################
# Test code
###############################################################################
if __name__ == '__main__':
    import time

    np.set_printoptions(precision=3, linewidth=2000, threshold=np.nan, suppress=True)

    if (False):
        """
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
        """
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


        # scanAngle or "Cone": +/- (deg)
        # angDelta or "Slice": (deg)
        # minCost - smallest cost to display representing no object detected (recommended set to 0)
        # maxCost - largest cost to display representing imminent collision (recommended set to 9 max)
        h = Histogram(grid, origin=[0.5 * grid.nCols * grid.resolution, 0], scanAngle=45, angDelta=3)

        # OUTPUT
        output = h.getNearestAngle(0)

        print(output)
    # end if
