#!/usr/bin/env python
""" Simple occupancy-grid-based mapping. 

Author: David Gutow, Rich
Version: 8/2018
"""

from math       import *
import numpy as np
from matplotlib import pyplot as plt

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
        self.clear (distance, angle)
    # end
    
    ###########################################################################
    # clear   Clears and re-initializes the grid
    ###########################################################################
    def clear (self, distance=0, angle= 0):
        self.distance   = distance
        self.angle      = angle
        self.grid = [[ [0.0, 0.0] for x in range(self.nCols)] for y in range(self.nRows)]
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
    # graphGrid - 
    #   nPix is the num of pixels to draw a single cell (nPixels per cell)
    #   borders (T/F) is whether to draw a grid surrounding all the cells
    #   circle (T/F) is whether to draw a circle or a line segment in each
    #       occupied cell. Drawing a line is faster than drawing a circle.
    ###########################################################################    
    def graphGrid (self):         
        if (self.circle):
            # Draw a circle in each occupied cell (slow):
            for row in range (self.nRows):
                for col in range (self.nCols):
                    if (not self.isZero(row, col)):
                        pt = Point((col+1) * self.nPix, (self.nRows - row) * self.nPix)
                        cir = Circle(pt, self.frame/2)
                        cir.setFill("red")
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
                        lin.setFill("red")
                        lin.setWidth(self.frame)
                        lin.draw(self.win)                    
                # end for col
            # end for row               
        # end if circle
 
        #win.getMouse()
        #win.close
    # end graphGrid

    ###########################################################################
    # initGraphGrid - 
    ###########################################################################        
    def getGrid(self):
        return (np.flip([[self.getValue(x, y) for y in range(self.nCols)] for x in range(self.nRows)], axis=0).tolist())
    # end    
    
# end class

###############################################################################
# Class Histogram
###############################################################################   
class Histogram(object):
    def __init__(self, origin, scanAngle, angDelta, minCost, maxCost):

        self.scanAngle = scanAngle
        self.angDelta = angDelta
        self.minCost = minCost
        self.maxCost = maxCost
        self.origin = origin
    # end
    
    ###########################################################################
    # getDist - 
    ###########################################################################     
    def getDist(self, fromcoords, tocoords):
        return sqrt((tocoords[0] - fromcoords[0]) ** 2 + (tocoords[1] - fromcoords[1]) ** 2)
    # end
    
    ###########################################################################
    # getCost - 
    ###########################################################################     
    def getCost(self, row, col, grid):
        # get the maximum distance based on our scanner; to be used as a scaling factor
        maxDist = sqrt((((grid.nCols / 2) * grid.resolution) ** 2) + ((grid.nRows * grid.resolution) ** 2))

        # replace [0.0, 0.0] distances since that could be interpreted as next to the robot with large number
        if (grid.grid[row][col][0] != 0 and grid.grid[row][col][1] != 0):
            dist = self.getDist(self.origin, grid.grid[row][col])
        else:
            dist = 99999.0

        cost = int(self.maxCost * (1 - (dist / maxDist)))

        # large number should produce a number lower than the minCost, so set the "cost" to zero
        if (cost < self.minCost):
            cost = 0

        return cost
    # end
    
    ###########################################################################
    # getCone - 
    ###########################################################################     
    def getCone(self, x, y, grid): #
        return 1 if (y >= (x - (grid.nCols / 2)) * -tan(radians(self.scanAngle))) and (y >= (x-(grid.nCols / 2)) * tan(radians(self.scanAngle))) else 0
    # end
    
    ###########################################################################
    # getScanMatrix - 
    ###########################################################################     
    def getScanMatrix(self, grid):
        scanMatrix = [[self.getCone(x, y, grid) for x in range(grid.nCols)] for y in range(grid.nRows)]
        return np.flip(np.array(scanMatrix), axis=0)
    # end
    
    ###########################################################################
    # scanCostGrid - 
    ###########################################################################     
    def scanCostGrid(self, grid):
        slices = []

        for angle in range(-self.scanAngle, self.scanAngle, self.angDelta):
            newangle = abs(angle - 90)

            beginAngle = newangle
            endAngle = newangle + self.angDelta

            if (beginAngle < 90 or endAngle < 90):
                smallNum = -0.001
            else:
                smallNum = 0.001

            beginAngle = beginAngle + smallNum if beginAngle % 90 == 0 else beginAngle
            endAngle = endAngle + smallNum if endAngle % 90 == 0 else endAngle

            if (newangle < 90):
                slice_ = [[1 if y >= (x - (grid.shape[1] / 2)) * (-1 if beginAngle < 0 else 1) *                            tan(radians(beginAngle)) and                                 y <= (x - (grid.shape[1] / 2)) * (-1 if endAngle < 0 else 1) *                            tan(radians(endAngle)) else 0                            for x in range(grid.shape[1])] for y in range(grid.shape[0])]
            elif (newangle >= 90):
                slice_ = [[1 if y <= (x - (grid.shape[1] / 2)) * (-1 if beginAngle < 0 else 1) *                            tan(radians(beginAngle)) and                                 y >= (x - (grid.shape[1] / 2)) * (-1 if endAngle < 0 else 1) *                            tan(radians(endAngle)) else 0                            for x in range(grid.shape[1])] for y in range(grid.shape[0])]

            slice_ = np.flip(np.array(slice_), axis = 0)
            slice_ = np.multiply(slice_, grid)
            slice_ = np.sum(slice_)

            slices.append([angle, slice_])

        return slices
    # end
    
    ###########################################################################
    # calcHist - 
    ###########################################################################     
    def calcHist(self, grid):
        costArray = [[self.getCost(x, y, grid) for y in range(grid.nCols)] for x in range(grid.nRows)]
        costGrid = np.multiply(np.flip(np.array(costArray), axis=0), self.getScanMatrix(grid))

        return self.scanCostGrid(costGrid)
    # end
    
    ###########################################################################
    # getAngle - 
    ###########################################################################     
    def getAngle(self, array, nearest):
        array = np.array(array)
        minCostSum = np.amin(array[:,1]) # get the slices with the minimum cost...
        print(minCostSum)
        array = array[np.where(array[:,1] == minCostSum)[0]][:,0]

        # if there are multiple paths, pick the one closest to "nearest angle"
        closestAngle = array[(np.abs(array - nearest)).argmin()]

        angFactor = 1 if (closestAngle > 0) else -1

        furthestAngle = array[(np.abs(array - angFactor * self.scanAngle)).argmin()]

        angle = closestAngle + angFactor * (abs(furthestAngle - closestAngle) / 2)

        return angle

    pass
    # end
    
###############################################################################
# Test code
###############################################################################
if __name__ == '__main__':

    np.set_printoptions(precision=3, linewidth=2000, threshold=np.nan, suppress=True)

    # RPLIDAR A2 scanner radius is 12 meters max
    maxWidth = 16 # meters
    desired_columns = 60
    res = (maxWidth * 100) / desired_columns # cm/Grid

    g = Grid(resolution=res, nRows=desired_columns, nCols=desired_columns, distance=0, angle=0)

    # # TEST SCRIPT - KEEP OPENING BETWEEN -40 and -30 degrees and 15 to 25 degrees
    # for i in range(-60, -40, 1):
    #     for j in range(200, 300, 10):
    #         g.enterRange(0, 0, j, i)
    #
    # for i in range(-20, 10, 1):
    #     for j in range(200, 300, 10):
    #         g.enterRange(0, 0, j, i)
    #
    # for i in range(35, 60, 1):
    #     for j in range(100, 300, 10):
    #         g.enterRange(0, 0, j, i)
    #
    # for i in range(-40, -30, 1):
    #     g.enterRange(0, 0, 350, i)
    #
    # for i in range(15, 30, 1):
    #     g.enterRange(0, 0, 350, i)

    s = LIDAR(portname='/dev/ttyUSB0')

    for j in range(10):
        obstacles = s.scan() # returns an array of one rotation of obstacles

        for i in obstacles:
            g.enterRange(0, 0, i[1] / 10, i[0])


    # scanAngle or "Cone": +/- (deg)
    # angDelta or "Slice": (deg)
    # minCost - smallest cost to display representing no object detected (recommended set to 0)
    # maxCost - largest cost to display representing imminent collision (recommended set to 9 max)
    h = Histogram(origin=[0.5 * g.nCols * g.resolution, 0], scanAngle=45, angDelta=3, minCost=0, maxCost=9)

    # # Display Cost Grid
    # print(np.flip(np.array([[h.getCost(x, y, g) for y in range(g.nCols)] for x in range(g.nRows)]), axis = 0))
    # print("")
    # # Display Cost Array
    # print(np.array(h.calcHist(g)))
    # print("")

    # OUTPUT
    output = h.getAngle(h.calcHist(g), 0)

    print(output)
