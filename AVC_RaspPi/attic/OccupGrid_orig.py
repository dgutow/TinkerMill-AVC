#!/usr/bin/env python
""" Simple occupancy-grid-based mapping. 

Author: David Gutow
Version: 9/2017
"""

from math       import *
#from graphics   import *

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
        """ 
        Construct an empty occupancy grid.              
        """
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
        # The last time the map was translated/rotated the map was set so 
        # that it's angle was perpindiclar to the car and the car's position
        # was at point [X = (gridWidth * gridResolution / 2), Y = 0.0]
        # Figure out where the car is relative to this position  
        
        # Position of the car when this measurement was made
        distTravelled = carCumDist   - self.distance
        angleDiff     = carCurrAngle - self.angle
        carXpos       = (sin(radians(angleDiff)) * distTravelled)
        carYpos       = (cos(radians(angleDiff)) * distTravelled)
        carXpos       += self.Xpos
        carYpos       += self.Ypos
        
        # Now figure out where the range point is relative to the car's 
        # current position
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
        
        # Throw the point out if out of range
        if (col < 0 or col >= self.nCols):
            return
        if (row < 0 or row >= self.nRows):
            return
        
        # Replace any current value on the assumption that this value is more
        # accurate
        self.grid[row][col] = [x, y]
        #print ("Grid pos (%6.2f, %6.2f) - Point (%6.2f, %6.2f) entered at row/col %2d/%2d" % 
        #   (self.Xpos, self.Ypos, x,y,row,col))
    # end

    ###########################################################################
    # recenterGrid - translates and rotates the map to the new distance 
    # and angle.  NOTE - rather than creating a second grid and transferring
    # all the rotated/translated points to it, we do it within the same grid.
    # CAUTION though, this method only works if we are going forward, e.g.
    # moving the data in the grid generally downward.
    ###########################################################################    
    def recenterGrid(self, dist, angle):
        # Calculate the delta X,Y that all points will be moved by
        deltaAngle = angle - self.angle
        deltaDist  = dist  - self.distance
        deltaY     = deltaDist * cos(radians(deltaAngle))       
        deltaX     = deltaDist * sin(radians(deltaAngle))
        #print ("Recenter - Input dist-%4.1f angle-%4.1f,  Delta X/Y %4.1f/%4.1f\n" % 
        #   (dist, angle, deltaX, deltaY)),
        
        for row in range(self.nRows):
            for col in range(self.nCols):
                if (not self.isZero(row, col)):
                    [x,y] = self.grid[row][col]
                    x = x - deltaX
                    y = y - deltaY
                    # we must zero out this cell first in case the new position
                    # turns out to be in the same cell
                    self.grid[row][col] = [0.0, 0.0]   
                    self.enterPoint(x,y)
                # end
            # end col
        # end row
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
    # avoidObstacles    
    ###########################################################################   
    def avoidObstacles():
        pass

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
    # end def
    
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
                lin.draw(self.win)            
                
            # Draw the horiz edges
            for row in range (self.nRows+1):
                Plt = Point (self.frame, self.frame + (row * nPix))
                Prt = Point (self.frame + (self.nCols * nPix), self.frame + (row * nPix)) 
                lin = Line (Plt, Prt)
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
    # end def
# end 
    
###############################################################################
# Class Histogram
###############################################################################
""" 
class Histogram(object)
    
    minAngle = -45      # Minimum angle of the histogram
    maxAngle = 45       # Maximum angle of the histogram
    angDelta = 5        # delta degrees between histogram cells
    hist     = []       # the histogram
    
    ###########################################################################
    # __init__  
    ###########################################################################      
    def __init__(self)
        ncells = int((maxAngle - minAngle) / angDelta)
        
        for i in range (ncells)
            hist[i] = 0
    # end init
    
    ###########################################################################
    # calcHist - an occupgrid is passed in
    ###########################################################################         
    def calcHist(self, grid)
        for row in range (grid.nRows):
            for col in range (grid.nCols):
                if (not grid.isZero(row, col)):
                    pass
            # end for col
        # end for row  
    # end init        
""" 

###############################################################################
# Test code
###############################################################################
from graphics import *

if __name__ == '__main__':
    g = Grid(resolution=10, nCols=10, nRows=9, distance=0, angle=0)
    g.enterRange (35, -10, 0,  10)  # Resulting point should be at (43.92, 34.47)
    g.enterRange (00,  0, 35, -10)  # Resulting point should be at (43.92, 34.47)    
    g.enterRange (35, -10, 20, 10)  # Resulting point should be at (43.92, 54.47)  
    g.enterRange (25,  10, 20, 10)  # Resulting point should be at (61.18, 43.41)
    g.enterRange (0, 0, 45, -90)
    g.printGrid("")
    #g.graphGrid ("AFTER POINTS ENTERED:", 50, False, False)



"""         UNIT TEST CODE
gridResolution  = 15    # 15 cm = ~6"/cell
gridWidth       = 104   # 104 cells * 6"/cell = 52 feet wide
gridHeight      = 32    # 32 cells * 6"/cell = 16 feet high

if __name__ == '__main__':
    g = Grid(resolution=10, nCols=10, nRows=9, distance=0, angle=0)
    g.enterRange (35, -10, 0,  10)  # Resulting point should be at (43.92, 34.47)
    g.enterRange (00,  0, 35, -10)  # Resulting point should be at (43.92, 34.47)    
    g.enterRange (35, -10, 20, 10)  # Resulting point should be at (43.92, 54.47)  
    g.enterRange (25,  10, 20, 10)  # Resulting point should be at (61.18, 43.41)       
    g.printGrid("\nAFTER POINTS ENTERED:") 
    g.graphGrid ("AFTER POINTS ENTERED:", 50)
    
    g.recenterGrid(10, 0)           # All points should shift down by -10
    g.printGrid("\nAFTER RE-CENTERING (10,0):")  
    g.graphGrid ("AFTER RE-CENTERING (10,0):", 50)    
    
    g.recenterGrid(20, 45)          # All points should shift by -7.1, -7.1
    g.printGrid("\nAFTER RE-CENTERING (10,45):")   
    g.graphGrid ("AFTER RE-CENTERING  (10,45):", 50) 
    
    g.clear (distance=0, angle= 0)  # Start with a fresh slate
    g.printGrid("\nAFTER CLEARING GRID:")   
    g.graphGrid ("AFTER CLEARING GRID:", 50)  
    
    g.enterRange  (10, 0, 10, 0)    # Resulting point should be at (50.0, 20.0)   
    g.recenterGrid(20, 45)          # point should shift to (50-14.1), 20.0-14.1)
    
    g.printGrid("AFTER CLEARING AND RE-CENTERING (10,45):") 
    g.graphGrid ("AFTER CLEARING AND RE-CENTERING (10,45):", 50)      
"""
###############################################################################
# Attic code
###############################################################################           
"""       
        # Find the left and right walls in the map. Search the first few rows on
        # either side for the leftmost and rightmost obstacles.
        nValsFound = 0
        sumXvals   = 0.0
        for xIndex in range(0, maxWallDist):
            for yIndex in range(0, 4):
            [x,y] = map.getValue[xIndex][yIndex]
                if (not isZero(x) and not isZero(y):
                    sumXvals += x
                    nValsFound += 1
            # end for
        # end for
        if nValsFound > 0:
            leftWallPos = sumXvals / nValsFound
        else:
            leftWallPos = 0.0
        # end
        
        # Now look for the right wall. Start at the rightmost cell and work in
        nValsFound = 0
        sumXvals   = 0.0
        for xIndex in range(gridWidth-1, gridWidth-maxWallDist, -1):
            for yIndex in range(0, 4):
            [x,y] = map.getValue[xIndex][yIndex]
                if (not isZero(x) and not isZero(y):
                    sumXvals += x
                    nValsFound += 1
            # end for
        # end for
        if nValsFound > 0:
            rightWallPos = sumXvals / nValsFound
        else:
            rightWallPos = 0.0
        # end    
        
        # Now calculate the car's position in the map reference frame
        if (not isZero (leftWallPos) and not isZero(rightWallPos) and
            not isZero (carLeftDist) and not isZero(carRightDist)):
            # we found both walls  and we have both left/right distances.  
            # Average the resulting position
            carXpos = ((leftWallPosition + carLeftDist) + 
                       (rightWallPos - carLeftDist)) / 2
"""        
