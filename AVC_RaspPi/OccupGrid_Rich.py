from math import *

import numpy as np
from matplotlib import pyplot as plt

class Grid(object):
    def __init__(self, resolution=10, nRows=50, nCols=50, distance=0, angle=0):
        self.resolution = resolution
        self.nCols      = nCols 
        self.nRows      = nRows       
        self.Xpos       = nCols * resolution / 2
        self.Ypos       = 0.0
        self.clear (distance, angle)
        
    def clear (self, distance=0, angle= 0):
        self.distance   = distance
        self.angle      = angle     
        self.grid = [[ [0.0, 0.0] for x in range(self.nCols)] for y in range(self.nRows)]    
        
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
        
    def enterPoint(self, x, y):
        col = int (x / self.resolution)
        row = int (y / self.resolution)
        
        if (col < 0 or col >= self.nCols):
            return
        if (row < 0 or row >= self.nRows):
            return
        
        self.grid[row][col] = [x, y]
        
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
    
    def getValue (self, xIndex, yIndex):
        return self.grid[xIndex][yIndex]
    
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
        
#     def graphGrid (self, str, nPix, borders = False, circle = False):  
#         frame = nPix / 2
#         xSize = self.nCols * nPix
#         ySize = self.nRows * nPix
        
#         win = GraphWin( str, xSize + (2*frame), ySize + (2*frame) )
        
#         if borders:
#             for col in range (self.nCols+1):
#                 Ptp = Point (frame + (col * nPix), frame)
#                 Pbt = Point (frame + (col * nPix), frame + (self.nRows * nPix)) 
#                 lin = Line (Ptp, Pbt)
#                 lin.draw(win)            
                
#             for row in range (self.nRows+1):
#                 Plt = Point (frame, frame + (row * nPix))
#                 Prt = Point (frame + (self.nCols * nPix), frame + (row * nPix)) 
#                 lin = Line (Plt, Prt)
#                 lin.draw(win)  
        
#         if (circle):
#             for row in range (self.nRows):
#                 for col in range (self.nCols):
#                     if (not self.isZero(row, col)):
#                         pt = Point((col+1) * nPix, (self.nRows - row) * nPix)
#                         cir = Circle(pt, frame/2)
#                         cir.setFill("red")
#                         cir.draw(win)                        
#         else:
#             for row in range (self.nRows):
#                 for col in range (self.nCols):
#                     if (not self.isZero(row, col)):
#                         Ptp = Point((col+1) * nPix, (self.nRows - row) * nPix - frame)
#                         Pbt = Point((col+1) * nPix, (self.nRows - row) * nPix + frame)
#                         lin = Line(Ptp, Pbt)
#                         lin.setFill("red")
#                         lin.setWidth(frame)
#                         lin.draw(win)                    
 
#         win.getMouse()
#         win.close
    def getGrid(self):
        return (np.flip([[self.getValue(x, y) for y in range(self.nCols)] for x in range(self.nRows)], axis=0).tolist())

class Histogram(object):    
    def __init__(self, origin, scanAngle, angDelta, minCost, maxCost):

        self.scanAngle = scanAngle
        self.angDelta = angDelta
        self.minCost = minCost
        self.maxCost = maxCost
        self.origin = origin        
            
    def getDist(self, fromcoords, tocoords):
        return sqrt((tocoords[0] - fromcoords[0]) ** 2 + (tocoords[1] - fromcoords[1]) ** 2)
    
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
    
    def getCone(self, x, y, grid): # 
        return 1 if (y >= (x - (grid.nCols / 2)) * -tan(radians(self.scanAngle))) and (y >= (x-(grid.nCols / 2)) * tan(radians(self.scanAngle))) else 0
    
    def getScanMatrix(self, grid):
        scanMatrix = [[self.getCone(x, y, grid) for x in range(grid.nCols)] for y in range(grid.nRows)]
        return np.flip(np.array(scanMatrix), axis=0)

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

    def calcHist(self, grid):
        costArray = [[self.getCost(x, y, grid) for y in range(grid.nCols)] for x in range(grid.nRows)] 
        costGrid = np.multiply(np.flip(np.array(costArray), axis=0), self.getScanMatrix(grid))

        return self.scanCostGrid(costGrid)
    
    def getAngle(self, array, nearest):
        array = np.array(array)
        minCostSum = np.amin(array[:,1]) # get the slices with the minimum cost...
        
        array = array[np.where(array[:,1] == minCostSum)[0]][:,0]
        
        # if there are multiple paths, pick the one closest to "nearest angle"
        closestAngle = array[(np.abs(array - nearest)).argmin()]
        
        angFactor = 1 if (closestAngle > 0) else -1
        
        furthestAngle = array[(np.abs(array - angFactor * self.scanAngle)).argmin()]
        
        angle = closestAngle + angFactor * (abs(furthestAngle - closestAngle) / 2)
        
        return angle
        
    pass

if __name__ == '__main__':
    g = Grid(resolution=7.5, nRows=60, nCols=60, distance=0, angle=0)
#     g = Grid(resolution=10, nRows=19, nCols=20, distance=0, angle=0)
#     g.enterRange (35, -10, 0,  10)  # Resulting point should be at (43.92, 34.47)
#     g.enterRange (00,  0, 35, -10)  # Resulting point should be at (43.92, 34.47)    
#     g.enterRange (35, -10, 20, 10)  # Resulting point should be at (43.92, 54.47)  
#     g.enterRange (25,  10, 20, 10)  # Resulting point should be at (61.18, 43.41) 
    
    # KEEP OPENING BETWEEN -40 and -30 degrees and 15 to 25 degrees
    for i in range(-60, -40, 1):
        for j in range(200, 300, 10):
            g.enterRange(0, 0, j, i)
    
    for i in range(-20, 10, 1):
        for j in range(200, 300, 10):
            g.enterRange(0, 0, j, i)
            
    for i in range(35, 60, 1):
        for j in range(100, 300, 10):
            g.enterRange(0, 0, j, i)
    
    
#     for i in range(-40, -30, 1):
#         g.enterRange(0, 0, 350, i)
        
#     for i in range(15, 30, 1):
#         g.enterRange(0, 0, 350, i)
    
#     g.graphGrid ("AFTER POINTS ENTERED:", 4, False, False 
#     g.printGrid("\nAFTER POINTS ENTERED:") 
    
    np.set_printoptions(precision=3, linewidth=2000, threshold=np.nan)
    # scanAngle or "Cone": +/- (deg)
    # angDelta or "Slice": (deg)
    # minCost - smallest cost to display representing no object detected (recommended set to 0)
    # maxCost - largest cost to display representing imminent collision (recommended set to 9 max)
    h = Histogram(origin=[0.5 * g.nCols * g.resolution, 0], scanAngle=45, angDelta=5, minCost=0, maxCost=5)
    
    # Display Cost Grid 
#     print(np.flip(np.array([[h.getCost(x, y, g) for y in range(g.nCols)] for x in range(g.nRows)]), axis = 0))
#     print("")
    # Display Cost Array
#     print(np.array(h.calcHist(g)))
#     print("")
    
    # OUTPUT
    output = h.getAngle(h.calcHist(g), 0)
    
    print(output)
    
