"""
 vehicleControl.py 
 Called by mainLoop every 100 mSec.  Controls the vehicle during the various 
 raceModes
 
 Written by David Gutow 9/2017
"""

import math
from raceModes       import raceModes
from rangeSensorPair import rangeSensorPair

############################################################################### 
# withinTol - helper function. Returns true if the two values are within the
# given tolerance.
###############################################################################
def withinTol (value1, value2, tolerance):
    if ( value1 > (value2 - tolerance) and value1 < (value2 + tolerance) ):
        return True
    return False
# end within

############################################################################### 
# fuseIrSensors - fuses vehicle position using the two IR range sensor pairs.
# The distance from the left wall (measured or estimated) is used as our 
# distance value (controlDist).
###############################################################################
def fuseIrSensors (rangeLeftPair, rangeRightPair ):
    # Get the results from both sensor pairs
    leftWallAngle    = rangeLeftPair.getPlatformAng()
    leftVehDist      = rangeLeftPair.calcDistFront  
    leftDataValid    = rangeLeftPair.dataValid
    leftControlDist  = leftVehDist +  (rsLRspacing/2)    # Left wall used as reference
    
    rightWallAngle   = rangeRightPair.getPlatformAng()
    rightVehDist     = rangeRigthPair.calcDistFront
    rightDataValid   = rangerightPair.dataValid
    rightControlDist = trackWidth - (rightVehDist + (rsLRspacing/2))  
    
    # Check if both sensor pairs report valid ranges and their angles 
    # correspond and their distances add up then use both sets of data    
    if (leftDataValid and rightDataValid):
        if ( withinTol (leftWallAngle,rightWallAngle, 10) ):
            if ( withinTol (leftVehDist + rightVehDist + rsLRspacing, trWidth, 100) ):
                # Ok I'm convinced both sensors are valid                
                vehState.leftWallAngle = leftWallAngle
                vehState.leftWallDist  = leftWallDist
                vehState.leftDataValid = True
                
                vehState.rightWallAngle= rightWallAngle
                vehState.rightWallDist = rightWallDist
                vehState.rightDataValid= True
                
                vehState.controlAngle  = (leftWallAngle + rightWallAngle) / 2
                vehState.controlDist   = (leftControlDist + rightControlDist) / 2                
                vehState.compassAngle  = vehState.iopCompassAngle
            else:
                # Interestingly, the angles are close but not the distances
                # Probably another vehicle beside us.  Figure out which distance 
                # is closest to the previous and use it
                if ( (abs(leftWallDist   - vehState.leftWallDist)) < 
                     (abs(rightWallAngle - vehState.rightWallDist)) ):  
                    # The left sensor pair are closer
                    vehState.leftWallAngle = leftWallAngle
                    vehState.leftWallDist  = leftWallDist
                    vehState.leftDataValid = True
 
                    vehState.rightDataValid= False 
                    
                    vehState.controlAngle  = leftWallAngle
                    vehState.controlDist   = leftControlDist                 
                    vehState.compassAngle  = vehState.iopCompassAngle
                else:
                    # The right sensor pair are closer
                    vehState.leftDataValid = False
 
                    vehState.rightWallAngle= rightWallAngle
                    vehState.rightWallDist = rightWallDist
                    vehState.rightDataValid= True
                    
                    vehState.controlAngle  = rightWallAngle
                    vehState.controlDist   = rightControlDist                 
                    vehState.compassAngle  = vehState.iopCompassAngle  
                # end
            # end withinTol - dist
        else:
            # The two angles aren't close to each other, use the 
            # one which is closest to the last angle
            if (abs(leftWallAngle - controlAngle)) < (abs(rightWallAngle - controlAngle)):
                # The leftWallAngle was closest
                vehState.leftWallAngle = leftWallAngle
                vehState.leftWallDist  = leftWallDist
                vehState.leftDataValid = True
 
                vehState.rightDataValid= False 
                    
                vehState.controlAngle  = leftWallAngle
                vehState.controlDist   = leftControlDist                 
                vehState.compassAngle  = vehState.iopCompassAngle                              
            else:
                # The rightWallAngle was closest
                vehState.leftDataValid = False
 
                vehState.rightWallAngle= rightWallAngle
                vehState.rightWallDist = rightWallDist
                vehState.rightDataValid= True
                    
                vehState.controlAngle  = rightWallAngle
                vehState.controlDist   = rightControlDist                 
                vehState.compassAngle  = vehState.iopCompassAngle   
            # end abs
        # end withinTol - angles
    elif (leftDataValid and 
          withinTol (leftWallAngle,   vehState.ControlAngle, 10) and
          withinTol (leftControlDist, vehState.controlDist,  50) ):
        # Only the left sensor data is valid and the left data is close to
        # the previous values. Probably another vehicle on the right.
        vehState.leftWallAngle = leftWallAngle
        vehState.leftWallDist  = leftWallDist
        vehState.leftDataValid = True
 
        vehState.rightDataValid= False 
                    
        vehState.controlAngle  = leftWallAngle
        vehState.controlDist   = leftControlDist                 
        vehState.compassAngle  = vehState.iopCompassAngle            
    elif (rightDataValid and 
          withinTol (rightWallAngle,   vehState.ControlAngle, 10) and
          withinTol (rightControlDist, vehState.controlDist,  50)   ):
        # Only the right sensor data is valid and the right data is close to
        # the previous values. Probably another vehicle on the left. 
        vehState.leftDataValid = False
 
        vehState.rightWallAngle= rightWallAngle
        vehState.rightWallDist = rightWallDist
        vehState.rightDataValid= True
                    
        vehState.controlAngle  = rightWallAngle
        vehState.controlDist   = rightControlDist                 
        vehState.compassAngle  = vehState.iopCompassAngle   
    else:
        # Neither sensor has valid data - probably going through the intersection
        # Stay on the current track using the compass
        vehState.leftDataValid  = False
        vehState.rightDataValid = False        
        vehState.controlAngle   = vehState.iopCompassAngle - vehState.compassAngle      
    # end   
    
# def fuseIrSensors

############################################################################### 
# normControl - standard vehicle control when there are no obstacles
#
###############################################################################
class normControl (object):
    Kprop           = 0.0   # Proportional constant
    Kint            = 0.0   # Integral constant
    integrator      = 0.0   # The accumulator value
    antiwindup      = 20.0  # anti-windup value
    angleErr        = 0.0   # Difference between desired angle and actual angle
    distErr         = 0.0   # Difference between desired dist and actual dist
    respDist        = 500   # Distance to complete a distance offset (cm)
    angSetpoint     = 0.0
    distSetpoint    = trackWidth/2
    
    ########################################################################### 
    # Constructor
    ###########################################################################    
    def __init__(self, prop=0.2, int=0.0, windup=20, resDist=500):
        self.clear (self, prop, int, windup, resDist)
    # end init
  
    ########################################################################### 
    # Clear - clears the system and allows the constants to be reset
    ########################################################################### 
    def clear(self, prop, int, windup, resDist):
        self.Kprop      = prop
        self.Kint       = int
        self.integrator = 0.0
        self.antiwindup = windup
        self.respDist   = resDist    
    # end

    ########################################################################### 
    # setSetPoints - sets the two setpoints in the system. 
    ###########################################################################
    def setSetPoints (self, angle=0.0, distance=trackWidth/2, respDist=500):
        self.angSetpoint    = angle
        self.distSetpoint   = distance
        self.respDist       = respDist
    #end
    
    ########################################################################### 
    # Update - calculates angle cmd for a given vehicle angle and distance
    ###########################################################################    
    def update (self, angleMeas, distMeas):
        # Calculate the steering angle needed to compensate for distance error
        self.distErr = self.distSetpoint - distMeas
        self.distAng = math.degrees(math.atan (self.distErr / self.resDist))
        
        # Calculate the total steering angle error
        self.angleErr = self.angSetpoint - angleMeas + self.distAng
        
        self.integrator += (self.angleErr * self.Kint)
        if self.integator > self.antiwindup:
            self.integrator = self.antiwindup
        if self.integrator < -1*self.antiwindup:
            self.integrator = -1*self.antiwindup
            
        output = (self.Kprop * angleErr) + self.integrator
        return output
    # end
        