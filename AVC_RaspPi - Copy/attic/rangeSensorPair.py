#
"""
 rangeSensorPair.py 
 Class to reprent the data from a pair of rangefinders.  The ranges from the
 two rangefinders are entered and then combined to get the platform angle
 and linear distance to a far wall
 The class has to be initialized with initRightSide either True (this range pair 
 is on the right side) or False (Left side).  Since the angle definitions are
 mirror images of each other, the resulting platform angles are mirror images
 of each other.  If the pair is on the left side the platform angle is
 inverted.
 
 Written by David Gutow 8/2017
"""

import math
import sys
import pdb

class rangeSensorPair(object):
    # Initialized attributes
    initFrontAng   = 0.0   # angle of front sensor from perpindicular
    initRearAng    = 0.0   # angle of rear sensor from perpindicular
    initSensorDist = 0.0   # vetical distance between the two sensors
    initTotalAng   = 0.0   # total angle between the two sensors
    initMinDist    = 0.0   # minimum distance the sensor will work
    initMaxDist    = 0.0   # maximum distance the sensor will work
    initRightSide  = True  # True if this instantiation is for the right side
                           # of the platform, False for left side.        
    # Measured attributes
    measFrontRange = 0.0   # measured distance of front sensor
    measRearRange  = 0.0   # measure distance of rear sensor
        
    # Calculated attributes
    calcDistFront  = 0.0   # distance to wall using front sensor
    calcDistRear   = 0.0   # distance to wall using rear sensor
    calcAngFront   = 0.0   # angle between front leg and wall
    calcAngRear    = 0.0   # angle between rear leg and wall  
    calcWallDist   = 0.0   # distance of two range points on far wall
    frontValid     = False # data from front sensor is valid
    rearValid      = False # data from rear sensor is valid
    dataValid      = False # overall results are valid    
    
    # Intermediate calculations - saved for debugging/verification
    constRatio     = 0.0   # ratio used in law of sines
    frontWallAngle = 0.0   # far angle between front leg and wall
    rearWallAngle  = 0.0   # Far angle between rear leg and wall

    ###########################################################################
    def __init__(self, initFrontAng, initRearAng, initSensorDist, 
                            initMinDist, initMaxDist, rightSide): 
        self.initFrontAng   = math.radians(initFrontAng)
        self.initRearAng    = math.radians(initRearAng)
        self.initTotalAng   = self.initFrontAng + self.initRearAng
        if (self.initTotalAng < 0.001):
            # DAG signal error
            print ("rangeSensorError.init ERROR - total angle equals 0!")
            sys.exit()
        # end
        
        self.initSensorDist = initSensorDist
        self.initMinDist    = initMinDist
        self.initMaxDist    = initMaxDist
        self.initRightSide  = rightSide
    # end __init__
    
    ###########################################################################
    def newMeasurement (self, measFrontRange, measRearRange):
        # pdb.set_trace()
        self.measFrontRange  = measFrontRange
        self.measRearRange   = measRearRange   
        
        # check if sensor ranges are valid
        self.frontValid = True
        self.rearValid  = True
        self.dataValid  = True
        if (measFrontRange > self.initMaxDist or measFrontRange < self.initMinDist): 
            self.frontValid = False
            self.dataValid  = False
        if (measRearRange  > self.initMaxDist or measRearRange  < self.initMinDist):     
            self.rearValid  = False    
            self.dataValid  = False
        if ( not self.dataValid):
            print ("rangeSensorError.newmeas ERROR - data not valid!")
            return (False)
                   
        # law of cosines
        self.calcWallDist = math.sqrt (self.measFrontRange**2 + self.measRearRange**2 - 
            2 * self.measFrontRange * self.measRearRange * math.cos(self.initTotalAng))
        
        # law of sines - constRatio = distance/sin(ang)
        self.constRatio       = self.calcWallDist / math.sin(self.initTotalAng) 
        self.frontWallAngle   = math.asin (self.measRearRange  / self.constRatio)
        self.rearWallAngle    = math.asin (self.measFrontRange / self.constRatio)
        
        # figure out the distance/angle of the platform
        self.calcAngFront   = math.pi - self.frontWallAngle - self.initFrontAng
        self.calcAngRear    = math.pi - self.rearWallAngle  - self.initRearAng 
        
        self.calcDistFront  = math.sin (self.frontWallAngle) * self.measFrontRange
        self.calcDistRear   = math.sin (self.rearWallAngle ) * self.measRearRange
        return (True)      
    # end newMeasurement
    
    ###########################################################################
    # The angles are defined as mirror images from left to right side to 
    # on the left side the platform is the negative of the right side
    # Positive angles are tilted to the right, negative angles tilted to left
    
    def getPlatformAng(self):
        if (self.initRightSide):
            return (math.degrees (math.pi / 2 - self.calcAngFront))
        else:
            return ( -1 * math.degrees (math.pi / 2 - self.calcAngFront))        
    # end
    
    ###########################################################################       
    def printAttr (self, str="testing"):
        print (str)
        print ("INITIALIZED: Angles F/R %5.2f/%5.2f, Total Ang %5.2f, Sensor Dist %5.1f, Min/Max %5.1f/%5.1f" %
            (math.degrees(self.initFrontAng), math.degrees(self.initRearAng), 
            math.degrees(self.initTotalAng),  self.initSensorDist, 
            self.initMinDist, self.initMaxDist))
        print ("   MEASURED: Front Range, Rear Range %5.1f, %5.1f" % 
            (self.measFrontRange, self.measRearRange))
        print ("INTERMEDATS: Wall Angles F/R %5.2f/%5.2f, Wall Distance %5.1f" %
            (math.degrees(self.frontWallAngle), math.degrees(self.rearWallAngle), 
            self.calcWallDist))       
        print (" CALCULATED: Platform Angles F/R %5.2f/%5.2f, Platform Dist F/R %6.2f/%6.2f" %
            (math.degrees(self.calcAngFront), math.degrees(self.calcAngRear), 
            self.calcDistFront, self.calcDistRear))
     # end printAttributes          
    
# end rangeSensorPair class     
    
###############################################################################
# TESTING
###############################################################################
if __name__ == "__main__":
    ##### TEST # 1 
    print ("######### TEST 1")
    p = rangeSensorPair(50, 10, 0, 100, 500, True)
    p.newMeasurement (100, 122.67)
    p.printAttr ("50/10 angles, 100/122.67 ranges - test #1:")

    # Far angles - front angle should be 70 degrees
    angle = math.degrees(p.frontWallAngle)
    if (angle < 69.9 or angle > 70.1): 
        print ("rangeSensorError ERROR - frontWallAngle is %5.1f, should be 70" % (angle))
    # rear platform angle should be 50 degrees
    angle = math.degrees(p.rearWallAngle)
    if (angle < 49.9 or angle > 50.1):
        print ("rangeSensorError ERROR - rearWallAngle is %5.1f, should be 50" % (angle))    
  
    # platform angles - platform angle should be 60 degrees
    angle = math.degrees(p.calcAngFront)
    if (angle < 59.9 or angle > 60.1): 
        print ("rangeSensorError ERROR - calcAngFront is %5.1f, should be 70" % (angle))
    # rear platform angle should be 120 degrees
    angle = math.degrees(p.calcAngRear)
    if (angle < 119.9 or angle > 120.1):
        print ("rangeSensorError ERROR - calcAngRear is %5.1f, should be 50" % (angle))        
    # platform angle should be 30 degrees
    angle = p.getPlatformAng()
    if (angle < 29.9 or angle > 30.1): 
        print ("rangeSensorError ERROR - platformAngle is %5.1f, should be 30" % (angle))  
    # platform distance should be 93.97 
    dist = p.calcDistFront
    if (dist < 93.87 or dist > 94.07): 
        print ("rangeSensorError ERROR - platformDistFront is %5.1f, should be 93.97" % (dist))
    dist = p.calcDistRear
    if (dist < 93.87 or dist > 94.07): 
        print ("rangeSensorError ERROR - calcDistRear is %5.1f, should be 93.97" % (dist))        
    
    ###### TEST # 2 
    print ("######### TEST 2")
    left = rangeSensorPair(45, 45, 0, 100, 500, False)
    left.newMeasurement (348.7, 244.15)
    left.printAttr ("45/45 angles, 348 / 244 ranges - test #2:")

    # Far wall angles - angle should be 35 degrees
    angle = math.degrees(left.frontWallAngle)
    if (angle < 34.9 or angle > 35.1): 
        print ("rangeSensorError ERROR - frontWallAngle is %5.1f, should be 35" % (angle))
    # rear platform angle should be 45 degrees
    angle = math.degrees(left.rearWallAngle)
    if (angle < 54.9 or angle > 55.1):
        print ("rangeSensorError ERROR - rearWallAngle is %5.1f, should be 55" % (angle))    
  
    # platform angles - platform angle should be 100 degrees
    angle = math.degrees(left.calcAngFront)
    if (angle < 99.9 or angle > 100.1): 
        print ("rangeSensorError ERROR - calcAngFront is %5.1f, should be 100" % (angle))
    # rear platform angle should be 80 degrees
    angle = math.degrees(left.calcAngRear)
    if (angle < 79.9 or angle > 80.1):
        print ("rangeSensorError ERROR - calcAngRear is %5.1f, should be 80" % (angle)) 
        
    # platform angle should be 10 degrees
    angle = left.getPlatformAng()
    if (angle < 09.9 or angle > 10.1): 
        print ("rangeSensorError ERROR - platformAngle is %5.1f, should be 10" % (angle))  
    # platform distance should be 200 
    dist = left.calcDistFront
    if (dist < 199.9 or dist > 200.1): 
        print ("rangeSensorError ERROR - platformDistFront is %5.1f, should be 200" % (dist))
    dist = left.calcDistRear
    if (dist < 199.0 or dist > 200.1): 
        print ("rangeSensorError ERROR - calcDistRear is %5.1f, should be 200" % (dist))        
                
# end 
