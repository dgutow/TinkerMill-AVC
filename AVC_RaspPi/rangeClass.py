# rangeClass.py
#
"""
 rangeClass.py 
 Class to store the previous 'x' range measurements.  These are stored in
 a circular buffer. 
 Written by David Gutow 8/2017
"""

###############################################################################
# class Range - a class to store a range measurement (circular buffer)
###############################################################################
class Range (object):
    time        = []    # The time of this range measurement 
    angle       = []    # The angle of the scanner for this measurement
    sensor      = []    # Which sensor was used (1: 20-150cm, 2: 100-550cm)
    range       = []    # The range reported
    size        = 0     # The size of the array of ranges
    index       = 0     # the index of the latest range
    
    ###########################################################################
    def __init__(self, size):
        self.size = size   
        self.index  = 0    
        
        for i in range(self.size):
            self.time.append(0.0)
            self.angle.append(0.0)
            self.sensor.append(0.0)
            self.range.append(0.0)
        # end for
        
    # end __init__
    
    ###########################################################################
    def enterRange (self, time, angle, ranger, range):
        self.index += 1
        if (self.index >= self.size):
            self.index = 0
            
        self.time  [self.index] = time
        self.angle [self.index] = angle
        self.sensor[self.index] = ranger
        self.range [self.index] = range
    # end enterRange
    
    ###########################################################################
    # getRange - get a range tuple.  The parameter passed in 'age' is which 
    # range to return; 0 to get the most recent range, -1 to get the next 
    # most recent, -2 to get the next one after that, etc. 
    
    def getRange (self, age):
        if (age > 0 or age < -1 * self.size + 1):
            return [0, 0, 0, 0]
        # end if

        age += self.index       # remember age is always 0 or negative
        if (age < 0):           # Did we underflow
            age += self.size
        # end if
        
        return [self.time[age], self.angle[age], self.sensor[age], self.range[age]]

    # end enterRange
    
# end class 


###############################################################################
# TESTING
###############################################################################
if __name__ == "__main__":
 
    r = Range(4)
    r.enterRange(4, 4, 4, 4)
    r.enterRange(3, 3, 3, 3)
    r.enterRange(2, 2, 2, 2)    
    r.enterRange(1, 1, 1, 1)
    
    range0 = r.getRange(0)   # Should return [1,1,1,1]
    print 'Range0 =', range0
    range1 = r.getRange(-1)  # should return [2,2,2,2]
    print 'Range1 =', range1
    range2 = r.getRange(-2)  # Should return [3,3,3,3]
    print 'Range2 =', range2
    range3 = r.getRange(-3)  # Should return [4,4,4,4]
    print 'Range3 =', range3
    
    r.enterRange(0, 0, 0, 0)
    range4 = r.getRange(0)   # should return [0,0,0,0]
    print 'Range4 =', range4
    range5 = r.getRange(-1)  # Should return [1,1,1,1]
    print 'Range5 =', range5
    range6 = r.getRange(-2)  # should return [2,2,2,2]
    print 'Range6 =', range6
    range7 = r.getRange(-3)  # Should return [3,3,3,3]
    print 'Range7 =', range7
    range8 = r.getRange(1)   # Error shold return 0's
    print 'Range8 =', range8
    range9 = r.getRange(-4)  # Error shold return 0's
    print 'Range9 =', range9
    
    
