# trendClass.py
#
"""
 trendClass.py 
 Class to store historical data and calculate an average of a number of 
 historical data points or to calculate the expected next point using
 linear regression.  Along with each data point, the validity of the
 data point is stored and this is taken into account when calculating 
 the various values
 Written by David Gutow 8/2017
"""
import math

class trendClass(object):

    nDatum = 0         # number of data elements to store
    index  = 0         # Index to store next datum
    data   = []        # The data array
    valid  = []        # valid flags for each data item   
        
    ###########################################################################
    def __init__(self, size):
        self.nDatum = size 
        index  = 0    
        
        for i in range(self.nDatum):
            self.data.append(0.0)
            self.valid.append(False)
        # end for
        
    # end __init__
    
    ###########################################################################
    # addDatum - enter the next value into the array.  The value could be
    # invalid.
    def addDatum (self, datum, valid):
        self.data[self.index]   = datum
        self.valid[self.index]  = valid
        self.index = self.index + 1
        if self.index >= self.nDatum:
            self.index = 0
    # end addDatum
    
    ###########################################################################
    # getNumValid - returns the number of valid points out of the last nItems
    def getNumValid (self, nItems):  
        index = self.index - nItems
        if index < 0:              # Underflow?
            index = self.nDatum + index
        nValidValues = 0
        
        for i in range(0, nItems):
            if (self.valid[index]):
                nValidValues += 1
            index += 1
            if index >= self.nDatum:
                index = 0
        # end loop          
         
        return nValidValues
         # end getNumValid
    
    ###########################################################################
    # Return the average of the last nItems, taking into account that some or
    # all may be invalid
    def getAverage (self, nItems):
        index = self.index - nItems
        if index < 0:              # Underflow?
            index = self.nDatum + index           
        average      = 0.0
        nValidValues = 0
        
        for i in range(0, nItems):
            if (self.valid[index]):
                average += self.data[index]
                nValidValues += 1
            index += 1
            if index >= self.nDatum:
                index = 0
        # end loop       
        
        # Calculate the average
        if nValidValues > 0:
            return (average / nValidValues)
        else:
            return (float('nan')) 
    # end getAverage
    
    ########################################################################### 
    # getTrend - returns the expected next value using the last nItems.  It 
    # uses a linear least squares fit, taking into account that some of the data
    # may be invalid.
    def getTrend (self, nItems):
        index = self.index - nItems
        if index < 0:              # Underflow?
            index = self.nDatum + index           
        xList   = []    # list of values along the 'x' axis
        yList   = []    # list of corresponding values along the 'y' axis
        nValidValues = 0
        
        for i in range(0, nItems):
            if (self.valid[index]):
                xList.append(i)         # Use our index for the 'x' values
                yList.append(self.data[index])
                nValidValues += 1
            index += 1
            if index >= self.nDatum:
                index = 0
        # end loop 
        
        return 0.0
    # end getTrend
    
    ########################################################################### 
    # printData - prints the entire list in order, with the oldest data printed
    # first.  For debugging
    def printData (self):       
        index = self.index     # This should be the oldest data
        
        for i in range(0, self.nDatum):
            if self.data[index]:
                validity = "valid"
            else:
                validity = "invalid"               
            print ("%d - data %6.2f - %7s index %d" %  
            (i, self.data[index], validity, index))
            index += 1
            if index >= self.nDatum:
                index = 0
        # end loop
        
    # end printData    
    
# end trendClass class    
    
###############################################################################
# TESTING
###############################################################################
if __name__ == "__main__":
 
    print ("\n################################ TEST 1")    
    test1 = trendClass(4)
    test1.addDatum (1.0 , True)
    test1.addDatum (2.0 , True)    
    test1.addDatum (3.0 , True)
    test1.addDatum (4.0 , True)
    test1.addDatum (5.0 , True)
    test1.printData()
    
    ave = test1.getAverage(1)
    if (ave != 5.0):
        print ("ERROR - test 1.0 average is %5.1f, should be 5.0" % (ave)) 
    ave = test1.getAverage(2)
    if (ave != 4.5):
        print ("ERROR - test 1.1 average is %5.1f, should be 4.5" % (ave)) 
    ave = test1.getAverage(3)
    if (ave != 4.0):
        print ("ERROR - test 1.2 average is %5.1f, should be 4.0" % (ave))    
    ave = test1.getAverage(4)
    if (ave != 3.5):
        print ("ERROR - test 1.3 average is %5.1f, should be 3.5" % (ave))   
        
    nvalid = test1.getNumValid (1)
    if (nvalid != 1):
        print ("ERROR - test 1.4 nValid is %5.1f, should be 1" % (nvalid)) 
    nvalid = test1.getNumValid (2)
    if (nvalid != 2):
        print ("ERROR - test 1.5 nValid is %5.1f, should be 2" % (nvalid))    
    nvalid = test1.getNumValid (3)
    if (nvalid != 3):
        print ("ERROR - test 1.6 nValid is %5.1f, should be 3" % (nvalid))     
    nvalid = test1.getNumValid (4)
    if (nvalid != 4):
        print ("ERROR - test 1.6 nValid is %5.1f, should be 4" % (nvalid))    


    print ("\n################################ TEST 2")        
    test1.addDatum (5.0 , True)
    test1.addDatum (3.0 , True)
    test1.addDatum (4.0 , True)
    test1.addDatum (0.0,  False)  # invalid 
    test1.addDatum (5.0 , True)    
    test1.printData()
    
    ave = test1.getAverage(2)
    if (ave != 5.0):
        print ("ERROR - test 2.1 average is %5.1f, should be 5.0" % (ave)) 
    ave = test1.getAverage(3)
    if (ave != 4.5):
        print ("ERROR - test 2.2 average is %5.1f, should be 4.5" % (ave))    
    ave = test1.getAverage(4)
    if (ave != 4.0):
        print ("ERROR - test 2.3 average is %5.1f, should be 4.0" % (ave))  
    
    nvalid = test1.getNumValid (1)
    if (nvalid != 1):
        print ("ERROR - test 2.4 nValid is %5.1f, should be 1" % (nvalid)) 
    nvalid = test1.getNumValid (2)
    if (nvalid != 1):
        print ("ERROR - test 2.5 nValid is %5.1f, should be 1" % (nvalid))    
    nvalid = test1.getNumValid (3)
    if (nvalid != 2):
        print ("ERROR - test 2.6 nValid is %5.1f, should be 2" % (nvalid))     
    nvalid = test1.getNumValid (4)
    if (nvalid != 3):
        print ("ERROR - test 2.6 nValid is %5.1f, should be 3" % (nvalid)) 

        
    print ("\n################################ TEST 3")         
    next = test1.getTrend(3)
    if (next <= 34554):
        print ("ERROR - test 3.0 nValid is %5.1f, should be 3" % (nvalid))   
    
    
# end testing   