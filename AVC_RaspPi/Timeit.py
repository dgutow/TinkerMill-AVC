#!/usr/bin/env python
""" 
Time the various modules

Author: David Gutow
Version: 12/2018
"""

import sys
import time
import numpy as np
import os as os

NTIMERS = 20                    # The number of timers we can handle

###############################################################################
# Class Timeit
###############################################################################
class Timeit(object):

    ###########################################################################
    # __init__   start_iter is the iteration number you wish to start timing
    # Use 0 to start timing from the first iteration, 1 to start timing from
    # the second iteration, etc.
    ###########################################################################
    def __init__(self, start_iter=0):
        self.start_iter = start_iter
        self.nIters     = [ 0 for x in range(NTIMERS)]      # number of iters
        self.startTime  = [ 0 for x in range(NTIMERS)]      # last start time
        self.endTime    = [ 0 for x in range(NTIMERS)]      # last end time
        self.cumTime    = [ 0 for x in range(NTIMERS)]      # cumulative time
        self.timing     = [ False for x in range(NTIMERS)]  # currently timing 
        
    ###########################################################################
    # start  - starts a timer   
    ###########################################################################        
    def start (self, timer):
        if (timer >= NTIMERS):
            print ("Timeit 'start' called with too many timers")
            return
            
        self.nIters[timer] += 1
        if (self.nIters[timer] > self.start_iter):
            self.startTime[timer] = time.perf_counter()
            self.timing[timer] = True

    ###########################################################################
    # stop  - stop a running timer   
    ###########################################################################            
    def stop (self, timer):
        if (timer >= NTIMERS):
            print ("Timeit 'stop' called with too many timers")
            return
            
        if (self.nIters[timer] > self.start_iter):    
            self.endTime[timer]  = time.perf_counter()
            self.cumTime[timer] += (self.endTime[timer] - self.startTime[timer])
            self.timing[timer]   = False
            
    ###########################################################################
    # printStats  - print out all the statistics we've collected 
    ###########################################################################         
    def printStats (self, timer, str=""):
        if (self.nIters[timer] > 0):
            aveTime = self.cumTime[timer] / (self.nIters[timer] - self.start_iter)
            lastTime= self.endTime[timer] - self.startTime[timer]
            if (not self.timing[timer]):
                print ("%s timer %2d: average time: %f, last time: %f" % 
                        (str, timer, aveTime, lastTime) )
            else:
                print ("%s timer %d: ERROR still timing" % (str, timer) )   
    # end printStats
  
    ###########################################################################
    # printAll  - print out all the statistics all timers 
    ###########################################################################
    def printAll (self, str = ""):
        for x in range (NTIMERS):
            self.printStats (x, str)            
    # end printAll
    
# end class Timeit    

###############################################################################
# Test code
###############################################################################
if __name__ == '__main__':
    timer0 = Timeit(0)
    timer1 = Timeit(1)  # Won't start until the second iteration
    
    # Test in a loop.  The first time through will be for 0.5 seconds, 
    # the second time will be 1.5 seconds
    
    for i in range (2):
        timer0.start(0)
        timer0.start(10)
        timer1.start(0)
        timer1.start(19)
        
        if (i ==0):
            period = 0.5
        else:
            period = 1.5
        startTime = time.perf_counter()              
        while (time.perf_counter() < (period + startTime)):
                pass
        
        timer0.stop(0)
        timer0.stop(10)
        timer1.stop(0)
        timer1.stop(19)   
    
    # All done, print all the stats
    timer0.printAll ("timer0")
    timer1.printAll ("timer1")
    