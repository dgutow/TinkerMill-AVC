"""
 raceState.py 
 Class to store the current mode of the system during the race.
 Written by David Gutow 8/2017
"""

from constants      import *        # Vehicle and course constants

############################################################################### 
# class RaceModes - an enumeration of the possible states of the race
###############################################################################
class raceModes(object):

    ###########################################################################
    # The enumerations of each state:
    NONE            = 0     # Not defined
    WAIT_FOR_BIST   = 1     # robot being initialized
    WAIT_FOR_START  = 2     # waiting for the start signal
    
    RACE_BEGIN      = 4     # Just starting, getting going
    RACE_STRAIGHT   = 5     # racing in normal Modes
    RACE_CURVE      = 6     # racing around a curve
    NEGOT_CROSSING  = 7     # negotiating the intersection
    
    APPR_STOPSIGN   = 10    # spotted a stop sign
    NEGOT_STOPSIGN  = 11    # negotiating stop sign
    RECOV_STOPSIGN  = 12    # recovery from stop sign
    
    APPR_HOOP       = 20    # spotted the hoop
    NEGOT_HOOP      = 21    # negotiating the hoop
    RECOV_HOOP      = 22    # recovery from hoop
    
    APPR_BARRELS    = 30    # spotted the barrels
    NEGOT_BARRELS   = 31    # negotiating the barrels
    RECOV_BARRELS   = 32    # recovery from barrels
    
    APPR_RAMP       = 40    # spotted the ramp
    NEGOT_RAMP      = 41    # negotiating the ramp
    RECOV_RAMP      = 42    # recovery from ramp
    
    APPR_PED        = 50    # spotted the pedestrian
    APPR_XWALK      = 51
    STOP_XWALK      = 52    # 
    XWALK_STARTUP   = 53    #     
    
    WAIT_FOR_END    = 60    # waiting for signal of end of course
    NEGOT_END       = 61    # ending race
    
    TERMINATE       = 100   # 
    ERROR           = 101   # An error occured
    ESTOP           = 102   # An estop occured
    
    UNKNOWN         = 110   # huh?
    
    # Strings for use in text messages
    stringDict = {NONE           : "NONE"          ,
                  WAIT_FOR_BIST  : "WAIT_FOR_BIST" ,
                  WAIT_FOR_START : "WAIT_FOR_START",
                  RACE_BEGIN     : "RACE_BEGIN"    ,
                  RACE_STRAIGHT  : "RACE_STRAIGHT" ,
                  RACE_CURVE     : "RACE_CURVE"    ,
                  NEGOT_CROSSING : "NEGOT_CROSSING",
                  APPR_STOPSIGN  : "APPR_STOPSIGN" ,
                  NEGOT_STOPSIGN : "NEGOT_STOPSIGN",
                  RECOV_STOPSIGN : "RECOV_STOPSIGN",
                  APPR_HOOP      : "APPR_HOOP"     ,
                  NEGOT_HOOP     : "NEGOT_HOOP"    ,
                  RECOV_HOOP     : "RECOV_HOOP"    ,
                  APPR_BARRELS   : "APPR_BARRELS"  ,
                  NEGOT_BARRELS  : "NEGOT_BARRELS" ,
                  RECOV_BARRELS  : "RECOV_BARRELS" ,
                  APPR_RAMP      : "APPR_RAMP"     ,
                  NEGOT_RAMP     : "NEGOT_RAMP"    ,
                  RECOV_RAMP     : "RECOV_RAMP"    ,
                  APPR_PED       : "APPR_PED"      ,
                  APPR_XWALK     : "APPR_XWALK"    ,
                  STOP_XWALK     : "STOP_XWALK"    ,
                  XWALK_STARTUP  : "XWALK_STARTUP" ,
                  WAIT_FOR_END   : "WAIT_FOR_END"  ,
                  NEGOT_END      : "NEGOT_END"     ,
                  TERMINATE      : "TERMINATE"     ,
                  ERROR          : "ERROR"         ,
                  ESTOP          : "ESTOP"         ,
                  UNKNOWN        : "UNKNOWN"       }    
    # Nominal speeds in each state
    speedDict = {NONE           : speedZero,
                 WAIT_FOR_BIST  : speedZero,
                 WAIT_FOR_START : speedZero,
                 RACE_BEGIN     : speedApproach,
                 RACE_STRAIGHT  : speedMax,
                 RACE_CURVE     : speedMax,
                 NEGOT_CROSSING : speedMax,
                 APPR_STOPSIGN  : speedApproach,
                 NEGOT_STOPSIGN : speedPed,
                 RECOV_STOPSIGN : speedRecov,
                 APPR_HOOP      : speedApproach,
                 NEGOT_HOOP     : speedHoop,
                 RECOV_HOOP     : speedRecov,
                 APPR_BARRELS   : speedApproach,
                 NEGOT_BARRELS  : speedBarrels,
                 RECOV_BARRELS  : speedRecov,
                 APPR_RAMP      : speedApproach,
                 NEGOT_RAMP     : speedRamp,
                 RECOV_RAMP     : speedRecov,
                 APPR_PED       : speedApproach,
                 APPR_XWALK     : speedApproach,
                 STOP_XWALK     : speedZero,
                 XWALK_STARTUP  : speedApproach,
                 WAIT_FOR_END   : speedMax,
                 NEGOT_END      : speedMax,
                 TERMINATE      : speedZero,
                 ERROR          : speedZero,
                 ESTOP          : speedZero,
                 UNKNOWN        : speedZero }                      
 
    ###########################################################################    
    # The values stored in this class
    
    currMode        = NONE
    prevMode        = NONE  # The previous mode we were in
    modeCount       = 0     # Count of how long we've been in this mode
 
    ###########################################################################
    # setMode - Sets a new mode
    #   
    def setMode(self, mode):
        self.prevMode   = self.currMode
        self.currMode   = mode
        self.modeCount  = 0
    # end
        
    ###########################################################################
    # setMode - Sets a new mode. Doesn't work....
    #   
    def getMode(self):
        return (self.currMode)
    # end
    
    ###########################################################################
    # getSpeed - Gets the speed for this state
    #   
    def getSpeed(self):
        return (self.speedDict[self.currMode])
    # end
    
    ###########################################################################
    # setSpeed - Gets the speed for this state
    #   
    def setSpeed(self, speed):
        self.speedDict[self.currMode] = speed
    # end    
    
    ###########################################################################
    # newMode - Returns true if we've just switched to a mode.  Also increments
    # the modeCount value each time it's called.
    #   
    def newMode(self):
        self.modeCount += 1      # First time around it'll be equal to '1'
        if self.modeCount <= 1:
            return True          # We are in a new mode
        # end
        return False
    # end
    
    ###########################################################################
    # toString - converts a mode to a string
    #     
    def toString(self, mode):
        return (self.stringDict[mode])
    # end toString
    
    ###########################################################################
    def printMode(self, str):
        print ("%s %2d: %s" % (
                str, self.currMode , self.toString(self.currMode)))
    # end printMode
    
# end

###############################################################################
# TESTING
###############################################################################
if __name__ == "__main__":   
    mode = raceModes()
    mode.printMode ("starting up:  ")
    
    mode.setMode (raceModes.NONE           )     
    mode.printMode ("NONE          ")
    print("speed", mode.getSpeed())
    mode.setMode (raceModes.WAIT_FOR_BIST  )     
    mode.printMode ("WAIT_FOR_BIST ")
    mode.setMode (raceModes.WAIT_FOR_START )     
    mode.printMode ("WAIT_FOR_START")

    mode.setMode (raceModes.RACE_BEGIN     )     
    mode.printMode ("RACE_BEGIN    ")
    mode.setMode (raceModes.RACE_STRAIGHT  )     
    mode.printMode ("RACE_STRAIGHT ")
    mode.setMode (raceModes.RACE_CURVE     )     
    mode.printMode ("RACE_CURVE    ")
    mode.setMode (raceModes.NEGOT_CROSSING )     
    mode.printMode ("NEGOT_CROSSING")

    mode.setMode (raceModes.APPR_STOPSIGN  )     
    mode.printMode ("APPR_STOPSIGN ")
    mode.setMode (raceModes.NEGOT_STOPSIGN )     
    mode.printMode ("NEGOT_STOPSIGN")
    mode.setMode (raceModes.RECOV_STOPSIGN )     
    mode.printMode ("RECOV_STOPSIGN")

    mode.setMode (raceModes.APPR_HOOP      )     
    mode.printMode ("APPR_HOOP     ")
    mode.setMode (raceModes.NEGOT_HOOP     )     
    mode.printMode ("NEGOT_HOOP    ")
    mode.setMode (raceModes.RECOV_HOOP     )     
    mode.printMode ("RECOV_HOOP    ")

    mode.setMode (raceModes.APPR_BARRELS   )     
    mode.printMode ("APPR_BARRELS  ")
    mode.setMode (raceModes.NEGOT_BARRELS  )     
    mode.printMode ("NEGOT_BARRELS ")
    mode.setMode (raceModes.RECOV_BARRELS  )     
    mode.printMode ("RECOV_BARRELS ")

    mode.setMode    (raceModes.APPR_RAMP      )     
    mode.printMode  ("APPR_RAMP     ")
    mode.setMode    (raceModes.NEGOT_RAMP     )     
    mode.printMode  ("NEGOT_RAMP    ")
    mode.setMode    (raceModes.RECOV_RAMP     )     
    mode.printMode  ("RECOV_RAMP    ")

    mode.setMode (raceModes.APPR_PED       )     
    mode.printMode ("APPR_PED      ")
    mode.setMode (raceModes.APPR_XWALK     )     
    mode.printMode ("APPR_XWALK    ")
    mode.setMode (raceModes.STOP_XWALK     )     
    mode.printMode ("STOP_XWALK    ")
    mode.setMode (raceModes.XWALK_STARTUP  )     
    mode.printMode ("XWALK_STARTUP ")

    mode.setMode (raceModes.WAIT_FOR_END   )     
    mode.printMode ("WAIT_FOR_END  ")
    mode.setMode (raceModes.NEGOT_END      )     
    mode.printMode ("NEGOT_END     ")

    mode.setMode (raceModes.TERMINATE      )     
    mode.printMode ("TERMINATE     ")
    mode.setMode (raceModes.ERROR          )     
    mode.printMode ("ERROR         ")
    mode.setMode (raceModes.ESTOP          )     
    mode.printMode ("ESTOP         ")

    mode.setMode (raceModes.UNKNOWN        )
    mode.printMode ("UNKNOWN       ")
        
# end    
