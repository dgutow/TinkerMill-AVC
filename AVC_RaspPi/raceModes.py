# raceModes.py
#
"""
 raceState.py 
 Class to store the current mode of the system during the race.
 Written by David Gutow 8/2017
"""

############################################################################### 
# class RaceModes - an enumeration of the possible states of the race
###############################################################################
class raceModes(object):

    ###########################################################################
    # The enumerations of each mode:
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
        if mode == self.NONE:
            return "NONE"
        elif mode == self.WAIT_FOR_BIST:
            return "WAIT_FOR_BIST"       
        elif mode == self.WAIT_FOR_START:
            return "WAIT_FOR_START"    
            
        elif mode == self.RACE_BEGIN:
            return "RACE_BEGIN"
            
        elif mode == self.RACE_STRAIGHT:
            return "RACE_STRAIGHT"               
        elif mode == self.RACE_CURVE:
            return "RACE_CURVE"  
        elif mode == self.NEGOT_CROSSING:
            return "NEGOT_CROSSING"     
            
        elif mode == self.APPR_STOPSIGN:
            return "APPR_STOPSIGN"     
        elif mode == self.NEGOT_STOPSIGN:
            return "NEGOT_STOPSIGN"   
        elif mode == self.RECOV_STOPSIGN:
            return "RECOV_STOPSIGN"              
            
        elif mode == self.APPR_HOOP:
            return "APPR_HOOP"      
        elif mode == self.NEGOT_HOOP:
            return "NEGOT_HOOP"          
        elif mode == self.RECOV_HOOP:
            return "RECOV_HOOP"  
            
        elif mode == self.APPR_BARRELS:
            return "APPR_BARRELS"   
        elif mode == self.NEGOT_BARRELS:
            return "NEGOT_BARRELS"    
        elif mode == self.RECOV_BARRELS:
            return "RECOV_BARRELS"  
            
        elif mode == self.APPR_RAMP:
            return "APPR_RAMP"    
        elif mode == self.NEGOT_RAMP:
            return "NEGOT_RAMP"     
        elif mode == self.RECOV_RAMP:
            return "RECOV_RAMP"  
            
        elif mode == self.APPR_PED:
            return "APPR_PED"     
        elif mode == self.APPR_XWALK:
            return "APPR_XWALK"         
        elif mode == self.STOP_XWALK:
            return "STOP_XWALK" 
        elif mode == self.XWALK_STARTUP:
            return "XWALK_STARTUP"         
            
        elif mode == self.WAIT_FOR_END:
            return "WAIT_FOR_END"     
        elif mode == self.NEGOT_END:
            return "NEGOT_END"                  
        elif mode == self.TERMINATE:
            return "TERMINATE"   
        elif mode == self.ERROR:
            return "ERROR" 
        elif mode == self.ESTOP:
            return "ESTOP"
        elif mode == self.UNKNOWN:
            return "UNKNOWN"         
        else:           
            return "truly unknown???"    
    # end toString
    
    ###########################################################################
    def printMode(self, str):
        print ("%s currMode = %2d - %s" % (
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