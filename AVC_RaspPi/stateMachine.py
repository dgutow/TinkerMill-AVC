"""
 stateMachine.py 
 Called by mainLoop every 100 mSec.  Determines what need to happen in each
 state
 
 Written by David Gutow 9/2017
"""

############################################################################### 
# 
###############################################################################

from vehicleState   import *
from raceModes      import raceModes   
from constants      import *        # Vehicle and course constants

if WINDOWS:
    from serialClassSim  import serialClass
else:    
    from serialClass     import serialClass
# end if WINDOWS 

simMaxCnt   = 20            # max count for use during simulations
InitMaxCnt  = 30            # Max time without an IOP msg before we signal a problem
BistMaxCnt  = 30            # Max time IOP is in BIST before we signal a problem

############################################################################### 
# stateControl - choose what to do depending on our current state
###############################################################################

def stateMachine (vehState, serialPort):
    #################################################
    raceModes.printMode("STATEMACHINE:")
    if vehState.mode.currMode == raceModes.NONE:
        if vehState.mode.newMode():
            playSound (raceModes.NONE)   
            
        if (vehState.iopMode == IOP_MODE_NONE):    
            # We haven't received anything from IOP yet
            if (vehState.mode.modeCount >= InitMaxCnt):
                vehState.errorString = "IOP NOT COMMUNICATING"
                vehState.mode.setMode (raceModes.ERROR) 
            # end if           
        else:
            vehState.mode.setMode (raceModes.WAIT_FOR_BIST)  
        # end if
        
    #################################################         
    elif vehState.mode.currMode == raceModes.WAIT_FOR_BIST:
        if vehState.mode.newMode():   
            playSound (raceModes.WAIT_FOR_BIST)     
         
        # Check if the IOP BIST is complete
        if (vehState.iopMode <> IOP_MODE_BIST):
            if (vehState.mode.modeCount > BistMaxCnt):
                vehState.errorString = "IOP BIST FAILURE"
                vehState.mode.setMode (raceModes.ERROR)  
        else:
            vehState.mode.setMode(raceModes.WAIT_FOR_START)       
        # end if             
        
    #################################################        
    elif vehState.mode.currMode == raceModes.WAIT_FOR_START:
        if vehState.mode.newMode():   
            playSound (raceModes.WAIT_FOR_START)   
            serialPort.sendCommand ('D', 0, 0, 0)

        # end

        if (vehState.iopStartSwitch):     # We're Off!
            vehState.mode.setMode(raceModes.RACE_BEGIN) 
        # end

    #################################################        
    elif vehState.mode.currMode == raceModes.RACE_BEGIN:
        if vehState.mode.newMode():   
            playSound (raceModes.RACE_BEGIN)   
            serialPort.sendCommand ('D', 0, 0, 0)

        # end

        if (vehState.iopStartSwitch):     # We're Off!
            vehState.mode.setMode(raceModes.RACE_STRAIGHT) 
        # end
        
    ########################################################################### 
    # NORMAL RACE MODES
    ###########################################################################                
    elif vehState.mode.currMode == raceModes.RACE_STRAIGHT:
        if vehState.mode.newMode():   
            playSound (raceModes.RACE_STRAIGHT)   
            visionSend(obstacleSequence[vehState.obstacleIndex])  
            vehState.obstacleIndex += 1
        # end
        
        # If we got an obstacle sighting from the vision system transition
        newState = obstacleTransition ()
        if (newState != raceModes.NONE):    
            vehState.mode.setMode(newState)   
        
    #################################################         
    elif vehState.mode.currMode == raceModes.RACE_CURVE:
        if vehState.mode.newMode():   
            playSound (raceModes.RACE_CURVE)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_CROSSING) 
        
    #################################################         
    elif vehState.mode.currMode == raceModes.NEGOT_CROSSING:
        if vehState.mode.newMode():   
            playSound (raceModes.NEGOT_CROSSING)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.APPR_STOPSIGN)  
        
    ########################################################################### 
    # STOPSIGN MODES
    ###########################################################################                           
    elif vehState.mode.currMode == raceModes.APPR_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (raceModes.APPR_STOPSIGN)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_STOPSIGN) 
        
    #################################################         
    elif vehState.mode.currMode == raceModes.NEGOT_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (raceModes.NEGOT_STOPSIGN)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.APPR_HOOP) 
            
    #################################################             
    elif vehState.mode.currMode == raceModes.RECOV_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (raceModes.RECOV_STOPSIGN)    

    ########################################################################### 
    # HOOP MODES
    ###########################################################################                           
    elif vehState.mode.currMode == raceModes.APPR_HOOP:
        if vehState.mode.newMode():   
            playSound (raceModes.APPR_HOOP)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_HOOP)    
        
    #################################################
    elif vehState.mode.currMode == raceModes.NEGOT_HOOP:
        if vehState.mode.newMode():   
            playSound (raceModes.NEGOT_HOOP)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.APPR_BARRELS)   
        
    #################################################        
    elif vehState.mode.currMode == raceModes.RECOV_HOOP:
        if vehState.mode.newMode():   
            playSound (raceModes.RECOV_HOOP)
            
    ########################################################################### 
    # BARREL MODES
    ###########################################################################          
    elif vehState.mode.currMode == raceModes.APPR_BARRELS:
        if vehState.mode.newMode():   
            playSound (raceModes.APPR_BARRELS)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_BARRELS)  
        
    #################################################    
    elif vehState.mode.currMode == raceModes.NEGOT_BARRELS:
        if vehState.mode.newMode():   
            playSound (raceModes.NEGOT_BARRELS)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.APPR_RAMP)    
 
    #################################################    
    elif vehState.mode.currMode == raceModes.RECOV_BARRELS:
        if vehState.mode.newMode():   
            playSound (raceModes.RECOV_BARRELS)  
            
    ########################################################################### 
    # RAMP MODES
    ###########################################################################     
    elif vehState.mode.currMode == raceModes.APPR_RAMP:
        if vehState.mode.newMode():   
            playSound (raceModes.APPR_RAMP)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_RAMP)  
        
    #################################################    
    elif vehState.mode.currMode == raceModes.NEGOT_RAMP:
        if vehState.mode.newMode():   
            playSound (raceModes.NEGOT_RAMP)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.APPR_PED)   
        
    #################################################    
    elif vehState.mode.currMode == raceModes.RECOV_RAMP:
        if vehState.mode.newMode():   
            playSound (raceModes.RECOV_RAMP)   
            
    ########################################################################### 
    # PEDESTRIAN MODES
    ###########################################################################             
    elif vehState.mode.currMode == raceModes.APPR_PED:
        if vehState.mode.newMode():   
            playSound (raceModes.APPR_PED)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_PED)   
        
    #################################################   
    elif vehState.mode.currMode == raceModes.NEGOT_PED:
        if vehState.mode.newMode():   
            playSound (raceModes.NEGOT_PED)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.TERMINATE)
            
    ################################################# 
    elif vehState.mode.currMode == raceModes.RECOV_PED:
        if vehState.mode.newMode():   
            playSound (raceModes.RECOV_PED)
        
    ########################################################################### 
    # ERROR MODES
    ###########################################################################         
    elif vehState.mode.currMode == raceModes.ERROR:   
        if vehState.mode.newMode():  
            print "STATECONTROL - in ERROR State: %s" % (vehState.errorString)
            playSound (raceModes.ERROR)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.ERROR)   
        
    #################################################
    else:
        playSound (raceModes.UNKNOWN)       
        vehState.mode.errorString = "UNRECOGNIZED MODE"
        vehState.mode.setMode(raceModes.ERROR)    
    # endif
    
# end def

############################################################################### 
# obstacleTransition - figres out which state to transition to based on the
# obstacle reported by the vision system
###############################################################################
def obstacleTransition ():
    if vehState.obstacleType == obstacle.NONE:
        return raceModes.NONE       
    elif vehState.obstacleType == obstacle.PEDESTRIAN:
        return raceModes.APPR_PED
    elif vehState.obstacleType == obstacle.STOPSIGN:
        return raceModes.APPR_STOPSIGN
    elif vehState.obstacleType == obstacle.RAMP:
        return raceModes.APPR_RAMP
    elif vehState.obstacleType == obstacle.HOOP:
        return raceModes.APPR_HOOP
    elif vehState.obstacleType == obstacle.BARRELS:
        return raceModes.APPR_BARRELS
    elif vehState.obstacleType == obstacle.COURSE_END:
        return raceModes.WAIT_FOR_END
    elif vehState.obstacleType == obstacle.ALL:
        return raceModes.NONE
    else:
        vehState.errorString = "OBSTACLE TRANSITION ERROR - unknown obstacle type"
        return raceModes.ERROR
    #end
#end    

############################################################################### 
# playSound
###############################################################################    
def playSound (n):  
    raceModes.printMode("STATEMACHINE - Transistioned to")     
# end

###############################################################################
# Test code
###############################################################################
if __name__ == '__main__':
    stateMachine ()
    
    
