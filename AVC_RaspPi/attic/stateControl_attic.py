"""
 stateControl.py 
 Called by mainLoop every 100 mSec.  Determines what need to happen in each
 state
 
 Written by David Gutow 9/2017
"""

############################################################################### 
# 
###############################################################################    
def playSound (n):  
    print ("=============================================>>>> Playing Sound %d\n" % (n))
# end

############################################################################### 
# 
###############################################################################

from vehicleState   import *  
from raceModes      import raceModes
import  mainLoop    

simMaxCnt   = 20            # max count for use during simulations
InitMaxCnt  = 20            # Max time without an IOP msg before we signal a problem
BistMaxCnt  = 30            # Max time IOP is in BIST befoe we signal a problem

maxSpeed    = 100           # Maximum speed for use during normal racing
apprSpeed   = 50            # Speed to use when approaching an obstacle

############################################################################### 
# stateControl - choose what to do depending on our current state
###############################################################################

def stateControl (vehState):
    #################################################
    if vehState.mode.currMode == raceModes.NONE:
        if vehState.mode.newMode():
            playSound (0)   
            
        if (vehState.iopMode == iopModes.NO_MODE):    
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
            playSound (1)     
         
        # Check if the IOP BIST is complete
        if (vehState.iopMode <> iopModes.BIST_MODE):
            if (vehState.mode.modeCount > BistMaxCnt):
                vehState.errorString = "IOP BIST FAILURE"
                vehState.mode.setMode (raceModes.ERROR)  
        else:
            vehState.mode.setMode(raceModes.WAIT_FOR_START)       
        # end if             
        
    #################################################        
    elif vehState.mode.currMode == raceModes.WAIT_FOR_START:
        if vehState.mode.newMode():   
            playSound (2)   
            send_command ('D', 0, 0, 0)

        if (vehState.iopStartSwStatus):     # We're Off!
            vehState.mode.setMode(raceModes.RACE_STRAIGHT) 
            
    ########################################################################### 
    # NORMAL RACE MODES
    ###########################################################################                
    elif vehState.mode.currMode == raceModes.RACE_STRAIGHT:
        if vehState.mode.newMode():   
            playSound (3)   
            
        if (vehState.mode.modeCount >= simMaxCnt):    
            vehState.mode.setMode(raceModes.RACE_CURVE)   
        
    #################################################         
    elif vehState.mode.currMode == raceModes.RACE_CURVE:
        if vehState.mode.newMode():   
            playSound (4)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_CROSSING) 
        
    #################################################         
    elif vehState.mode.currMode == raceModes.NEGOT_CROSSING:
        if vehState.mode.newMode():   
            playSound (5)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.APPR_STOPSIGN)  
        
    ########################################################################### 
    # STOPSIGN MODES
    ###########################################################################                           
    elif vehState.mode.currMode == raceModes.APPR_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (6)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_STOPSIGN) 
        
    #################################################         
    elif vehState.mode.currMode == raceModes.NEGOT_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (7)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.APPR_HOOP) 
            
    #################################################             
    elif vehState.mode.currMode == raceModes.RECOV_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (7)    

    ########################################################################### 
    # HOOP MODES
    ###########################################################################                           
    elif vehState.mode.currMode == raceModes.APPR_HOOP:
        if vehState.mode.newMode():   
            playSound (8)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_HOOP)    
        
    #################################################
    elif vehState.mode.currMode == raceModes.NEGOT_HOOP:
        if vehState.mode.newMode():   
            playSound (9)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.APPR_BARRELS)   
        
    #################################################        
    elif vehState.mode.currMode == raceModes.RECOV_HOOP:
        if vehState.mode.newMode():   
            playSound (8)
            
    ########################################################################### 
    # BARREL MODES
    ###########################################################################          
    elif vehState.mode.currMode == raceModes.APPR_BARRELS:
        if vehState.mode.newMode():   
            playSound (10)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_BARRELS)  
        
    #################################################    
    elif vehState.mode.currMode == raceModes.NEGOT_BARRELS:
        if vehState.mode.newMode():   
            playSound (4)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.APPR_RAMP)    
 
    #################################################    
    elif vehState.mode.currMode == raceModes.RECOV_BARRELS:
        if vehState.mode.newMode():   
            playSound (10)  
            
    ########################################################################### 
    # RAMP MODES
    ###########################################################################     
    elif vehState.mode.currMode == raceModes.APPR_RAMP:
        if vehState.mode.newMode():   
            playSound (11)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_RAMP)  
        
    #################################################    
    elif vehState.mode.currMode == raceModes.NEGOT_RAMP:
        if vehState.mode.newMode():   
            playSound (12)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.APPR_PED)   
        
    #################################################    
    elif vehState.mode.currMode == raceModes.RECOV_RAMP:
        if vehState.mode.newMode():   
            playSound (12)   
            
    ########################################################################### 
    # PEDESTRIAN MODES
    ###########################################################################             
    elif vehState.mode.currMode == raceModes.APPR_PED:
        if vehState.mode.newMode():   
            playSound (13)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_PED)   
        
    #################################################   
    elif vehState.mode.currMode == raceModes.NEGOT_PED:
        if vehState.mode.newMode():   
            playSound (14)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.TERMINATE)
            
    ################################################# 
    elif vehState.mode.currMode == raceModes.RECOV_PED:
        if vehState.mode.newMode():   
            playSound (13)
        
    ########################################################################### 
    # ERROR MODES
    ###########################################################################         
    elif vehState.mode.currMode == raceModes.ERROR:   
        if vehState.mode.newMode():  
            print "STATECONTROL - in ERROR State: %s" % (vehState.errorString)
            playSound (15)   
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.ERROR)   
        
    #################################################
    else:
        playSound (16)       
        vehState.mode.errorString = "UNRECOGNIZED MODE"
        vehState.mode.setMode(raceModes.ERROR)    
    # endif
    
# end def

###############################################################################
# Test code
###############################################################################
if __name__ == '__main__':
    stateControl ()
    
    
