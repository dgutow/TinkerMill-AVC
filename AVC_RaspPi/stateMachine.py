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
from printOut       import *
from OccupGrid_Rich import *

if SIM_TEENSY:
    from serialClassSim  import serialClass
else:    
    from serialClass     import serialClass
# end if SIM_TEENSY 

apprCount       = 2     # Count of loops to stay in any of the appr states
BistMaxCnt      = 90    # 3 sec - max time for IOP to get to BIST
NormMaxCnt      = 30    # 2 sec - max time for IOP to enter NORM mode after cmd 
simMaxCnt       = 100   # 
ErrorMaxCnt     = 200   # Number of iterations before repeating error msg

hist = Histogram(origin=[0.5 * ogNcols * ogResolution, 0], scanAngle=45, angDelta=3)

maxDist = sqrt((((ogNcols / 2) * ogResolution) ** 2) + ((ogNrows * ogResolution) ** 2))

############################################################################### 
# stateControl - choose what to do depending on our current state
###############################################################################

def stateMachine (vehState, serialPort, occGrid):
    # vehState.mode.printMode("STATEMACHINE:")  
    
    ##*************************************************************************
    # Startup states
    ##*************************************************************************      
    if vehState.mode.currMode == raceModes.NONE:
        if vehState.mode.newMode():
            playSound (vehState) 
            
        # Just go directly to WAIT_FOR_BIST mode
        vehState.mode.setMode (raceModes.WAIT_FOR_BIST)
    # end if
        
    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.WAIT_FOR_BIST:
        if vehState.mode.newMode():   
            serialPort.sendCommand (cmdGoToMode, 0, 0, 0)   # Go to BIST mode 
            playSound (vehState)     
         
        # Are we in BIST mode yet?
        if (vehState.iopMode <> IOP_MODE_BIST):       
            # Not yet - wait for up to BistMaxCnt for IOP BIST mode
            if (vehState.mode.modeCount > BistMaxCnt):
              vehState.errorString = ("IOP NOT COMMUNICATING")
              vehState.mode.setMode (raceModes.ERROR) 
            # end if       
        else:
            # Yep, we're in BIST mode, was there a BIST error
            if (vehState.iopBistStatus <> 0):
                vehState.errorString = ( "IOP BIST FAILURE")
                    #"IOP BIST FAILURE, value %d" % (vehState.iopBistStatus))
                vehState.mode.setMode (raceModes.ERROR)  
            else:
                vehState.mode.setMode(raceModes.WAIT_FOR_START) 
        # end if             
        
    #------------------------------------------------------        
    elif vehState.mode.currMode == raceModes.WAIT_FOR_START:
        if vehState.mode.newMode():   
            serialPort.sendCommand (cmdGoToMode, 1, 0, 0)   # Go to NORMAL mode        
            playSound (vehState)  
            
        # end

        if (vehState.iopMode <> IOP_MODE_NORMAL):
            # Wait up to NormMaxCnt for IOP to be normal mode        
            if (vehState.mode.modeCount > NormMaxCnt):
                vehState.errorString = "IOP NOT GOING TO NORMAL MODE"
                vehState.mode.setMode (raceModes.ERROR) 
            # end if               
        else:
            # IOP is in NORMAL mode so set steering.  We'll send this
            # command every cycle but who cares?
            #serialPort.sendCommand (cmdTurn, 0, 0, 0)   # Set steering to 0
        
            # Did we get the start switch closure?
            if (vehState.iopStartSwitch):     # We're Off!
                # Clear out the occ grid and then move to RACE
                occGrid.clear()    
                vehState.mode.setMode(raceModes.RACE_BEGIN)                                             
            #end
        # end
        
    #------------------------------------------------------         
    #------------------------------------------------------        
    elif vehState.mode.currMode == raceModes.RACE_BEGIN:
        if vehState.mode.newMode():   
            playSound (vehState)   
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)             
        # end

        # This is just a transitory state to allow the occGrid to initialize       
        vehState.mode.setMode(raceModes.RACE_STRAIGHT) 
        # end
         
    ##*************************************************************************
    # NORMAL RACE STATES
    ##*************************************************************************                
    elif vehState.mode.currMode == raceModes.RACE_STRAIGHT:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
        
        angle = hist.getAngle(hist.getSlices(hist.getCostArray(occGrid, maxDist), hist.scanAngle, hist.angDelta), 0)
        print ("Histogram Angle = ", angle)
        serialPort.sendCommand (cmdTurn, angle, 0, 0)  
            
        # If we got an obstacle sighting from the vision system transition        
        obstacleTransition (vehState)      
          
    #------------------------------------------------------         
    #------------------------------------------------------         
    elif vehState.mode.currMode == raceModes.RACE_CURVE:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        # If we got an obstacle sighting from the vision system transition
        newState = obstacleTransition (vehState)  
        
    #------------------------------------------------------        
    #------------------------------------------------------         
    elif vehState.mode.currMode == raceModes.NEGOT_CROSSING:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        # If we got an obstacle sighting from the vision system transition
        newState = obstacleTransition (vehState)   
        
    ##************************************************************************* 
    # STOPSIGN STATES
    ##*************************************************************************                           
    elif vehState.mode.currMode == raceModes.APPR_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        # Make sure we still spot the obstacle otherwise just go back to RACE
        if (vehState.obstacleType <> obstacle.STOPSIGN):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)
         
        # If we've been in the approach state long enough go to negotiate
        if (vehState.mode.modeCount >= apprCount):     
            vehState.mode.setMode(raceModes.NEGOT_STOPSIGN) 
    #------------------------------------------------------         
    #------------------------------------------------------         
    elif vehState.mode.currMode == raceModes.NEGOT_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.RECOV_STOPSIGN) 
    #------------------------------------------------------             
    #------------------------------------------------------             
    elif vehState.mode.currMode == raceModes.RECOV_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)           

    ##************************************************************************* 
    # HOOP STATES
    ##*************************************************************************                           
    elif vehState.mode.currMode == raceModes.APPR_HOOP:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        # Make sure we still spot the obstacle otherwise just go back to RACE
        if (vehState.obstacleType <> obstacle.HOOP):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)
         
        # If we've been in the approach state long enough go to negotiate
        if (vehState.mode.modeCount >= apprCount):     
            vehState.mode.setMode(raceModes.NEGOT_HOOP)    
        
    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.NEGOT_HOOP:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.RECOV_HOOP)   
        
    #------------------------------------------------------        
    elif vehState.mode.currMode == raceModes.RECOV_HOOP:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)            
            
    ##************************************************************************* 
    # BARREL STATES
    ##*************************************************************************          
    elif vehState.mode.currMode == raceModes.APPR_BARRELS:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        # Make sure we still spot the obstacle otherwise just go back to RACE
        if (vehState.obstacleType <> obstacle.BARRELS):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)
         
        # If we've been in the approach state long enough go to negotiate
        if (vehState.mode.modeCount >= apprCount):     
            vehState.mode.setMode(raceModes.NEGOT_BARRELS)  
        
    #------------------------------------------------------    
    elif vehState.mode.currMode == raceModes.NEGOT_BARRELS:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.RECOV_BARRELS)    
 
    #------------------------------------------------------    
    elif vehState.mode.currMode == raceModes.RECOV_BARRELS:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
                        
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)             
            
    ##************************************************************************* 
    # RAMP STATES
    ##*************************************************************************     
    elif vehState.mode.currMode == raceModes.APPR_RAMP:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        # Make sure we still spot the obstacle otherwise just go back to RACE
        if (vehState.obstacleType <> obstacle.RAMP):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)
         
        # If we've been in the approach state long enough go to negotiate
        if (vehState.mode.modeCount >= apprCount):     
            vehState.mode.setMode(raceModes.NEGOT_RAMP)  
        
    #------------------------------------------------------    
    elif vehState.mode.currMode == raceModes.NEGOT_RAMP:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.RECOV_RAMP)   
        
    #------------------------------------------------------    
    elif vehState.mode.currMode == raceModes.RECOV_RAMP:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.RACE_STRAIGHT) 
            
    ##************************************************************************* 
    # PEDESTRIAN STATES
    ##*************************************************************************             
    elif vehState.mode.currMode == raceModes.APPR_PED:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        # Make sure we still spot the obstacle otherwise just go back to RACE
        if (vehState.obstacleType <> obstacle.PEDESTRIAN):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)
         
        # If we've been in the approach state long enough go to negotiate
        if (vehState.mode.modeCount >= apprCount):     
            vehState.mode.setMode(raceModes.NEGOT_STOPSIGN)   
        
    #------------------------------------------------------   
    elif vehState.mode.currMode == raceModes.NEGOT_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
                        
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.RECOV_STOPSIGN)
            
    #------------------------------------------------------ 
    elif vehState.mode.currMode == raceModes.RECOV_STOPSIGN:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)             
        
    ##************************************************************************* 
    # TERMINATE RACE STATES
    ##*************************************************************************        
    elif vehState.mode.currMode == raceModes.WAIT_FOR_END:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end    
            
        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_END)            
            
    elif vehState.mode.currMode == raceModes.NEGOT_END:
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end 

        if (vehState.mode.modeCount >= simMaxCnt):     
            vehState.mode.setMode(raceModes.NEGOT_END)  
            
    elif vehState.mode.currMode == raceModes.TERMINATE:  
        if vehState.mode.newMode():   
            playSound (vehState) 
            serialPort.sendCommand (cmdMove, vehState.mode.getSpeed(), 99, 0)
        # end            
        #We'll stay in terminate state from now on
        
            
    ##************************************************************************* 
    # ERROR STATES
    ##*************************************************************************         
    elif vehState.mode.currMode == raceModes.ERROR:   
        if vehState.mode.newMode(): 
            playSound (vehState)        
            printOut  ( "STATECONTROL - in ERROR State: %s" % (vehState.errorString))
     
        if (vehState.mode.modeCount >= ErrorMaxCnt):     
            vehState.mode.setMode(raceModes.ERROR)   
        
    #------------------------------------------------------
    else:
        # Something is drastically wrong here, we didn't recognize the state
        # Go to unknown state first for the playSound
        vehState.mode.setMode(raceModes.UNKNOWN) 
        playSound (vehState)       
        vehState.mode.errorString = "UNRECOGNIZED MODE"
        vehState.mode.setMode(raceModes.ERROR)    
    # endif
    
# end def

############################################################################### 
# obstacleTransition - figres out which state to transition to based on the
# obstacle reported by the vision system
###############################################################################

def obstacleTransition (vehState):
    if vehState.obstacleType == obstacle.NONE:
        return raceModes.NONE 
        
    elif vehState.obstacleType == obstacle.PEDESTRIAN:
        vehState.mode.setMode(raceModes.APPR_PED)
        
    elif vehState.obstacleType == obstacle.STOPSIGN:
        vehState.mode.setMode(raceModes.APPR_STOPSIGN)    
        
    elif vehState.obstacleType == obstacle.RAMP:
        vehState.mode.setMode(raceModes.APPR_RAMP)     
        
    elif vehState.obstacleType == obstacle.HOOP:
        vehState.mode.setMode(raceModes.APPR_HOOP)     
        
    elif vehState.obstacleType == obstacle.BARRELS:
        vehState.mode.setMode(raceModes.APPR_BARRELS)     
        
    elif vehState.obstacleType == obstacle.COURSE_END:
        vehState.mode.setMode(raceModes.WAIT_FOR_END)     
        
    else:
        vehState.errorString = "OBSTACLE TRANSITION ERROR - unknown obstacle type"
        return raceModes.ERROR
    #end
#end    

############################################################################### 
# playSound
###############################################################################    
def playSound (vehState):  
    vehState.mode.printMode("STATEMACHINE: Transitioned to")     
# end

###############################################################################
# Test code
###############################################################################
if __name__ == '__main__':
    from Queue           import Queue
    import time
    import struct   
    
    #import pdb 
    #pdb.set_trace()

    ################################################################################
    # get_iopTestTlm( )
    ################################################################################
    def get_iopTestTlm(IopTlmQueue, vehState):
        time = 0
        msg = ""
        
        # Get the last message put onto the queue
        tlm_cnt = 0
        # Keep looping until we get the last message put onto the queue    
        while (not IopTlmQueue.empty()):
            msg = IopTlmQueue.get_nowait()
            #printOut("MAINLOOP:GET_IOPTLM - msg length (%d)" % ( len(msg)) )
            #printOut("MAINLOOP:GET_IOPTLM - got msg (%s)" % ( msg) )
            time = proc_iopTestTlm(msg, vehState)        
            tlm_cnt += 1
        # end while
    
        if (tlm_cnt > 0):
            print "MAINLOOP:GET_IOPTLM - nPkts %d, Time %3d, Mode %1d, AccCnt %2d, Switch %2d/%2d" % (
                tlm_cnt, vehState.iopTime, vehState.iopMode, vehState.iopAcceptCnt, 
                vehState.iopSwitchStatus, vehState.iopStartSwitch) 
        else:
            #print ("MAINLOOP:GET_IOPTLM - no new telemetry ")         
            pass
                
        return (msg)        # We'll return the last message on queue
    # end    
        
        
    ################################################################################
    # proc_iopTestTlm - process_telemetry
    ################################################################################
    
    def proc_iopTestTlm (data, vehState):        
        #try:
        telemArray  = struct.unpack('<LLhhhhhhhhhhhhhhhhhhhhhh', data)
        #except:      
            # print ("MAINLOOP:PROC_IOPTLM - ERROR unable to parse telemetry")
            # print ("MAINLOOP:PROC_IOPTLM - Length of data is ", len(data) )             
            # return 0
        
        pktId                   = telemArray[0]
        vehState.iopTime        = telemArray[1]
        vehState.iopMode        = telemArray[2]
        vehState.iopAcceptCnt   = telemArray[3]
        vehState.iopBistStatus  = telemArray[4]
        vehState.iopSpeed       = telemArray[5]
        vehState.iopSteerAngle  = telemArray[6]         
        vehState.iopCumDistance = telemArray[7]
        
        # Enter the side IR sensors into the two rangeSensorPairs
        irLF_Range              = telemArray[8]
        irLR_Range              = telemArray[9]        
        irRF_Range              = telemArray[10]
        irRR_Range              = telemArray[11]
        
        vehState.iopSwitchStatus= telemArray[12]  
        vehState.iopStartSwitch = telemArray[12] & 0x01 
        
        measScanAngle           = telemArray[13]
        measRejCnt          = telemArray[14]
        measRejReason            = telemArray[15]
    
        vehState.iopBattVolt1   = telemArray[16]
        vehState.iopBattVolt2   = telemArray[17]
        vehState.iopAccelVert   = telemArray[18]
        vehState.iopGyroHoriz   = telemArray[19]
        vehState.iopCompAngle   = telemArray[20]
        vehState.iopCameraAngle = telemArray[21]        
        vehState.iopSpare2      = telemArray[22]
        vehState.iopSpare3      = telemArray[23]   
        
        if  False:
            print "MAINLOOP:PROC_IOPTLM - Time %3d, Mode %1d, AccCnt %2d, Switch %2d/%2d" % (
            vehState.iopTime, vehState.iopMode, vehState.iopAcceptCnt, 
            vehState.iopSwitchStatus, vehState.iopStartSwitch)   
            #print "PROCESS_TELEM - Bist %d, Speed %d, SteerAng %d, Distance  %d" % (
            # vehState.iopBistStatus, vehState.iopSpeed, vehState.iopSteerAngle, 
            # vehState.iopCumDistance)                  
        #end
        if False:
            hexArr = [hex(ord(val)) for val in data]
            print ("PROCESS_TELEM - %s,%s,%s,%s,%s,%s,%s" % 
                (hexArr[0], hexArr[1], hexArr[2], hexArr[3], 
                hexArr[4], hexArr[5], hexArr[6]) )
        #end
        
        #######################################################################
        # Let's make sure we're always operating on compass angles which range
        # -180 to +180, rather than 0 to 360 degrees.
        if (vehState.iopCompAngle > 180):
            vehState.iopCompAngle -= 360
        #print ("MAINLOOP:PROC_IOPTLM - 1")
        # #######################################################################  
        # # Enter the scanner info into the iopRanges buffer
        # vehState.iopRanges.enterRange(time   = vehState.iopTime, 
                                    # angle  = measScanAngle, 
                                    # ranger = measScanSensor, 
                                    # range  = measScanDist) 
        # #print ("MAINLOOP:PROC_IOPTLM - 2")                                      
        # #######################################################################                                      
        # # Enter the scanner info into the occupGrid - DAG should we 
        # # enter the data if we are using the short range sensor?
        # occGrid.enterRange ( carCumDist   = vehState.iopCumDistance, 
                            # carCurrAngle = vehState.iopCompAngle, 
                            # scanDist     = measScanDist, 
                            # scanAngle    = measScanAngle)       
        # #print ("MAINLOOP:PROC_IOPTLM - 3")                             
        # #######################################################################        
        # # Enter the side IR sensor data into the two rangeSensorPairs
        # if not SIM_TEENSY:
            # rangeLeftPair.newMeasurement (measFrontRange = irLF_Range, 
                                    # measRearRange  = irLR_Range)                                     
            # rangeRightPair.newMeasurement(measFrontRange = irRF_Range, 
                                    # measRearRange  = irRR_Range)   
        # # end SIM_TEENSY     
        # # print ("MAINLOOP:PROC_IOPTLM - 4 end")        
        return vehState.iopTime
    # end
    
    ################################################################################
    # TEST CODE - TEST CODE
    ################################################################################
    # IopTlmQueue is used to pass telemetry packets from the IOP serial port thread
    IopTlmQueue  = Queue(50)
    serialPort   = serialClass (IopTlmQueue) # Serial port setup and support
    
    vehState     = vehicleState()
    vehState.mode.setMode (raceModes.NONE)
        
    for i in range(0, 100):
        stateMachine (vehState, serialPort)   
        
        if (i == 30):            # try STOPSIGN state
            vehState.obstacleType = obstacle.STOPSIGN          
        if (i == 35):            # reset obstacle
            vehState.obstacleType = obstacle.NONE              
  
        if (i == 40):            # try HOOP state
            vehState.obstacleType = obstacle.HOOP      
        if (i == 45):            # reset obstacle
            vehState.obstacleType = obstacle.NONE             
          
        if (i == 50):            # try BARRELS state
            vehState.obstacleType = obstacle.BARRELS         
        if (i == 55):            # reset obstacle
            vehState.obstacleType = obstacle.NONE             
         
        if (i == 60):            # try RAMP state  
            vehState.obstacleType = obstacle.RAMP            
        if (i == 65):            # reset obstacle
            vehState.obstacleType = obstacle.NONE             
          
        if (i == 70):            # try APPR_PED state  
            vehState.obstacleType = obstacle.PEDESTRIAN           
        if (i == 75):            # reset obstacle
            vehState.obstacleType = obstacle.NONE   

        if (i == 80):            # try APPR_PED state  
            vehState.obstacleType = obstacle.COURSE_END           
        if (i == 85):            # reset obstacle
            vehState.obstacleType = obstacle.NONE               
          
        get_iopTestTlm(IopTlmQueue, vehState)
        time.sleep (0.6)                                    
    # end    
    serialPort.killThread()
