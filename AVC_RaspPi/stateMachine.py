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
from OccupGrid_v5_1 import *

if SIM_TEENSY:
    from serialClassSim  import serialClass
else:
    from serialClass     import serialClass
# end if SIM_TEENSY

apprCount       = 2     # Count of loops to stay in any of the appr states
BistMaxCnt      = 50    # 5 sec - max time for IOP to get to BIST
NormMaxCnt      = 20    # 2 sec - max time for IOP to enter NORM mode after cmd
simMaxCnt       = 100   #
ErrorMaxCnt     = 200   # Number of iterations before repeating error msg
printCnt        = 8     # print out the current mode every n cycles

###############################################################################
# stateControl - choose what to do depending on our current state
###############################################################################

def stateMachine (vehState, serialPort, occGrid):   
    ##*************************************************************************
    # Increment the mode counter so we know how long we've been in this mode
    ##*************************************************************************    
    vehState.mode.incCntr ()
    vehState.mode.printMode ("STATEMACHINE: Current Mode - ", printCnt) 
       
    # Check if Teensy went into ESTOP mode, but only if RPI not just coming up
    if (vehState.iopMode == IOP_MODE_ESTOP and vehState.mode.currMode != raceModes.INIT): 
        vehState.errorString = ( "IOP in EMERGANCY_STOP")
        vehState.mode.setMode(raceModes.ERROR)        
    
    ##*************************************************************************
    # Startup states
    ##*************************************************************************
    if vehState.mode.currMode == raceModes.INIT:
        if (vehState.iopMode == IOP_MODE_ESTOP):
            print("TEENSY was in ESTOP mode. Please wait...")            
            serialPort.sendCommand (CMD_MODE, 0, 0, 0)   # Go to BIST mode 
        else:
            # Teensy not in ESTOP, just go directly to WAIT_FOR_BIST mode
            vehState.mode.setMode (raceModes.WAIT_FOR_BIST)
    # end if

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.WAIT_FOR_BIST:
        if vehState.mode.newMode():
            print("TEENSY is booting. Please wait...")            
            serialPort.sendCommand (CMD_MODE, 0, 0, 0)   # Go to BIST mode

        # Are we in BIST mode yet?
        if (vehState.iopMode == IOP_MODE_BIST):       
             # Yep, we're in BIST mode, check if there was a BIST error
            if (vehState.iopBistStatus == 0):
                # No error, wait for start of Teensy to be in NORMAL mode
                serialPort.sendCommand ('D', 1, 0, 0)   # Go to NORMAL mode                
                vehState.mode.setMode(raceModes.WAIT_FOR_NORM)                
            else:
                # We had a bist error somewhere...
                print("Teensy in error mode. Reset.")               
                vehState.errorString = ( "IOP BIST FAILURE")
                    #"IOP BIST FAILURE, value %d" % (vehState.iopBistStatus))
                vehState.mode.setMode (raceModes.ERROR)    
            # end if 
        else:
            # We're not in BIST mode yet, has it been too long?
            if (vehState.mode.modeCount > BistMaxCnt):
              vehState.errorString = ("IOP NOT COMMUNICATING")
              vehState.mode.setMode (raceModes.ERROR)
            # end if   
    
    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.WAIT_FOR_NORM:    
        
        if (vehState.iopMode == IOP_MODE_NORMAL):
            vehState.mode.setMode(raceModes.WAIT_FOR_START)    
        else:                    
            # Wait up to NormMaxCnt for IOP to be normal mode
            if (vehState.mode.modeCount > NormMaxCnt):
                vehState.errorString = "IOP NOT GOING TO NORMAL MODE"
                vehState.mode.setMode (raceModes.ERROR)

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.WAIT_FOR_START:
        if vehState.mode.newMode():
            # IOP is now in NORMAL mode so set steering.  
            serialPort.sendCommand (CMD_MOVE, 0, 0, 0)
            serialPort.sendCommand (CMD_TURN, 1, 30000, 0)           

        else:
            # Did we get the start switch closure?
            if (vehState.iopStartSwitch):     # We're Off!
                vehState.mode.setMode(raceModes.RACE_BEGIN)
                # Send the move command ASAP
                serialPort.sendCommand (CMD_MOVE, vehState.mode.getSpeed(), 100, 0)
            # end
        # end

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.RACE_BEGIN:
        # This is just a transitory state to initialize things
        # Clear out the occ grid and then move to RACE
        occGrid.clear(0, 0)
        vehState.mode.setMode(raceModes.RACE_STRAIGHT)
        # end

    ##*************************************************************************
    # NORMAL RACE STATES
    ##*************************************************************************
    elif vehState.mode.currMode == raceModes.RACE_STRAIGHT:

        serialPort.sendCommand (CMD_MOVE, vehState.mode.getSpeed(), 100, 0)
        serialPort.sendCommand (CMD_TURN, vehState.histAngle, 1000, 0)

        # If we got an obstacle sighting from the vision system transition
        obstacleTransition (vehState)
        
    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.RACE_CURVE:

        # If we got an obstacle sighting from the vision system transition
        obstacleTransition (vehState)

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.NEGOT_CROSSING:

        # If we got an obstacle sighting from the vision system transition
        obstacleTransition (vehState)

    ##*************************************************************************
    # STOPSIGN STATES
    ##*************************************************************************
    elif vehState.mode.currMode == raceModes.APPR_STOPSIGN:

        # Make sure we still spot the obstacle otherwise just go back to RACE
        if (vehState.obstacleType != OBSTACLE_STOPSIGN):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)

        # If we've been in the approach state long enough go to negotiate
        if (vehState.mode.modeCount >= apprCount):
            vehState.mode.setMode(raceModes.NEGOT_STOPSIGN)

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.NEGOT_STOPSIGN:

        if (vehState.mode.modeCount >= simMaxCnt):
            vehState.mode.setMode(raceModes.RECOV_STOPSIGN)

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.RECOV_STOPSIGN:

        if (vehState.mode.modeCount >= simMaxCnt):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)

    ##*************************************************************************
    # HOOP STATES
    ##*************************************************************************
    elif vehState.mode.currMode == raceModes.APPR_HOOP:

        # Make sure we still spot the obstacle otherwise just go back to RACE
        if (vehState.obstacleType != OBSTACLE_HOOP):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)

        # If we've been in the approach state long enough go to negotiate
        if (vehState.mode.modeCount >= apprCount):
            vehState.mode.setMode(raceModes.NEGOT_HOOP)

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.NEGOT_HOOP:

        if (vehState.mode.modeCount >= simMaxCnt):
            vehState.mode.setMode(raceModes.RECOV_HOOP)

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.RECOV_HOOP:

        if (vehState.mode.modeCount >= simMaxCnt):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)

    ##*************************************************************************
    # BARREL STATES
    ##*************************************************************************
    elif vehState.mode.currMode == raceModes.APPR_BARRELS:

        # Make sure we still spot the obstacle otherwise just go back to RACE
        if (vehState.obstacleType != OBSTACLE_BARRELS):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)

        # If we've been in the approach state long enough go to negotiate
        if (vehState.mode.modeCount >= apprCount):
            vehState.mode.setMode(raceModes.NEGOT_BARRELS)

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.NEGOT_BARRELS:

        if (vehState.mode.modeCount >= simMaxCnt):
            vehState.mode.setMode(raceModes.RECOV_BARRELS)

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.RECOV_BARRELS:

        if (vehState.mode.modeCount >= simMaxCnt):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)

    ##*************************************************************************
    # RAMP STATES
    ##*************************************************************************
    elif vehState.mode.currMode == raceModes.APPR_RAMP:

        # Make sure we still spot the obstacle otherwise just go back to RACE
        if (vehState.obstacleType != OBSTACLE_RAMP):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)

        # If we've been in the approach state long enough go to negotiate
        if (vehState.mode.modeCount >= apprCount):
            vehState.mode.setMode(raceModes.NEGOT_RAMP)

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.NEGOT_RAMP:

        if (vehState.mode.modeCount >= simMaxCnt):
            vehState.mode.setMode(raceModes.RECOV_RAMP)

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.RECOV_RAMP:

        if (vehState.mode.modeCount >= simMaxCnt):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)

    ##*************************************************************************
    # PEDESTRIAN STATES
    ##*************************************************************************
    elif vehState.mode.currMode == raceModes.APPR_PED:

        # Make sure we still spot the obstacle otherwise just go back to RACE
        if (vehState.obstacleType != OBSTACLE_PEDESTRIAN):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)

        # If we've been in the approach state long enough go to negotiate
        if (vehState.mode.modeCount >= apprCount):
            vehState.mode.setMode(raceModes.NEGOT_STOPSIGN)

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.NEGOT_STOPSIGN:

        if (vehState.mode.modeCount >= simMaxCnt):
            vehState.mode.setMode(raceModes.RECOV_STOPSIGN)

    #------------------------------------------------------
    elif vehState.mode.currMode == raceModes.RECOV_STOPSIGN:

        if (vehState.mode.modeCount >= simMaxCnt):
            vehState.mode.setMode(raceModes.RACE_STRAIGHT)

    ##*************************************************************************
    # TERMINATE RACE STATES
    ##*************************************************************************
    elif vehState.mode.currMode == raceModes.WAIT_FOR_END:

        if (vehState.mode.modeCount >= simMaxCnt):
            vehState.mode.setMode(raceModes.NEGOT_END)

    elif vehState.mode.currMode == raceModes.NEGOT_END:
        if vehState.mode.newMode():
            playSound (vehState)

        if (vehState.mode.modeCount >= simMaxCnt):
            vehState.mode.setMode(raceModes.NEGOT_END)

    elif vehState.mode.currMode == raceModes.TERMINATE:
        if vehState.mode.newMode():
            #We'll stay in terminate state from now on
            playSound (vehState)

    ##*************************************************************************
    # ERROR STATES
    ##*************************************************************************
    elif vehState.mode.currMode == raceModes.ERROR:
        vehState.mode.printMode (vehState.errorString, printCnt) 

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
    if vehState.obstacleType == OBSTACLE_PEDESTRIAN:
        vehState.mode.setMode(raceModes.APPR_PED)

    elif vehState.obstacleType == OBSTACLE_STOPSIGN:
        vehState.mode.setMode(raceModes.APPR_STOPSIGN)

    elif vehState.obstacleType == OBSTACLE_RAMP:
        vehState.mode.setMode(raceModes.APPR_RAMP)

    elif vehState.obstacleType == OBSTACLE_HOOP:
        vehState.mode.setMode(raceModes.APPR_HOOP)

    elif vehState.obstacleType == OBSTACLE_BARRELS:
        vehState.mode.setMode(raceModes.APPR_BARRELS)

    elif vehState.obstacleType == OBSTACLE_COURSE_END:
        vehState.mode.setMode(raceModes.WAIT_FOR_END)
    #end
#end

###############################################################################
# playSound
###############################################################################
def playSound (vehState):
    pass
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
            print("MAINLOOP:GET_IOPTLM - nPkts tlm_cnt, \
                Time vehState.iopTime, Mode vehState.iopMode, \
                AccCnt vehState.iopAcceptCnt, Switch vehState.iopSwitchStatus/vehState.iopStartSwitch")
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
        vehState.iopLeftEncoder = telemArray[22]
        vehState.iopRightEncoder= telemArray[23]

        if  False:
            print("MAINLOOP:PROC_IOPTLM - Time vehState.iopTime, Mode \
                vehState.iopMode, AccCnt vehState.iopAcceptCnt, Switch \
                vehState.iopSwitchStatus/vehState.iopStartSwitch")
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
    vehState.mode.setMode (raceModes.INIT)

    for i in range(0, 100):
        stateMachine (vehState, serialPort)

        if (i == 30):            # try STOPSIGN state
            vehState.obstacleType = OBSTACLE_STOPSIGN
        if (i == 35):            # reset obstacle
            vehState.obstacleType = OBSTACLE_NONE

        if (i == 40):            # try HOOP state
            vehState.obstacleType = OBSTACLE_HOOP
        if (i == 45):            # reset obstacle
            vehState.obstacleType = OBSTACLE_NONE

        if (i == 50):            # try BARRELS state
            vehState.obstacleType = OBSTACLE_BARRELS
        if (i == 55):            # reset obstacle
            vehState.obstacleType = OBSTACLE_NONE

        if (i == 60):            # try RAMP state
            vehState.obstacleType = OBSTACLE_RAMP
        if (i == 65):            # reset obstacle
            vehState.obstacleType = OBSTACLE_NONE

        if (i == 70):            # try APPR_PED state
            vehState.obstacleType = OBSTACLE_PEDESTRIAN
        if (i == 75):            # reset obstacle
            vehState.obstacleType = OBSTACLE_NONE

        if (i == 80):            # try APPR_PED state
            vehState.obstacleType = OBSTACLE_COURSE_END
        if (i == 85):            # reset obstacle
            vehState.obstacleType = OBSTACLE_NONE

        get_iopTestTlm(IopTlmQueue, vehState)
        time.sleep (0.6)
    # end
    serialPort.killThread()
