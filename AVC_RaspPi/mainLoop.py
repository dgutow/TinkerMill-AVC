"""
 mainLoop.py - The main loop of the racing vehicle.
 
 Written by David Gutow 8/2017
"""

import time
import threading
import struct
from Queue           import Queue
from simulator       import *       # Only for the simulator
from vehicleState    import *       # Everything we know about the vehicle
from constants       import *       # Vehicle and course constants
from stateMachine    import stateMachine
from rangeSensorPair import rangeSensorPair
from raceModes       import raceModes
from rangeClass      import Range
from OccupGrid       import Grid
from guiInterface    import guiIfClass
from printOut        import *

if WINDOWS:
    from serialClassSim  import serialClass
else:    
    from serialClass     import serialClass
# end if WINDOWS    

###############################################################################
# Global variables 
###############################################################################
# Vehicle State holds everything known about the current vehicle state
vehState        = vehicleState()

# The two side IT range sensor pairs
rangeLeftPair   = rangeSensorPair(initFrontAng   = rsLeftFrontAng, 
                              initRearAng    = rsLeftRearAng, 
                              initSensorDist = rsFRspacing, 
                              initMinDist    = rsMinDistance,
                              initMaxDist    = rsMaxDistance, 
                              rightSide      = rsLeftSide)
                              
rangeRightPair  = rangeSensorPair(initFrontAng   = rsRightFrontAng, 
                              initRearAng    = rsRightRearAng, 
                              initSensorDist = rsFRspacing, 
                              initMinDist    = rsMinDistance,
                              initMaxDist    = rsMaxDistance, 
                              rightSide      = rsRigthSide)                             

# The vehicle occupancy grid
occGrid       = Grid (ogResolution, ogNrows, ogNcols, ogStartDist, ogStartAngle)

# IopTlmQueue is used to pass telemetry packets from the IOP serial port thread
IopTlmQueue  = Queue(50)

# visionTlmQueue is queue to send telemetry packets from the vision task
# visionCmdQueue is queue to send command packets to the vision task
visionTlmQueue  = Queue(10)
visionCmdQueue  = Queue(10)

# Serial port setup and support
serialPort  = serialClass (IopTlmQueue)  

# Interface to the GUI host
guiIf = guiIfClass (GUI_IPADDR, GUI_MAINPORT, GUI_IOPPORT, GUI_VISPORT)

############################################################################### 
# Initialize the entire system
###############################################################################

def initializations():

    # Start the image processing task(s)
    
    # initialize vehicle state
    vehState.mode.setMode (raceModes.NONE)   
    
    time.sleep(0.5) 
    printOut("INITIALIZATIONS: initializations complete")       
    
# end initializations   

############################################################################## 
# This is the main loop of the system.  It loops forever (or until the state
# becomes Modes.Terminate)
##############################################################################

def mainLoop():
    loopCntr    = 0
    printOut ("MAIN_LOOP: starting loop")
    
    while (vehState.mode.currMode != raceModes.TERMINATE and loopCntr < 200):  
        time.sleep (1)          # dag remove    
        
        loopCntr += 1         
        if loopCntr % 1 == 0:
            printOut (("\nMAIN_LOOP: Loop #%2d" % (loopCntr)))       
    
        # Check if we received a command from the GUI host and
        # send a telemetry packet
        guiIf.get_cmd ()         
        guiIf.send_rpiTlm (vehState, rangeLeftPair, rangeRightPair)  
        
        # Get all the telemetry msgs and parse into state structure
        iopMsg = get_iopTlm ()
        if (len(iopMsg) != 0):
            print ("MAIN_LOOP - 4.5")               
            guiIf.send_iopTlm (iopMsg)
        print ("MAIN_LOOP - 5")

        
        #visMsg = getVisionTelemetry()
        #if (length(visMsg) != 0):
        #    guiIf.send_visTlm (visMsg)  
            
        # Now do all the state specific actions
        #stateMachine (vehState, serialPort)
        
        # Let the iop know we're alive
        vehState.currHeartBeat += 1        
        serialPort.sendCommand ('H', vehState.currHeartBeat, 0, 0)
        
        if loopCntr % 3 == 0:
            serialPort.sendCommand ('N', 0, 0, 0 )        
        
        print ("MAIN_LOOP - 6")        
        
    # end while
    
    printOut ("MAIN_LOOP: Terminating mainLoop, Killing serialPort")
    
    # Kill the simulator by sending an unknown command
    #serialPort.sendCommand ('Z', 0, 0, 0) 
    serialPort.killThread()       
    guiIf.close ()          # Close the gui interface TCP server thread
    
# end def 
    
################################################################################
# get_iopTlm( )
################################################################################
def get_iopTlm():
    global IopTlmQueue
    time = 0
    msg = ""
    
    # Get the last message put onto the queue
    tlm_cnt = 0
    # Keep looping until we get the last message put onto the queue  
    #print ("MAINLOOP:GET_IOPTLM - 1")    
    while (not IopTlmQueue.empty()):
        msg = IopTlmQueue.get_nowait()
        #printOut("MAINLOOP:GET_IOPTLM - msg length (%d)" % ( len(msg)) )
        #printOut("MAINLOOP:GET_IOPTLM - got msg (%s)" % ( msg) )
        time = proc_iopTlm(msg)        
        tlm_cnt += 1
    # end while
    #print ("MAINLOOP:GET_IOPTLM - 2")  
    
    if (tlm_cnt > 0):
        print ("MAINLOOP:GET_IOPTLM - received %d new pkts , time = %d" % (tlm_cnt, time))
    else:
        #print ("MAINLOOP:GET_IOPTLM - no new telemetry ")
        #print ("MAINLOOP:GET_IOPTLM - 3")          
        pass
        
    #print ("MAINLOOP:GET_IOPTLM - 4, end")      
    return (msg)        # We'll return the last message on queue
# end    
   
################################################################################
# proc_iopTlm - process_telemetry
################################################################################

def proc_iopTlm (data):        
        try:
            telemArray  = struct.unpack('<LLhhhhhhhhhhhhhhhhhhhhhh', data)
        except:      
            print ("MAINLOOP:PROC_IOPTLM - ERROR unable to parse telemetry")
            print ("MAINLOOP:PROC_IOPTLM - Length of data is ", len(data) )             
            return 0
        
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
        measScanSensor          = telemArray[14]
        measScanDist            = telemArray[15]

        vehState.iopBattVolt1   = telemArray[16]
        vehState.iopBattVolt2   = telemArray[17]
        vehState.iopAccelVert   = telemArray[18]
        vehState.iopGyroHoriz   = telemArray[19]
        vehState.iopCompAngle   = telemArray[20]
        vehState.iopCameraAngle = telemArray[21]        
        vehState.iopSpare2      = telemArray[22]
        vehState.iopSpare3      = telemArray[23]   
        
        if  True:
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
        #######################################################################  
        # Enter the scanner info into the iopRanges buffer
        vehState.iopRanges.enterRange(time   = vehState.iopTime, 
                                      angle  = measScanAngle, 
                                      ranger = measScanSensor, 
                                      range  = measScanDist) 
        #print ("MAINLOOP:PROC_IOPTLM - 2")                                      
        #######################################################################                                      
        # Enter the scanner info into the occupGrid - DAG should we 
        # enter the data if we are using the short range sensor?
        occGrid.enterRange ( carCumDist   = vehState.iopCumDistance, 
                             carCurrAngle = vehState.iopCompAngle, 
                             scanDist     = measScanDist, 
                             scanAngle    = measScanAngle)       
        #print ("MAINLOOP:PROC_IOPTLM - 3")                             
        #######################################################################        
        # Enter the side IR sensor data into the two rangeSensorPairs
        if not WINDOWS:
			rangeLeftPair.newMeasurement (measFrontRange = irLF_Range, 
                                      measRearRange  = irLR_Range)                                     
			rangeRightPair.newMeasurement(measFrontRange = irRF_Range, 
                                      measRearRange  = irRR_Range)   
        # end WINDOWS     
        # print ("MAINLOOP:PROC_IOPTLM - 4 end")        
        return vehState.iopTime
# end

################################################################################
# get_visTlm() - telemetry from the vision system
################################################################################
def get_visTlm():
    global visionTlmQueue
    
    new_data = False
    # Keep looping until we get the last message put onto the queue    
    while (not visionTlmQueue.empty()):
        msg = visionTlmQueue.get_nowait()
        time = proc_visTlm(msg)
        new_data = True
    # end while

    if new_data:
        print "GET VISION - received new telemetry"
    else:
        pass #print "GET VISION - no new telemetry "      

    return (msg)        # We'll return the last message on queue        
# end   

################################################################################
# proc_visTlm - process vision telemetry
################################################################################
def proc_visTlm (data): 
    pass
#end    
 
################################################################################
# visionSend()
################################################################################

def visionSend(obstacle):
    global visionCmdQueue  

    print "VISION SEND - sending value ", chr(obstacle)
       
# end   

###############################################################################
# TESTING
###############################################################################
if __name__ == "__main__":
    ##### TEST # 1 
    initializations()
    mainLoop()    
# end    
