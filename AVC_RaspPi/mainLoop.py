#!/usr/bin/python

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
from OccupGrid_Rich  import Grid
from guiInterface    import guiIfClass
from printOut        import *
from lidar_dag       import *


if SIM_TEENSY:
    from serialClassSim  import serialClass
else:    
    from serialClass     import serialClass
# end if SIM_TEENSY    

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
guiIf = guiIfClass (RPI_IPADDR, RPI_TCPPORT, UDP_IPADDR, UDP_IOPPORT, UDP_VISPORT)

# Number of accepted commands from the GUI
guiAcceptCnt = 0

############################################################################### 
# Initialize the entire system
###############################################################################

def initializations():

    # Start the image processing task(s)
    
    # initialize vehicle state
    vehState.mode.setMode (raceModes.NONE)   
    
    # start and initialize the RPLidar
    init_lidar_scan()
    
    time.sleep(0.5) 
    printOut("INITIALIZATIONS: initializations complete")       
    
# end initializations   

############################################################################## 
# This is the main loop of the system.  It loops forever (or until the state
# becomes Modes.Terminate)
##############################################################################

def mainLoop():
    abort = False
    loopCntr    = 0
    printOut ("MAIN_LOOP: Dwelling for 2 seconds...")
    time.sleep (2.0)
    last_time = time.clock()
    
    while (not abort): 
        #(vehState.mode.currMode != raceModes.TERMINATE and not abort): 
        # Wait until 0.1 seconds have gone by from the last loop
        while ( time.clock() < (last_time + 0.1) ):
            pass
        last_time = time.clock()
                       
        if loopCntr % 20 == 0:
            printOut ("\nMAIN_LOOP: Loop #%2d, time %f" % (loopCntr, time.clock()) )      
    
        # Check if we received a command from the GUI host and
        # send the GUI a telemetry packet
        guiCmd = guiIf.get_cmd () 
        abort = exec_guiCmd (guiCmd)           
        guiIf.send_rpiTlm (guiAcceptCnt, vehState, rangeLeftPair, rangeRightPair)  
        
        # Get the iop telemetry msgs and parse into state structure
        iopMsg = get_iopTlm (loopCntr)
        #if (len(iopMsg) != 0):              
        #    guiIf.send_iopTlm (iopMsg) done in get_iopTlm now

        # Get the vision temetry msgs and parse into state structure        
        #visMsg = getVisionTelemetry()
        #if (length(visMsg) != 0):
        #    guiIf.send_visTlm (visMsg)  
        
        # Get all the RPlidar data and enter it into the occGrid
        get_lidarTlm(loopCntr)
            
        # Now do all the state specific actions
        try:
            stateMachine (vehState, serialPort, occGrid)
        except:
            print ("MAIN_LOOP - ERROR in stateMachine")
        
        # Let the iop know we're alive
        vehState.currHeartBeat += 1        
        #serialPort.sendCommand ('H', vehState.currHeartBeat, 0, 0)   dag turn on  
        
        loopCntr += 1              
    # end while
    
    printOut ("MAIN_LOOP: Terminating mainLoop, killing serialPort")
    serialPort.killThread()     
      
    # Kill the simulator by sending an unknown command
    #serialPort.sendCommand ('Z', 0, 0, 0) 

    printOut ("MAIN_LOOP: serialPort killed, killing guiIf")   
    guiIf.close ()          # Close the gui interface TCP server thread
    printOut ("MAIN_LOOP: guiIf killed, returning...")       
# end def 
    
################################################################################
# get_lidarTlm(loopCntr)
################################################################################
def get_lidarTlm(loopCntr):
    # Get the lastest range points from the RPLidar
    scan_list = get_lidar_data()
    
    # Enter each of the range points into the occGrid
    for dataPt in scan_list:
        newPt = dataPt[0]
        qual  = dataPt[1]
        angle = dataPt[2]
        dist  = dataPt[3]  
        occGrid.enterRange(vehState.iopCumDistance, vehState.iopSteerAngle, 
                           dist/10, angle)                              

    # Now shift the occGrid down by the vehicles motion since the last time
    occGrid.recenterGrid(vehState.iopCumDistance, vehState.iopSteerAngle);
    
    # Send the occGrid as telemetry to our GUI
    # occGrid.sendUDP()
    
    if (loopCntr == 0):
        # Initialize the graphic window
        occGrid.initGraphGrid("Occupancy Grid", 4, True, False);
    if loopCntr % 10 == 0:
        # every seconds update the graph
        occGrid.graphGrid ();
# End
        

################################################################################
# get_iopTlm( )
################################################################################
def get_iopTlm(loopCntr):
    global IopTlmQueue
    time = 0
    msg = ""
    
    # Get the last message put onto the queue
    tlm_cnt = 0
    # Keep looping until we get the last message put onto the queue    
    while (not IopTlmQueue.empty()):
        msg = IopTlmQueue.get_nowait()
        #printOut("MAINLOOP:GET_IOPTLM - msg length (%d)" % ( len(msg)) )
        #printOut("MAINLOOP:GET_IOPTLM - got msg (%s)" % ( msg) )
        time = proc_iopTlm(msg)                  
        guiIf.send_iopTlm (msg)         # dag - send every pkt to gui
    
        tlm_cnt += 1
    # end while

    
    if (tlm_cnt > 0 and (loopCntr % 10 == 0) ):
        print "MAINLOOP:GET_IOPTLM - nPkts %d, Tim %3d, Mode %1d, Accept %2d, But %2d/%2d" % (
            tlm_cnt, vehState.iopTime, vehState.iopMode, vehState.iopAcceptCnt, 
            vehState.iopSwitchStatus, vehState.iopStartSwitch) 
    else:
        #print ("MAINLOOP:GET_IOPTLM - no new telemetry ")         
        pass
              
    return (msg)        # We'll return the last message on queue
# end    
   
################################################################################
# proc_iopTlm - process_telemetry
################################################################################
def proc_iopTlm (data):      
    global vehState  
    try:
        telemArray  = struct.unpack('<LLhhhhhhhhhhhhhhhhhhhhhhh', data)
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
    measRejCnt              = telemArray[14]
    measRejReason           = telemArray[15]

    vehState.iopBattVolt1   = telemArray[16]
    vehState.iopBattVolt2   = telemArray[17]
    vehState.iopAccelVert   = telemArray[18]
    vehState.iopGyroHoriz   = telemArray[19]
    vehState.iopCompAngle   = telemArray[20]
    vehState.iopCameraAngle = telemArray[21]   
    vehState.iopBrakeStatus = telemArray[22]    
    vehState.iopSpare2      = telemArray[23]
    vehState.iopSpare3      = telemArray[24]   
    
    if  False:
        print "MAINLOOP:PROC_IOPTLM - Pkid %d, Time %3d, Mode %1d, AccCnt %2d, Spd %3d, Switch %2d/%2d" % (
            telemArray[0],  telemArray[1],  telemArray[2],   telemArray[3],  telemArray[5], 
            telemArray[12] , telemArray[12] & 0x01)   
        #print "PROCESS_TELEM - Bist %d, Speed %d, SteerAng %d, Distance  %d" % (
            # vehState.iopBistStatus, vehState.iopSpeed, vehState.iopSteerAngle, 
            # vehState.iopCumDistance)                  
    #end

    if False:
        hexArr = [hex(ord(val)) for val in data]
        print ("PROC_IOPTELEM -----> %s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s" % 
            (hexArr[0], hexArr[1],  hexArr[2],   hexArr[3], 
             hexArr[4], hexArr[5],  hexArr[6],   hexArr[7], 
             hexArr[8], hexArr[9] , hexArr[10], hexArr[11] ,
             hexArr[12], hexArr[13] , hexArr[14], hexArr[15] ) )
    # end
    
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
                                  ranger = 1, 
                                  range  = 1) 
    #print ("MAINLOOP:PROC_IOPTLM - 2")                                      
    #######################################################################                                      
    # Enter the scanner info into the occupGrid - DAG should we 
    # enter the data if we are using the short range sensor?
    occGrid.enterRange ( carCumDist   = vehState.iopCumDistance, 
                         carCurrAngle = vehState.iopCompAngle, 
                         scanDist     = 1, 
                         scanAngle    = 1)       
    #print ("MAINLOOP:PROC_IOPTLM - 3")                             
    #######################################################################        
    # Enter the side IR sensor data into the two rangeSensorPairs
    if not SIM_TEENSY:
        #rangeLeftPair.newMeasurement (measFrontRange = irLF_Range, 
                                    #measRearRange  = irLR_Range)                                     
        #rangeRightPair.newMeasurement(measFrontRange = irRF_Range, 
                                    #measRearRange  = irRR_Range)   
        pass
    # end SIM_TEENSY     
    
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

###########################################################################
# exec_guiCmd  -  
###########################################################################    
def exec_guiCmd (cmdMsg):
    global guiAcceptCnt
    abort = False
    
    if (len(cmdMsg) == 0):      # Is there a real gui command here?
        return
    
    try:
        #print ("EXEC_GUICMD PARSE: Length of data is ", len(cmdMsg))
        cmdArray  = struct.unpack('<hhhh', cmdMsg)
        command   = chr(cmdArray[0])
        param1    = cmdArray[1]
        param2    = cmdArray[2]
        param3    = cmdArray[3]     
    except:
        print ("EXEC_GUICMD: Parse Error - unable to parse command") 
        return

    printOut ("EXEC_GUICMD - Cmd %s, P1 %d, P2 %d, P3 %d" % 
              (command, param1, param2, param3) )  
    
    if   (command == 'A'):      # Set scanner angles  
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1
        
    elif (command == 'B'):      # Set brake on/off 
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1        
        
    elif (command == 'C'):      # Set scanner sensor   
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1
        
    elif (command == 'D'):      # Set IOP mode    
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1
        
    elif (command == 'E'):      # E-stop
        serialPort.sendCommand (command, param1, param2, param3)
        serialPort.sendCommand (command, param1, param2, param3)        
        guiAcceptCnt += 1
        
    elif (command == 'F'):      # Set accelerations
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1
        
    elif (command == 'G'):      # not defined        
        bad_cmd (command, param1, param2, param3)   
        
    elif (command == 'H'):      # not defined 
        bad_cmd (command, param1, param2, param3)  
        
    elif (command == 'I'):      # not defined 
        bad_cmd (command, param1, param2, param3)  
        
    elif (command == 'J'):      # Send NOP to vision proc 
        guiAcceptCnt += 1
        
    elif (command == 'K'):      # not defined 
        bad_cmd (command, param1, param2, param3)  
        
    elif (command == 'L'):      # Set lighting scene
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1      
        
    elif (command == 'M'):      # Move       
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1
        
    elif (command == 'N'):      # Send NOP to IOP
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1    
        
    elif (command == 'O'):      # NOP to the Rpi        
        guiAcceptCnt += 1         
        
    elif (command == 'P'):      # Set speed PIDS        
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1  
        
    elif (command == 'Q'):      # Set turn PIDS        
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1       
        
    elif (command == 'R'):      # Set Rpi mode
        pass
        
    elif (command == 'S'):      # Set scanner speed            
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1   
        
    elif (command == 'T'):      # Turn            
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1      
        
    elif (command == 'U'):      # not defined              
        bad_cmd (command, param1, param2, param3)  
        
    elif (command == 'V'):      # Set scanner angles             
        serialPort.sendCommand (command, param1, param2, param3)
        guiAcceptCnt += 1        
        
    elif (command == 'W'):      # Write parameters to file 
        bad_cmd (command, param1, param2, param3)    
        
    elif (command == 'X'):      # Set speed
        bad_cmd (command, param1, param2, param3)  
        
    elif (command == 'Y'):      # Set vision mode
        guiAcceptCnt += 1   
        
    elif (command == 'Z'):      # Load parameters from file
        pass  
        
    elif (command == '1'):      # Abort the Python Program!  
        abort = True
        
    elif (command == '2'):      # n/d
        bad_cmd (command, param1, param2, param3) 
        
    elif (command == '3'):      # n/d 
        bad_cmd (command, param1, param2, param3)
        
    elif (command == '4'):      # n/d        
        bad_cmd (command, param1, param2, param3)
        
    elif (command == '5'):      # n/d        
        bad_cmd (command, param1, param2, param3)
        
    elif (command == '6'):      # n/d        
        bad_cmd (command, param1, param2, param3)
        
    elif (command == '7'):      # n/d        
        bad_cmd (command, param1, param2, param3)  
        
    elif (command == '8'):      # n/d        
        bad_cmd (command, param1, param2, param3) 
        
    elif (command == '9'):      # n/d        
        bad_cmd (command, param1, param2, param3)
        
    print ("EXEC_GUICMD - guiAcceptCnt %d\n" % ( guiAcceptCnt))  
    return (abort)
# end exec_cmd

###########################################################################
# bad_cmd  -  Output diagnostics in case of a unknown command
###########################################################################     
def bad_cmd (cmd, p1, p2, p3):
    printOut ("GUIINTERFACE:BAD_CMD - Cmd %s, P1 %d, P2 %d, P3 %d" % 
              (cmd, p1, p2, p3) )  
#end bad_cmd

###############################################################################
# MAIN-LOOP EXECUTION
###############################################################################
if __name__ == "__main__":
    ##### TEST # 1 
    initializations()
    mainLoop()    
# end    
