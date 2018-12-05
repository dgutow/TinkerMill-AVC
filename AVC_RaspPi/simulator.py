#
"""
 simulator.py 
 A simple simulator for the AVC vehicle.  Used to test and debug
 the vehicle command & control code
 
 Written by David Gutow 9/2017
"""

from queue           import Queue
import threading
import struct
import time

#from mainLoop   import telemetryQueue

# the internal states of the vehicle simulator
simCurrTime     = 0
simCurrMode     = 0     # 0 - BIST, 1 - Normal, 2 - Estop
simAccCntr      = 0
simBist         = 0
simCurrSpeed    = 0
simCurrSteering = 0
simCumDist      = 0
simIrLF         = 250
simIrLR         = 200
simIrRF         = 200
simIrRR         = 250
simSwitches     = 0
simSensorAngle  = 0
simSensor       = 0
simSensorDist   = 0
simVolt1        = 0
simVolt2        = 0
simAccel        = 0
simGyro         = 0
simCompass      = 0
simCamAngle     = 0
simbrakeStatus  = 0
simSpare        = 0

simEstopReason  = 0     # 0-none 1-commanded 2-estop button 3-bump 4-heartbeat
simTimeLastHB   = 0     # Time since last heartbeat

simulatorFlag   = True  # Simulator Thread will die when this is false
    
# Command Queue is used to pass command packets to the simulator
commandQueue    = Queue(20)

# USB Queue used to mimic the USB communication line for telemetry
usbQueue        = Queue(50)

################################################################################
# Start the vehicle simulator thread
################################################################################
def startVehSimulator():
    sim = threading.Thread(group=None, target=vehSimulator, name = "Vehicle Simulator" )
    simulatorFlag   = True          # Thread will die when this is false
    sim.start()
# end

################################################################################
# The vehicle simulator thread
################################################################################
def vehSimulator():
    global simCurrTime
    global simCurrMode
    global simAccCntr
    global simBist
    global simCurrSpeed
    global simCurrSteering
    global simCumDist
    global simIrLF
    global simIrLR
    global simIrRF
    global simIrRR
    global simSwitches
    global simSensorAngle
    global simSensor
    global simSensorDist
    global simVolt1
    global simVolt2
    global simAccel
    global simGyro
    global simCompass
    global simCamAngle
    global simEstopReason
    global simTimeLastHB
    global simulatorFlag
    global commandQueue
    global usbQueue

    print ("------------> VEHICLE SIMULATOR - starting loop")   
    cmdCnt      = 0
    loopCnt     = 0
    sleepTime   = 100     # set simulator period (millisecs)
    
    while (simulatorFlag and loopCnt < 600):
        # update the state of the simulator
        vehUpdateState(sleepTime)    
    
        # Check to see if we have any new commands
        while (not commandQueue.empty()):
            cmd = commandQueue.get_nowait()
            processCommand(cmd)        
            cmdCnt += 1
        # end while
    
        # Send out telemetry packet but not at first
        if simCurrTime > 1500:
            telemArray = createTelemetry()
            if usbQueue.qsize() < 40:
                usbQueue.put_nowait(telemArray)   
        
        time.sleep (sleepTime/1000.0)   #Goodnight  
        loopCnt += 1
        
        if ((loopCnt % 50) == 0):   
            print ("------------> VEHICLE SIMULATOR - looping ", loopCnt)
    # end while
    
    print ("------------> VEHICLE SIMULATOR - terminating")
# end    

################################################################################
# processCommand
################################################################################
def processCommand (cmd):
    global simCurrTime
    global simCurrMode
    global simAccCntr
    global simBist
    global simCurrSpeed
    global simCurrSteering
    global simCumDist
    global simIrLF
    global simIrLR
    global simIrRF
    global simIrRR
    global simSwitches
    global simSensorAngle
    global simSensor
    global simSensorDist
    global simVolt1
    global simVolt2
    global simAccel
    global simGyro
    global simCompass
    global simCamAngle
    global simEstopReason
    global simTimeLastHB
    global simulatorFlag
    global commandQueue
    global usbQueue
    
    # Grab the command and parse
    cmdFields = struct.unpack('<hhhh', cmd)
    command = chr(cmdFields[0] >> 8)
    print ("------------> VEHICLE SIMULATOR - received cmd ", command) 

    simAccCntr += 1             # inc the accept counter
    
    # Execute the command....
    if command == 'M':          # move command
        if simCurrMode == 1:    # Must be in normal mode
            simCurrSpeed    = cmdFields[1]
            cmddist         = cmdFields[2]
        else:
            simAccCntr -= 1     # else don't inc acc counter 
        # end        
    elif command == 'T':        # Turn command
        if simCurrMode == 1:     # Must be in normal mode
            simCurrSteering =  cmdFields[1]
        else:
            simAccCntr -= 1     # else don't inc acc counter 
        # end          
    elif command == 'E':        # Estop
        simCurrMode = 2   
        simEstopReason = 1          
    elif command == 'H':        # heartbeat
        simAccCntr -= 1         # don't inc acc counter on heartbeat
        simTimeLastHB = simCurrTime    
    elif command == 'L':        # lightscene
        pass
    elif command == 'P':        # PIDs
        pass
    elif command == 'Q':        # PIDs
        pass
    elif command == 'D':        # Set Mode
        simCurrMode = cmdFields[1]
        if simCurrMode == 2:      # Commanded into estop?
            simEstopReason = 1
        # end   
    elif command == 'N':        # NOP
        simAccCntr -= 1  
    elif command == 'S':        # scan speed
        pass
    elif command == 'A':        # scan angles
        pass        
    elif command == 'V':        # set camera angle
        simCamAngle = cmdFields[1]
    elif command == 'C':        # set which scan sensor
        simSensor = cmdFields[1]       
    else:                       # unrecognized command
        simAccCntr   -= 1       # don't inc acc counter on unrecog command   
        simulatorFlag = False   # Kill the Simulator Thread        
    # end if
  
# end 

################################################################################
# vehUpdateState - Update all our dynamic state variables...
################################################################################
def vehUpdateState(sleepTime): 
    global simCurrTime
    global simCurrMode
    global simAccCntr
    global simBist
    global simCurrSpeed
    global simCurrSteering
    global simCumDist
    global simIrLF
    global simIrLR
    global simIrRF
    global simIrRR
    global simSwitches
    global simSensorAngle
    global simSensor
    global simSensorDist
    global simVolt1
    global simVolt2
    global simAccel
    global simGyro
    global simCompass
    global simCamAngle
    global simEstopReason
    global simTimeLastHB
    global simulatorFlag
    global commandQueue
    global usbQueue
    
    simCurrTime += sleepTime  
    simCumDist += 1 #simCurrSpeed * sleepTime/1000
    
    # Let's go through a scenario
    if simSwitches == 0 and simCurrTime > 5000:     # after 5 seconds start
        print ("------------> VEHICLE SIMULATOR - setting START BUTTON ")
        simSwitches = 1                             # start button pushed
# end 
    
################################################################################
# createTelemetry()   
################################################################################
def packTelemetryPkt():   
    telemArray = []
    
    #telemArray.append(45)      # 0 CurrTime     
    
    telemArray.append(simCurrTime)      # 0 CurrTime 
    telemArray.append(simCurrMode)      # 1 currModemode   
    telemArray.append(simAccCntr)       # 2 naccepts   
    telemArray.append(simBist)          # 3 bist  
    telemArray.append(simCurrSpeed)     # 4 curr speed   
    telemArray.append(simCurrSteering)  # 5 curr steeringangle           
    telemArray.append(simCumDist)       # 6 cumDist   
    telemArray.append(simIrLF)          # 7 irLF   
    telemArray.append(simIrLR)          # 8 irLR           
    telemArray.append(simIrRF)          # 9 irRF  
    telemArray.append(simIrRR)          # 10 irRR  
    telemArray.append(simSwitches)      # 11 switches     
    telemArray.append(simSensorAngle)   # 12 IR angle  
    telemArray.append(simSensor)        # 13 IR use  
    telemArray.append(simSensorDist)    # 14 IR Range  
    telemArray.append(simVolt1)         # 15 batt1  
    telemArray.append(simVolt2)         # 16 batt2 
    telemArray.append(simAccel)         # 17 accel  
    telemArray.append(simGyro)          # 18 rot rate 
    telemArray.append(simCompass)       # 19 compass  
    telemArray.append(simCamAngle)      # 20 camera vertical angle        
    telemArray.append(simbrakeStatus)   # 21 brake           
    telemArray.append(0)                # 22 spare     
    
    # print "CREATETELEM: Length of telemArray is ", simCurrTime)
    # print ("vehSimulator:CREATETELEM - Length of telemArray is ", len(telemArray))
    print ("simCurrTime is ", simCurrTime)
   
    packedArray  = struct.pack('>L22h', *telemArray)
    # packedArray  = struct.pack('>L22h',     simCurrTime,
                                            # simCurrMode,
                                            # simAccCntr,   
                                            # simBist,  
                                            # simCurrSpeed, 
                                            # simCurrSteering,        
                                            # simCumDist,  
                                            # simIrLF,    
                                            # simIrLR,          
                                            # simIrRF,  
                                            # simIrRR, 
                                            # simSwitches,  
                                            # simSensorAngle,
                                            # simSensor, 
                                            # simSensorDist, 
                                            # simVolt1, 
                                            # simVolt2,
                                            # simAccel, 
                                            # simGyro,  
                                            # simCompass,   
                                            # simCamAngle,         
                                            # simbrakeStatus,         
                                            # 0)         
    # print ("vehSimulator:CREATETELEM - Length of packedArray is ", len(packedArray))
    return packedArray
# end

################################################################################
# parse_telemetry
################################################################################

def parse_telemetry (data):
            
        # print ("PARSE: Length of data is ", len(data))
        sys.stdout.flush()
        telemArray  = struct.unpack('<Lhhhhhhhhhhhhhhhhhhhhhh', data)
        
        iopTime        = telemArray[0]
        iopMode        = telemArray[1]
        iopAcceptCnt   = telemArray[2]
        iopBistStatus  = telemArray[3]
        iopSpeed       = telemArray[4]
        iopSteerAngle  = telemArray[5]         
        iopCumDistance = telemArray[6]
        
        # Enter the side IR sensors into the two rangeSensorPairs
        irLF_Range     = telemArray[7]
        irLR_Range     = telemArray[8]        
        irRF_Range     = telemArray[9]
        irRR_Range     = telemArray[10]
        
        iopSwitchStatus  = telemArray[11]  
        
        measScanAngle  = telemArray[12]
        measScanSensor = telemArray[13]
        measScanDist   = telemArray[14]

        iopBattVolt1   = telemArray[15]
        iopBattVolt2   = telemArray[16]
        iopAccelVert   = telemArray[17]
        iopGyroHoriz   = telemArray[18]
        iopCompAngle   = telemArray[19]
        iopCameraAngle = telemArray[20]        
        iopSpare2      = telemArray[21]
        iopSpare3      = telemArray[22]   
        
        if  True:
            print ("PARSE_TELEM - Time %6d, Mode %1d, AcceptCnt %3d, Bist %3d Speed %3d, SteerAngle %3d" % (
             iopTime, iopMode, iopAcceptCnt, iopBistStatus, iopSpeed, iopSteerAngle  )) 
            print ("PARSE_TELEM - cumDistance %3d, LF_Range  %3d, LR_Range %3d RF_Range %3d RR_Range %3d" % (
             iopCumDistance, irLF_Range, irLR_Range, irRF_Range, irRR_Range ))
            print ("PARSE_TELEM - switches %3d, scan angle  %3d, scan sensor %3d scan dist %3d" % (
             iopSwitchStatus, measScanAngle, measScanSensor, measScanDist ))  
            print ("PARSE_TELEM - Batt volt1 %3d, Batt volt2  %3d, accel %3d Gyro %3d" % (
             iopBattVolt1, iopBattVolt2, iopAccelVert, iopGyroHoriz ))  
            print ("PARSE_TELEM - Compass %3d, CameraAngle  %3d, spare 2 %3d spare 3 %3d\n" % (
             iopCompAngle, iopCameraAngle, iopSpare2, iopSpare3 ))  
            sys.stdout.flush() 
                       
        #end  

################################################################################
# print_telemetry
################################################################################   
def print_telemetry():
    print ("PRINT_TELEM - simCurrTime   ", simCurrTime)      # 0 CurrTime 
    print ("PRINT_TELEM - simCurrMode   ", simCurrMode)      # 1 currModemode   
    print ("PRINT_TELEM - simAccCntr    ", simAccCntr)       # 2 naccepts   
    print ("PRINT_TELEM - simBist       ", simBist)          # 3 bist  
    print ("PRINT_TELEM - simCurrSpeed  ", simCurrSpeed)     # 4 curr speed   
    print ("PRINT_TELEM - simCurrSteering", simCurrSteering)  # 5 curr steeringangle           
    print ("PRINT_TELEM - simCumDist    ", simCumDist)       # 6 cumDist   
    print ("PRINT_TELEM - simIrLF       ", simIrLF)          # 7 irLF   
    print ("PRINT_TELEM - simIrLR       ", simIrLR)          # 8 irLR           
    print ("PRINT_TELEM - simIrRF       ", simIrRF)          # 9 irRF  
    print ("PRINT_TELEM - simIrRR       ", simIrRR)          # 10 irRR  
    print ("PRINT_TELEM - simSwitches   ", simSwitches)      # 11 switches     
    print ("PRINT_TELEM - simSensorAngle", simSensorAngle)   # 12 IR angle  
    print ("PRINT_TELEM - simSensor     ", simSensor)        # 13 IR use  
    print ("PRINT_TELEM - simSensorDist ", simSensorDist)    # 14 IR Range  
    print ("PRINT_TELEM - simVolt1      ", simVolt1)         # 15 batt1  
    print ("PRINT_TELEM - simVolt2      ", simVolt2)         # 16 batt2 
    print ("PRINT_TELEM - simAccel      ", simAccel)         # 17 accel  
    print ("PRINT_TELEM - simGyro       ", simGyro)          # 18 rot rate 
    print ("PRINT_TELEM - simCompass    ", simCompass)       # 19 compass  
    print ("PRINT_TELEM - simCamAngle   ", simCamAngle)      # 20 camera vertical angle        
    print ("PRINT_TELEM - simbrakeStatus", simbrakeStatus)   # 21 brake           
    print ("PRINT_TELEM - simSpare      ", simSpare)         # 22 spare     


################################################################################
if __name__ == "__main__":
    startVehSimulator()  
# end 
