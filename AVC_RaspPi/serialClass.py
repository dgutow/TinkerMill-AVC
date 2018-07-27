"""
 serialClass.py - All the serial port support routines.  The serial port
 (actually USB) is used to send commands and receive telemetry from the
 IOP (teensie).
 
 Written by David Gutow 9/2017
"""

import time
import threading
import struct
import serial
import array
from Queue           import Queue
from constants       import *           # Vehicle and course constants
from printOut        import *
  
USB_Port =  '/dev/ttyACM0'
#USB_Port =  '/dev/ttyACM1'
  
###############################################################################
# class serialClass - serial port support
###############################################################################

class serialClass (object):
    serialPort      = serial.Serial(port=USB_Port, baudrate=115200) 
    serialPortFlag  = True      # Use to kill the serial port thread   
        
    ########################################################################### 
    # __init__
    ###########################################################################
    def __init__(self, telemQueue):
    
        self.telemQueue = telemQueue
        self.serialPortFlag  = True 
        
        # start the telemetry thread and give it some time to start up
        serThread = threading.Thread (
            group=None, target=self.serialPortThread, name = "serialPortThread" )
        serThread.start()
        time.sleep (0.2)  
    # end __init__
    
    ########################################################################### 
    # SerialPortThread(state)
    ###########################################################################
    def serialPortThread(self):
        print "SERIAL PORT THREAD: starting loop"  
        runFlag = self.serialPortFlag
        #self.serialPort.reset_input_buffer()
        self.serialPort.flushOutput()    
        self.serialPort.flushInput()    
        cnt = 0
    
        while (runFlag):
            data = self.serialPort.read(54)    # 54 length of a telemetry message
            self.serialPort.flushInput()            
            self.telemQueue.put(data)
            if (cnt %4== 0):
               #print ("serialPortThread: Flag %d, Rcvd data, currCnt = %d" % 	(runFlag, cnt) )
               #print ("serialPortThread MSG: [%s]\n" % 	(data) )              
               pass
            #time.sleep (0.1)                # DAG turn off when we get real port       
            cnt += 1
            runFlag = self.serialPortFlag	# For some reason this works?
        # end while
        printOut ("SERIAL PORT THREAD: terminated") 
    #end def    
    
    ###########################################################################
    # sendCommand (command, param1, param2, param3)   
    ###########################################################################
    def sendCommand (self, commandChar, param1, param2, param3):
        # pack the command into a struct
        packedArray  = struct.pack('<hhhhh', 0x5454, ord(commandChar), param1, param2, param3)
        arr_val = array.array('B', packedArray).tostring()
        # send it along
 
        print ("------------> serialPort:sendCommand - Sending %s, %d, %d, %d\n" % 
								(commandChar, param1, param2, param3) )       
        nbytes = self.serialPort.write(packedArray)
        if nbytes != 10:
        	printOut ("serialPort:sendCommand - ERROR nbytes = %d\n" % (nbytes) ) 
        # end       
    #end  
    
    ###########################################################################
    # killThread - stops the serial port thread  
    ########################################################################### 
    def killThread (self):
        self.serialPortFlag = False
        print "SERIAL PORT THREAD: setting serialPortFlag false" 
    # end
    
# End class    
        
################################################################################
# parse_telemetry
################################################################################

def parse_telemetry (data):            
        # print "PARSE: Length of data is ", len(data)
        telemArray  = struct.unpack('<Lhhhhhhhhhhhhhhhhhhhhhh', data)
          # OBE OBE
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
        measRejCnt = telemArray[13]
        measRejReason   = telemArray[14]
        
        measScanAngle           = telemArray[13]
        measRejCnt              = telemArray[14]
        measRejReason           = telemArray[15]        

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
             iopSwitchStatus, measScanAngle, measRejCnt, measRejReason ))  
            print ("PARSE_TELEM - Batt volt1 %3d, Batt volt2  %3d, accel %3d Gyro %3d" % (
             iopBattVolt1, iopBattVolt2, iopAccelVert, iopGyroHoriz ))  
            print ("PARSE_TELEM - Compass %3d, CameraAngle  %3d, spare 2 %3d spare 3 %3d\n" % (
             iopCompAngle, iopCameraAngle, iopSpare2, iopSpare3 ))   
        #end  
    
###############################################################################
    
if __name__ == "__main__":
    telemetryQueue  = Queue(400)
    usbQueue        = Queue(400)
    
    sc = serialClass(telemetryQueue)  
    tlm_cnt = 0
    
    for i in range(0, 25):
        if (i == 5): 
            sc.sendCommand ("N", 1, 2, 2)
        if (i == 10):
            sc.sendCommand ("T", 56, 256, 0)
        if (i == 15):
           sc.sendCommand  ("M", 1, 0, 0) 
        if (i == 20):  
            sc.sendCommand ("M", 2, 2, 2)  

        tlm_cnt = 0
        while (not telemetryQueue.empty()):
            msg = telemetryQueue.get_nowait()
            parse_telemetry(msg)        
            tlm_cnt += 1
        # end while  
        if (tlm_cnt > 0):
            pass # print ("GET TELEMETRY - received %d new pkts , i = %d" % (tlm_cnt, i))            
                  
        time.sleep (0.9)                                    
    # end
    
    sc.killThread()
    
# end


    
    
    
    
    
    
