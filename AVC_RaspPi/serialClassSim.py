"""
 serialClassSim.py - All the serial port support routines simulated.  
 The serial port(actually USB) is used to send commands and receive 
 telemetry from the IOP (teensie).
 
 Written by David Gutow 4/2018
"""

import pdb

import time
import threading
import struct
#import serial
import array
from Queue           import Queue
from simulator       import commandQueue
from simulator       import usbQueue
from constants       import *           # Vehicle and course constants
from printOut        import *
  
USB_Port =  '/dev/ttyACM0'
 
###############################################################################
# class serialClass - serial port support
###############################################################################

class serialClass (object):

    ###########################################################################
    # class simulated telemetry values
    ###########################################################################  
    iopTime        = 1000   # current time (milliseconds)
    iopMode        = 0
    iopAcceptCnt   = 0      # number of accepted commands
    iopBistStatus  = 0
    iopSpeed       = 0      # curr speed (cm/sec)
    iopSteerAngle  = 0      # steering angle (0.1 degrees)       
    iopCumDistance = 0      # distance traveled so far (cm)
    
    # Enter the side IR sensors into the two rangeSensorPairs
    irLF_Range     = 0      # sensor 0 distance (cm)
    irLR_Range     = 0        
    irRF_Range     = 0
    irRR_Range     = 0
    
    iopSwitchStatus= 0  
    
    measScanAngle  = 0      # curr angle of scanner (0.01 degrees)
    measScanSensor = 0      # Spare
    measScanDist   = 0      # spare

    iopBattVolt1   = 0
    iopBattVolt2   = 0
    iopAccelVert   = 0
    iopGyroHoriz   = 0
    iopCompAngle   = 0      # curr compass angle (0.1 degrees)
    iopCameraAngle = 0        
    iopSpare2      = 0
    iopSpare3      = 0     

    moveDistance   = 0.0    # (float) distance left to move from last move 
                            # command (cm)
    totalDistance  = 0.0    # total distance traveled so far (float equiv to 
                            # iopCumDistance) (cm)

    ########################################################################### 
    # __init__
    ###########################################################################
    def __init__(self, telemQueue):
    
        self.telemQueue = telemQueue
        self.serialThreadFlag  = True 
        self.serialThreadRunning = False
        
        # start the telemetry thread and give it some time to start up
        try:
            serThread = threading.Thread (
                group=None, target=self.serialPortThread, name = "serialPortThread" )
            serThread.start()
            time.sleep (0.2)  
        except:
            printOut ("SERIALCLASS THREAD: Error starting serialPortThread.")
    # end __init__
    
    ########################################################################### 
    # SerialPortThread(state)
    ###########################################################################
    def serialPortThread(self):
        printOut ("SERIALCLASS THREAD SIM: starting loop")
        self.serialThreadRunning = True
        lastIopMode = self.iopMode          # So we know the mode just switched
        timeInc     = 50                    # time increment each loop (msec)
        counter     = 0
    
        while (self.serialThreadFlag):
            time.sleep (0.4)        

            # Do some simple vehicle simulations:
            
            # If we just switched to race mode wait a simulated second and 
            # then flag the start button was pushed
            if (lastIopMode == 0 and self.iopMode == 1):
                counter += timeInc
                if (counter >= 1000):       # Waited a simulated 1 second?
                    self.iopSwitchStatus = 0xfff
                    lastIopMode = self.iopMode
                    printOut ("SERIALSIM - sending START SWITCH PUSHED")                      
                # end
            #end
            
            # if in race mode then simulate a move and turn 
            if (self.iopMode == 1): 
                try:
                    # We're moving so do some maintenance
                    if (self.moveDistance > 0.01):   
                        deltaDistance = float(self.iopSpeed * timeInc)/1000.0
                        self.moveDistance -= deltaDistance
                        if (self.moveDistance < 0.01):
                            self.moveDistance = 0.0
                        self.totalDistance += deltaDistance
                        self.iopCumDistance += int(self.totalDistance)
                    # end if moveDistance
                except:
                    printOut ("SERIALCLASS THREAD: Unable to process iopMode==1")                    
            # Stopped here dag - any other simulated stuff - the scanner?


            
            #printOut ("SERIALCLASS THREAD: looping...")
            self.iopTime += timeInc
            try:
                self.send_telemetry ()
            except:
                printOut ("SERIALCLASS THREAD: Error trying to send telemetry.")
        # end while
        
        printOut ("SERIALCLASS THREAD: terminating")
        self.serialThreadRunning = False       
    #end serialPortThread    
    
    ###########################################################################
    # sendCommand (command, param1, param2, param3)   
    ###########################################################################
    def sendCommand (self, cmdChar, p1, p2, p3): 
        #printOut ("SERIALPORTSIM:sendCommand - Received %s, %d, %d, %d" % 
        #                                        (cmdChar, p1, p2, p3) )                                
        self.processCmd (cmdChar, p1, p2, p3)          
    #end  
    
    ###########################################################################
    # killThread - stops the serial port thread  
    ########################################################################### 
    def killThread (self):
        self.serialThreadFlag = False
        printOut ("SERIAL PORT THREAD: setting serialThreadFlag false")
        while (self.serialThreadRunning == True):
            pass 
        printOut ("SERIAL PORT THREAD: serialThreadRunning went false")            
    # end
    
    #############################################################################
    # pack_telemetry
    #############################################################################
    def send_telemetry (self):            
        data  = struct.pack('<LLhhhhhhhhhhhhhhhhhhhhhh', 
                                    0x33333333,
                                    self.iopTime,
                                    self.iopMode,
                                    self.iopAcceptCnt,
                                    self.iopBistStatus,
                                    self.iopSpeed,
                                    self.iopSteerAngle,        
                                    self.iopCumDistance,
         
                                    self.irLF_Range,
                                    self.irLR_Range,        
                                    self.irRF_Range,
                                    self.irRR_Range,
                                    
                                    int(self.iopSwitchStatus),
                                    
                                    self.measScanAngle,
                                    self.measScanSensor,
                                    self.measScanDist,
                            
                                    self.iopBattVolt1,
                                    self.iopBattVolt2,
                                    self.iopAccelVert,
                                    self.iopGyroHoriz,
                                    self.iopCompAngle,
                                    self.iopCameraAngle,        
                                    self.iopSpare2,
                                    self.iopSpare3)                                     
        self.telemQueue.put_nowait(data)
        #printOut ("SERIALPORTSIM:send_telemetry - sent")
    # end pack_telemetry                                                                                                              
                                                        
    #############################################################################
    # processCmd - execute the command....
    #############################################################################
    def processCmd (self, cmdChar, p1, p2, p3):
        #printOut ("SERIALPORTSIM:procCommand - Received %s, %d, %d, %d" % (cmdChar, p1, p2, p3) ) 
                                                     
        if cmdChar == 'A':              # set scan angles
            self.iopAcceptCnt += 1  
             
        elif cmdChar == 'C':            # set which scan sensor 
            self.measScanSensor = p1
            self.iopAcceptCnt += 1            
         
        elif cmdChar == 'D':            # Set Mode
            self.iopMode = p1
            if self.iopMode == 2:       # Commanded into estop?
                self.iopBistStatus = 1
            self.iopAcceptCnt += 1       
            printOut ("SERIALSIM - received MODE command - %d" % (p1))             

        elif cmdChar == 'E':            # Estop
            self.iopMode = 2   
            self.iopBistStatus = 1      
            self.iopAcceptCnt += 1 
            printOut ("SERIALSIM - received MODE command - %d" % (p1))             
            
        elif cmdChar == 'H':            # heartbeat
            pass                 
            
        elif cmdChar == 'L':            # lightscene
            self.iopAcceptCnt += 1            
            
        elif cmdChar == 'M':            # move command
            if self.iopMode == 1:       # Must be in normal mode
                self.iopSpeed    = p1
                self.moveDistance     = float(p2)
                self.iopAcceptCnt += 1
                printOut ("SERIALSIM - received MOVE command, speed %d" % (p1))                 
            else:
                printOut ("SERIALSIM - received MOVE command.  ERROR not in normal mode") 
            # end if
            
        elif cmdChar == 'N':            # NOP
            self.iopAcceptCnt += 1              
              
        elif cmdChar == 'P':            # PIDs
            self.iopAcceptCnt += 1            
            
        elif cmdChar == 'Q':            # PIDs
            self.iopAcceptCnt += 1
            
        elif cmdChar == 'S':            # scan speed
            self.iopAcceptCnt += 1
                 
        elif cmdChar == 'T':            # Turn command
            if self.iopMode == 1:       # Must be in normal mode
                self.iopSteerAngle =  p1
                self.iopAcceptCnt += 1
                printOut ("SERIALSIM - received TURN command")   
            else:
                printOut ("SERIALSIM - received TURN command.  ERROR not in normal mode")
            # end if
            
        elif cmdChar == 'V':            # set camera angle
            self.iopCameraAngle = p1
            self.iopAcceptCnt += 1
 
        elif cmdChar == 'Z':            # shutdown
            simulatorFlag = False       # Kill the Simulator Thread
            self.iopAcceptCnt += 1 
            printOut ("SERIALSIM - received SHUTDOWN command")            

        else:                           # unrecognized command
            pass        
        # end if       
            
    # end processCommand    

# End class           

    
###############################################################################
    
if __name__ == "__main__":

    #pdb.set_trace()
    
    telemetryQueue  = Queue(10)
    
    sc = serialClass(telemetryQueue)  
    tlm_cnt = 0
    
    for i in range(0, 25):
        if (i == 1): 
            sc.sendCommand ("A", 1, 2, 2)
        if (i == 2):
            sc.sendCommand ("C", 56, 256, 0)
        if (i == 3):
           sc.sendCommand  ("D", 1, 0, 0) 
        if (i == 4):  
            sc.sendCommand ("E", 2, 2, 2)  
        if (i == 5):  
            sc.sendCommand ("H", 2, 2, 2)             
        if (i == 6):  
            sc.sendCommand ("L", 2, 2, 2)             
        if (i == 7):  
            sc.sendCommand ("M", 2, 2, 2)             
        if (i == 8):  
            sc.sendCommand ("N", 2, 2, 2)             
        if (i == 9):  
            sc.sendCommand ("P", 2, 2, 2)             
        if (i == 10):  
            sc.sendCommand ("Q", 2, 2, 2)      
        if (i == 11):  
            sc.sendCommand ("S", 2, 2, 2) 
        if (i == 12):  
            sc.sendCommand ("T", 2, 2, 2) 
        if (i == 13):  
            sc.sendCommand ("V", 2, 2, 2) 
        if (i == 14):  
            sc.sendCommand ("Z", 2, 2, 2) 
        if (i == 15):  
            sc.sendCommand ("H", 2, 2, 2) 
        if (i == 16):  
            sc.sendCommand ("B", 2, 2, 2) 

        tlm_cnt = 0
        while (not telemetryQueue.empty()):
            msg = telemetryQueue.get_nowait()       
            tlm_cnt += 1
        # end while  
        if (tlm_cnt > 0):
            printOut ("GET TELEMETRY - received %d new pkts , i = %d" % (tlm_cnt, i))            
                  
        time.sleep (0.5)                                    
    # end
    
    sc.killThread()
    
# end


    
    
    
    
    
    
