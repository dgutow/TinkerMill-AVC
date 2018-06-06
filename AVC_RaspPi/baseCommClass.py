"""
 baseCommClass.py - All the communication (ethernet/wifi) support routines
 for messages to/from the base station.
 
 Written by David Gutow 9/2017
"""

import time
import threading
import struct
import socket         # import *
import sys

from queue           import Queue

# Listen on localhost at port 34563
TCP_IP      = '127.0.0.1' 
TCP_PORT    = 34563
BUFFER_SIZE = 8            # All commands are exactly 8 bytes long

###############################################################################
# baseCommClass - TCP connection to the base station
###############################################################################

class baseCommClass (object):
    #sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    threadFlag  = True              # Use to kill the port thread   
    clientFlag  = False             # Indicates a client connection
        
    ########################################################################### 
    # __init__
    ###########################################################################
    def __init__(self, baseQueue):
        # # Set up the socket & connection
        # self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)       
    
        # hostname = socket.gethostname()
        # print ("BASESTATION: hostname is %s\n" % (hostname))
        # sys.stdout.flush()
        # #self.sock.bind (hostname, TCP_PORT)       
        # self.sock.bind (('', TCP_PORT))          

        # self.sock.listen(0)          # put the socket into listening mode           
        # print ("BASESTATION: Socket created and put into listening mode")     
        # sys.stdout.flush()
                    
        self.baseQueue          = baseQueue
        self.baseStationFlag    = True 
        self.connection         = None
        
        # start the telemetry thread and give it some time to start up
        baseThread = threading.Thread ( group=None, 
            target=self.baseStationThread, name = "baseStationThread" )
        baseThread.start()
        time.sleep (0.2)  
    # end __init__
    
    ########################################################################### 
    # baseStationThread(state)
    ###########################################################################
    def baseStationThread(self):
        print ("BASESTATION THREAD: starting loop")  
        sys.stdout.flush()
        loopCntr  = 0
        sleeptime = 0.5
        
        while (self.threadFlag):
            # Set up the socket & connection
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)            
            self.sock.bind (('', TCP_PORT))          
            self.sock.listen(0)          # put the socket into listening mode           
            print ("BASESTATION: Socket created and put into listening mode")     
            sys.stdout.flush()                       
            # Establish connection with client.
            print ("BASESTATION: Waiting to accept a connection")
            sys.stdout.flush()
            self.connection, addr = self.sock.accept()    
            print ('BASESTATION: Accepted connection from', addr)	
            sys.stdout.flush()
            
            # self.sock.setblocking(False)               # Don't block on receive             
            self.clientFlag = True    
            
            while (self.clientFlag and loopCntr < 20):
                time.sleep (sleeptime)      # Dag remove after debugging  
                loopCntr += 1  
                
                # Check if we got a message from our base station
                print ("BASESTATION THREAD: Checking for data packet" ) 
                sys.stdout.flush()                
                data = self.connection.recv(BUFFER_SIZE)
                if data: 
                    processCommand (data)
                    #self.baseQueue.put(data)  
                    print ("BASESTATION THREAD: Received a data packet" )
                    sys.stdout.flush()
                else:
                    # Client must have closed the connection
                    self.sock.close()
                    self.connection = None                    
                    self.clientFlag = False
                # end if
            #end inner while
            
        # end outer while
        
        print ("BASESTATION THREAD: terminating") 
        sys.stdout.flush()
    #end def    
    
    ###########################################################################
    # sendCommand (telemPkt)   
    ###########################################################################
    def sendTelemetry (self, telemPkt):
        if self.connection == None:
            print ("BASESTATION:sendTelemetry - ERROR, no connection\n" )
            sys.stdout.flush()            
        else:
            self.connection.send(telemPkt)  
            print ("BASESTATION:sendTelemetry - Sending telemetry packet\n" )
            sys.stdout.flush()            
        # end if 
    #end  
    
    ###########################################################################
    # sendString (string)   
    ###########################################################################
    def sendString (self, string):
        if self.connection == None:
            print ("BASESTATION:sendString - ERROR, no connection\n" )  
            sys.stdout.flush()
        else:
            self.connection.send(string)         
            print ("BASESTATION:sendString - Sending string\n" )  
            sys.stdout.flush()            
        # end if     
   
    #end  
    
    ###########################################################################
    # killThread - stops the baseStationFlag thread  
    ########################################################################### 
    def killThread (self):
        self.threadFlag = False
        print ("BASESTATION:killThread: setting threadFlag false") 
        sys.stdout.flush()
    # end
    
# End class    
       
    
###############################################################################
# TESTING
###############################################################################
if __name__ == "__main__":

    from simulator       import *
    dataQueue  = Queue(400)
    
    hostname = socket.gethostname()
    print ("MAIN: hostname is %s\n" % (hostname))
    sys.stdout.flush()    
    
    base = baseCommClass(dataQueue)
    
    for x in range(0, 60):
        print ("MAIN: updating state")   
        vehUpdateState(1)
        #print_telemetry()
        print ("MAIN: packing telemtry pkt")
        telemPkt = packTelemetryPkt()
        print ("MAIN: Sending telemtry")       
        base.sendTelemetry (telemPkt)

        time.sleep (1)
        
    # end for
    
    base.killThread()
    
# end


    
    
    
    
    
    
