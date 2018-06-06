#!/usr/bin/python
"""
 socketClass.py - All the support routines for Wifi communication

 Written by David Gutow 3/2018
"""

import time
import threading
import socket
import array
from Queue           import Queue
import sys
from printOut        import printOut, printErr

###############################################################################
# class socketClass - UDP socket support.  This class will set up a thread to 
# monitor for incoming commands iff cmdQueue is not None.  Otherwise this 
# class can only be used for outgoing messages.  The class can be set up to   
# be either a client or a server. 
###############################################################################

class socketClass (object):

    ########################################################################### 
    # __init__ - This initialization will set up a thread to monitor for 
    # incoming commands iff cmdQueue is not None.  Otherwise this class can
    # only be used for outgoing messages.
    ###########################################################################
    def __init__(self, ipAddr, portNo, server = False, cmdQueue=None, bufSize=1024):
        self.ipAddr     = ipAddr
        self.portNo     = portNo
        self.address    = (self.ipAddr, self.portNo)
        self.server     = server    # Set up as server (true) or client (false)
        self.cmdQueue   = cmdQueue
        self.bufSize    = bufSize
        self.sock       = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP         
        self.threadFlag = True       # Use to kill the serial port thread
        self.retAddress = (self.ipAddr, self.portNo) # Return addr of messages
        
        if (self.server):               # Setup this instance as a server
            printOut ("SOCKETCLASS: Setting up as UDP server - binding")
            self.sock.bind ( self.address )  
            #printOut ("SOCKETCLASS: Setting up as UDP server - listening")             
            #self.sock.listen (1)
            printOut ("SOCKETCLASS: Setting up as UDP server - complete")
        else:
            printOut ("SOCKETCLASS: Setting up as UDP client\n")           
            #self.sock.connect ( (self.ipAddr, self.portNo) )
        # endif
        
        if (cmdQueue is not None):               
            # start the command thread and give it some time to start up
            self.serThread = threading.Thread (
                group=None, target=self.receiveThread, name = "receiveThread" )
            self. serThread.start()
            time.sleep (0.2) 
            
            printOut ("SOCKETCLASS: Completed setting up thread\n")              
        # end if
        
    # end __init__
    
    ########################################################################### 
    # receiveThread(state)
    ###########################################################################
    def receiveThread(self):
        printOut ("SOCKETCLASS: starting receive thread")
        runFlag = self.threadFlag
    
        while (runFlag):
            data, address = self.sock.recvfrom(self.bufSize)
            self.retAddress = address
            self.cmdQueue.put(data)
            if (True):
                printOut ('SOCKETCLASS: Received %s bytes from address %s' 
                                            % (len(data), self.retAddress))              
            runFlag = self.threadFlag
        # end while
        
        self.sock.close()
        printOut ("SOCKETCLASS: terminating receive thread" )
    #end def    
    
    ###########################################################################
    # sendString (message)   
    ###########################################################################
    def sendString (self, message):
        printOut ('SOCKETCLASS sendString: sending %s bytes to %s' % 
                                                (len(message),self.retAddress))
        try:
            self.sock.sendto (message, self.retAddress ) 
            printOut ("SOCKETCLASS sendString: completed sending message")
        except:
            printOut ("SOCKETCLASS sendString: exception sending message")       

    #end  
    
    ###########################################################################
    # killThread - stops the serial port thread  
    ########################################################################### 
    def killThread (self):
        self.threadFlag = False        
        printOut ("SOCKETCLASS killThread: setting threadFlag false")
    # end
    
# End class    
    
###############################################################################
    
if __name__ == "__main__":
    IPADDR = "127.0.0.1"
    PORTNO = 61432

    printOut ("Getting instanceNo")
    sys.stdout.flush()
    instanceNo = sys.argv
    printOut ("instanceNo is %s\n" % (instanceNo))
    
    inputQueue  = Queue(400)    
    sc = socketClass(IPADDR, PORTNO, inputQueue, 128) 
    
    if (instanceNo == '1'):
        for i in range(0, 25):
            msg = inputQueue.get()
            printOut ("--> 1) RECEIVED - %s\n" % (msg))
            sc.sendString ("--> 1) XMITTNG msg %d" % (i))                                   
        # end for
        
    elif (instanceNo == '2'):
        sleep (2)
        for i in range(0, 25):
            sc.sendString ("2) XMITTNG msg %d" % (i))         
            msg = inputQueue.get()
            printOut ("2) RECEIVED - %s\n" % (msg))
            sleep (1)                      
        # end for   
    
    else:
        printOut ("ERROR - instance number not 1 or 2\n")
    # end elif else...
    
    sc.killThread()    
# end


    
    
    
    
    
    
