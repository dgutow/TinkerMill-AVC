#!/usr/bin/python
"""
 tcpSocketClass.py - All the support routines for Wifi communication

 Written by David Gutow 3/2018
"""

import time
import threading
import socket
import array
from   Queue           import Queue
import sys
from   printOut        import printOut, printErr

###############################################################################
# socket typed - defines the two enumerations:
#   TCP
#   UDP
###############################################################################
UDP = 1
TCP = 2

###############################################################################
# class tcpSocketClass - TCP socket support.  This class will set up a thread to 
# monitor for incoming commands iff cmdQueue is not None.  Otherwise this 
# class can only be used for outgoing messages.  The class can be set up to   
# be either a client or a server. 
###############################################################################

class tcpSocketClass (object):

    ########################################################################### 
    # __init__ - This initialization will set up a thread to monitor for 
    # incoming commands iff cmdQueue is not None.  Otherwise this class can
    # only be used for outgoing messages.
    ###########################################################################
    def __init__(self, sockType, ipAddr, portNo, server = True, cmdQueue=None, bufSize=1024):
        self.sockType   = sockType
        self.ipAddr     = ipAddr
        self.portNo     = portNo
        self.address    = (self.ipAddr, self.portNo)
        self.server     = server    # Set up as server (true) or client (false)
        self.cmdQueue   = cmdQueue
        self.bufSize    = bufSize
        self.conn       = None      # The current socket connection
         
        self.threadFlag = True      # Use to kill the serial port thread
        self.threadExec = False     # Indicates if the thread is running
        self.retAddress = (self.ipAddr, self.portNo) # Return addr of messages
        
        if (self.sockType != TCP and self.sockType != UDP):
            printOut("SocketClass: ERROR unknown socket type (TCP, UDP)")
            raise ValueError('SocketClass: Socket Type neither TCP or UDP')
            
        # SOCK_STREAM for TCP, SOCK_DGRAM for UDP
        if (self.sockType == TCP):
            printOut ("SocketClass: Setting up socket for TCP")         
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)             
        else:
            printOut ("SocketClass: Setting up socket for UDP")         
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    
        #end if            
        
        if (self.server):               # Setup this instance as a server
            #printOut ("TCPSOCKETCLASS: Setting up as server - binding to %s" % ((self.address)) )
            print "TCPSOCKETCLASS: Setting up as server - binding to ", self.address
            self.sock.bind ( self.address )  
            printOut ("TCPSOCKETCLASS: Setting up as server - listening")             
            self.sock.listen (1)
            printOut ("TCPSOCKETCLASS: Setting up as server - complete")
        else:
            printOut ("TCPSOCKETCLASS: Setting up as client\n")           
            self.sock.connect ( (self.ipAddr, self.portNo) )
        # endif
        
        if (cmdQueue is not None):               
            # start the command thread and give it some time to start up
            self.serThread = threading.Thread (
                group=None, target=self.receiveThread, name = "receiveThread" )
            self. serThread.start()
            while ( not self.threadExec):
                time.sleep (0.1)            
        # end if
        printOut ("TCPSOCKETCLASS: Completed setting up thread\n") 
        
    # end __init__
    
    ########################################################################### 
    # receiveThread(state)
    ###########################################################################
    def receiveThread(self):
        printOut ("TCPSOCKETCLASS: starting receive thread, waiting for accept")     
        
        # Keep waiting for a connection as long as we're told not to abort
        while (self.threadFlag):
            self.conn = None          
            printOut ("TCPSOCKETCLASS: waiting to accept connection")   
            self.threadExec = True              # Yep, the thread is executing  
            try:
                #conn, addr = self.sock.accept(0)
                conn, addr = self.sock.accept()               
                self.conn = conn
                printOut ("TCPSOCKETCLASS: accepted connection" )     
            except:
                time.sleep(1.0)
                       
            # Keep waiting for data as long as we have a connection and
            # we're not told to abort
            while (self.conn != None and self.threadFlag):
                #data, address = self.sock.recvfrom(self.bufSize)
                #self.retAddress = address                
                data = self.conn.recv(self.bufSize)                
                self.cmdQueue.put(data)
                
                if (True):
                    printOut ('TCPSOCKETCLASS: Received %s bytes from address %s' 
                                            % (len(data), self.retAddress))   
                    
                if (len(data) == 0):            # We lost the connection
                    printOut ('TCPSOCKETCLASS: connection lost')
                    self.conn = None
            # end while
        # end while
        
        self.sock.close()
        self.threadExec = False                 # Thread now terminated 
        printOut ("TCPSOCKETCLASS: receive thread terminated" )
    #end def    
    
    ###########################################################################
    # sendString (message)   
    ###########################################################################
    def sendString (self, message):
        if (self.conn != None):
            try:
                #printOut ('TCPSOCKETCLASS sendString: sending %s bytes to %s' % 
                #                                (len(message),self.retAddress))           
                size = self.conn.send ( message ) 
                if (size == 0):                 # We lost the connection
                    self.conn = None
                    printOut ("TCPSOCKETCLASS sendString: lost connection")
                else:
                    #printOut ("TCPSOCKETCLASS sendString: message sent")
                    pass
            except:
                printOut ("TCPSOCKETCLASS sendString: exception sending message")       

    #end  
    
    ###########################################################################
    # close - closes the TCP port and stops the port thread  
    ########################################################################### 
    def close (self):
        printOut ("TCPSOCKETCLASS close: Waiting for thread to terminate")    
        self.threadFlag = False   
        while (self.threadExec):
            time.sleep (0.1)
        printOut ("TCPSOCKETCLASS close: Tthread terminated")              
    # end
    
# End class    
    
###############################################################################
# TESTING
###############################################################################
#sockType, ipAddr, portNo, server = True, cmdQueue=None, bufSize=1024):
    
if __name__ == "__main__":
    IPADDR = "127.0.0.1"
    IPADDR = 'localhost'       
    IPADDR = ''    
    PORTNO = 61432

    instanceNo = sys.argv
    printOut ("instanceNo is %s\n" % (instanceNo[1]))
    
    inputQueue  = Queue(10)    
    
    if (instanceNo[1] == 'S'):                     # Server
        sock = tcpSocketClass(TCP, IPADDR, PORTNO, True, inputQueue)      
        for i in range(0, 25):
            time.sleep (3)
            try:
                msg = inputQueue.get_nowait()
                printOut ("--> 1) RECEIVED - %s\n" % (msg))                
            except:
                pass
            # end try
    
            sock.sendString ("Message %d" % (i))                                   
        # end for
        sock.close()       
        
    elif (instanceNo[1] == 'C'):                   # Client
        sleep (2)
        for i in range(0, 25):
            sc.sendString ("2) XMITTNG msg %d" % (i))         
            msg = inputQueue.get()
            printOut ("2) RECEIVED - %s\n" % (msg))
            sleep (1)                      
        # end for   
    
    else:
        printOut ("ERROR - instance number not S or C\n")
    # end elif else...
   
# end


    
    
    
    
    
    
