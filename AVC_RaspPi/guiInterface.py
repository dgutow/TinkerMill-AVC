"""
guiInterface.py 
Interface to the host GUI for diagnostics and testing purposes.  This
interface is not used during a race.  There are three sockets defined:
    mainSock - the main interface to the host.  Receives commands and 
               sends the main program telemetry (TCP socket)
    iopUdpSock  - echos the telemetry packet received from the IOP to the  
               host (UDP socket)
    visUdpSock  - echos the telemetry packet received from the Vision process
               to the host (UDP socket) 
               
Written by David Gutow 4/2018
"""

import os
import socket
import struct
from   printOut         import *
from   constants        import *       # Vehicle and course constants
from   tcpSocketClass   import *
from   queue            import Queue
from   vehicleState     import *

############################################################################## 
# class guiIfClass - 
##############################################################################
class guiIfClass (object):

    # hostQueue is queue to receive cmd packets from the GUI host
    guiQueue  = Queue(10)
    
    ###########################################################################
    # __init__   
    ###########################################################################
    def __init__(self, tcpIp = '', tcpPort = 61432, 
                       udpIp = '', iopUdpPort  = 61433, visUdpPort  = 61434):                                             
        self.hostIp      = tcpIp                # IP addr of host puter
        self.mainPort    = tcpPort              # Port number for main tlm/cmds
        self.udpIp       = udpIp                # IP Addr of UDP server
        self.iopUdpPort  = iopUdpPort           # Port number for iop tlm
        self.visUdpPort  = visUdpPort           # Port number for vision tlm
        self.mainAddr    = (tcpIp, tcpPort)   # complete address of ports
        self.iopUdpAddr  = (udpIp, iopUdpPort)
        self.visUdpAddr  = (udpIp, visUdpPort)
        self.acceptCnt   = 0                    # cnt of accepted cmds from host
        #self.howOften    = 10      
        
        try:
            # Set up the two UDP ports
            self.iopUdpSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            #self.iopUdpSock.connect (self.iopTlmAddr) 
            self.visUdpSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            #self.visUdpSock.connect (self.visTlmAddr)
            printOut("GUIINTERFACE: UDP ports setup complete")            
        except:
            printOut("GUIINTERFACE: ERROR unable to set up UDP ports")
            
        try:
            self.guiTcpSock = tcpSocketClass(TCP, self.hostIp, self.mainPort, 
                                                      True, self.guiQueue) 
            printOut("GUIINTERFACE: TCP ports setup complete")                                                       
        except:
            printOut("GUIINTERFACE: ERROR unable to set up TCP port")        
            
    # end __init__
    
    ###########################################################################
    # destructor  - closes up all ports and closes the queue 
    ###########################################################################    
    def __del__(self):                                             
        self.guiTcpSock.close()
    # end __del__    
    
    ###########################################################################
    # get_cmd 
    ###########################################################################    
    def get_cmd (self):
        cmd = ""
        try:
            cmd = self.guiQueue.get_nowait()
            printOut ("GUIINTERFACE:GET_CMD - RECEIVED CMD") 
        except:
            #printOut ("GUIINTERFACE:GET_CMD - DID NOT RECEIVE CMD")         
            pass
        # end try
        return (cmd)
    # end get_cmd
    
    ###########################################################################
    # send_rpiTlm - sends a telemetry packet from the main processor. 
    ###########################################################################     
    def send_rpiTlm (self, guiAcceptCnt, vehState):   
            #try: 
            # convert delta times to milliseconds
            lidar_get_data_time = int(vehState.lidar_get_data_time * 1000)
            grid_enter_data_time= int(vehState.grid_enter_data_time * 1000)
            hist_get_angle_time = int(vehState.hist_get_angle_time * 1000)
            grid_send_data_time = int(vehState.grid_send_data_time * 1000)
            histAngle           = int(vehState.histAngle * 10)
            
            # '<' - little-endian (win), 'L' - ulong, 'h' - short, 'B' - uchar
            #data = struct.pack('<LLhhBBBB', 1,2,3,4,5,6,7,8)
            data = struct.pack('<LLhhhhhhhhhh',  
                                0x22222222,
                                int(vehState.iopTime),
                                int(guiAcceptCnt),
                                int(vehState.mode.currMode),
                                histAngle, 
                                int(vehState.leftWallDist),
                                int(vehState.RightWallDist),
                                int(vehState.iopCumDistance), 
                                lidar_get_data_time,
                                grid_enter_data_time,
                                hist_get_angle_time,
                                grid_send_data_time)

            self.guiTcpSock.sendString(data)        
            #except:
            #printOut ("GUIINTERFACE:send_rpiTlm - ERROR Unable to send telemetry")  
            #pass       
    # end send_mainTlm
    
    ###########################################################################
    # send_iopTlm - sends a telemetry packet from the IOP processor.  
    ###########################################################################     
    def send_iopTlm (self, tlmMsg):      
        try:    
            self.iopUdpSock.sendto(tlmMsg, (self.udpIp, self.iopUdpPort))
            #printOut("GUIINTERFACE:SEND_IOPTLM - sending msg len (%d)" % ( len(tlmMsg)) )           
        except:
            printOut("GUIINTERFACE:SEND_IOPTLM - ERROR Unable to send telemetry")        
    # end send_iopTlm    
        
    ###########################################################################
    # send_visTlm - sends a telemetry packet from the Vision processor. 
    ###########################################################################
    def send_visTlm (self, tlmMsg):   
        try:
            self.visUdpSock.sendto(tlmMsg, (self.udpIp, self.visUdpPort))
        except:
            printOut("GUIINTERFACE:SEND_VISTLM - ERROR Unable to send telemetry")
    # end send_visTlm   

    ###########################################################################
    # close  - closes up all ports and closes the queue 
    ###########################################################################
    def close (self):   
        self.guiTcpSock.close()
    # end close       
    
# end class   

###############################################################################
# TESTING
###############################################################################
if __name__ == "__main__":   
    import time
    tlmMsg = "1234567890123456789012345678901234567890123456789012"   
    
    # Interface to the GUI host
    #iopUdpSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #iopUdpSock.sendto (tlmMsg, ("127.0.0.1", 61433))
    
    guiIf = guiIfClass ("127.0.0.1", GUI_MAINPORT, GUI_IOPPORT, GUI_VISPORT)
    time.sleep (1)
    
    print ("TEST: Sending iopTlm")
    guiIf.send_iopTlm (tlmMsg)
    print ("TEST: Completed iopTlm")  
    guiIf.close()
# end         
