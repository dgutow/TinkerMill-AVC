"""
 visionClass.py - All the vision processing support routines
 
 Written by David Gutow 9/2017
"""

import time
import struct
from multiprocessing import Process,  Queue

###############################################################################
# Cmds for the vision process
###############################################################################
VIS_NONE             = 'N'   # No current command
VIS_ALL_VERT         = 'A'   # Look for all vertical obstacles...
VIS_BARRELS          = 'B'   # a 'vertical' obstacle
VIS_PEDESTRIAN       = 'P'   # a 'vertical' obstacle
VIS_STOP_SIGN        = 'S'   # a 'vertical' obstacle
VIS_HOOP             = 'H'   # a 'vertical' obstacle
VIS_RAMP             = 'R'   # a 'vertical' obstacle
VIS_ALL_HORIZ        = 'L'   # Look for all horizontal obstacles...
VIS_HORIZ_WALK       = 'C'   # a 'horizontal' obstacle
VIS_HORIZ_END        = 'E'   # a 'horizontal' obstacle
VIS_TERMINATE        = 'Q'   # Terminate the vision task

###############################################################################
# visionTlmQueue is queue to send telemetry packets from the vision task
# visionCmdQueue is queue to send message packets to the vision task
###############################################################################
visTlmQueue  = Queue()
visCmdQueue  = Queue()

visProcess   = 0    # Need to declare it somewhere

###############################################################################
# initVisionProcess - used to start the multiprocessing task for vision
###############################################################################

def initVisProcess():
    print "initVisProcess: Starting up the vision task. "
    #print "initVisProcess: number of cpus ", cpu_count()
    
    visProcess = Process(target=visProcessTask, args=(1,))
    visProcess.start()
    time.sleep (0.5)
# end def

###############################################################################
# initVisionProcess - used to start the multiprocessing task for vision
###############################################################################
def cmdVisProcess(cmd):
    # all the valide cmds...
    if (cmd == VIS_ALL_VERT     or cmd == VIS_BARRELS       or 
        cmd == VIS_PEDESTRIAN   or cmd == VIS_STOP_SIGN     or 
        cmd == VIS_HOOP         or cmd == VIS_RAMP          or 
        cmd == VIS_ALL_HORIZ    or cmd == VIS_HORIZ_WALK    or 
        cmd == VIS_HORIZ_END    or cmd == VIS_TERMINATE     ):
        visCmdQueue.put (cmd)
        return True
    else:
        return False
    # end
#end
    
###############################################################################
# termVisionProcess - used to terminate the multiprocessing task
###############################################################################

def termVisProcess():
    print "termVisProcess: Terminating the vision task."
    cmdVisProcess(VIS_TERMINATE)
    print 'termVisProcess: Waiting to join...'   

    # make sure to swallow anything on the visTlmQueue DAG
    
    # visProcess.join()       
# end def
    
###############################################################################
# visProcessTask - the vision proc, hoepfully running on a seperate processor core.
###############################################################################
def visProcessTask( params ):
    #print ("VISION PROCESS starting loop") 
    visProcFlag   = True
    currCmd       = VIS_NONE
    cnt = 0

    while (visProcFlag and cnt < 1000):
        # Check if we received a new command...
        if (visCmdQueue.qsize() > 0):
            try:
                currCmd = visCmdQueue.get_nowait()
                print ("VISION PROCESS received command - ", currCmd)
            except:
                print ("Vision Process exception in nowait")
            else:
                pass
            # end				
        else:
            print ("VISION PROCESS loop, cnt = %d, flag = %d" % 
											(cnt, visProcFlag))			
        # end   
                 
        if   (currCmd == VIS_NONE):
            time.sleep (0.2)        # just sleep until we get a command           
        elif (currCmd == VIS_ALL_VERT):
            visAll_Vert()
        elif (currCmd == VIS_BARRELS):
            visBarrels()
        elif (currCmd == VIS_PEDESTRIAN):
            visPedestrian()
        elif (currCmd == VIS_STOP_SIGN):
            visStopSign()
        elif (currCmd == VIS_HOOP):
            visHoop()     
        elif (currCmd == VIS_RAMP):
            visRamp()  
        elif (currCmd == VIS_ALL_HORIZ):
            visAll_Horiz()
        elif (currCmd == VIS_HORIZ_WALK):
            visHorizWalk()
        elif (currCmd == VIS_HORIZ_END):
            visHorizEnd()
        elif (currCmd == VIS_TERMINATE):
            visProcFlag   = False
            break     # Terminate this bad boy
        else:
            print ("VISION PROCESS received bad command - ", currCmd)   
            currCmd = VIS_NONE
        #end if ...
             
        cnt += 1
    # end while
    print ("VISION PROCESS terminating")  
    
#end def  

###############################################################################
# The various imaging subroutines -
###############################################################################
delay = 0.2
def visAll_Vert():
    time.sleep(delay)
    return ( (VIS_NONE, 0.0, 0.0) )
#end

def visBarrels():
    cnt = 0
    while (cnt < 10000000):
		cnt += 1	
    print ("Returning from visBarrels") 
    return ( (VIS_NONE, 0.0, 0.0) )
#end

def visPedestrian():
    time.sleep(delay)
    return ( (VIS_NONE, 0.0, 0.0) )
#end

def visStopSign():
    time.sleep(delay)
    return ( (VIS_NONE, 0.0, 0.0) )
#end

def visHoop():
    cnt = 0
    while (cnt < 10000000):
        cnt += 1
    return ( (VIS_NONE, 0.0, 0.0) )
#end

def visRamp():
    time.sleep(delay)
    return ( (VIS_NONE, 0.0, 0.0) )
#end

def visAll_Horiz():
    time.sleep(delay)
    return ( (VIS_NONE, 0.0, 0.0) )
#end

def visHorizWalk():
    time.sleep(delay)
    return ( (VIS_NONE, 0.0, 0.0) )
#end

def visHorizEnd():
    time.sleep(delay)
    return ( (VIS_NONE, 0.0, 0.0) )
#end


###############################################################################
    
if __name__ == "__main__":
    initVisProcess()
    secs = 0
  
    time.sleep (2.0)   
    cmdVisProcess(VIS_BARRELS)    
        
    while (secs < 20):
        print ("MAIN-LOOP: secs = ", secs)
        cnt = 0
        while (cnt < 10000000):
            cnt += 1	

        secs += 1
        #time.sleep (1.0)        
    # end
    
    print ("Main done")       
    termVisProcess()
    
# end
    
    
###############################################################################
# class visionClass -
###############################################################################
"""
class visionClass (object):

    visProcFlag  = True             # Use to kill the process   
        
    ########################################################################### 
    # __init__
    ###########################################################################
    def __init__(self):
        self.visProcFlag    = True 
        self.visionCmdQueue = visionCmdQueue
        self.visionTlmQueue = visionTlmQueue        
        
        # start the vision process and give it some time to start up
        visPool = multiprocessing.Pool(1)
        visPool.map ( self.visProcess, (visionCmdQueue, visionTlmQueue) )

        time.sleep (0.2)  
    # end __init__
    
    ########################################################################### 
    # visProcess(state)
    ###########################################################################
    def visProcess( self, params ):
        print ("VISION PROCESS starting loop") 
        cmdQueue, tlmQueue = params[0], params[1]
        cnt = 0
    
        while (self.visProcFlag and cnt < 300):

            while (not cmdQueue.empty()):
                msgPkt = cmdQueue.get_nowait()           
            # end while
        
            time.sleep (0.1)                # DAG turn off when we get real port       
            cnt += 1
            
        # end while
        print ("VISION PROCESS terminating")   
    #end def    
    
    ###########################################################################
    # sendCommand (command, param1, param2, param3)   
    ###########################################################################
    def sendTelem (self, commandChar, param1, param2, param3):
        # pack the command into a struct
        packedArray  = struct.pack('>hhhh', ord(commandChar), param1, param2, param3)
    
        # send it along
        cmdQueue.put(packedArray)  
    #end  
    
    ###########################################################################
    # killProcess - stops the vision process 
    ########################################################################### 
    def killProcess (self):
        self.serialPortFlag = False
    # end
    
# End class    
"""    
    
    
    
    
    
