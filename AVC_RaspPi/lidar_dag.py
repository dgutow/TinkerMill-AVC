# this file covers all of the outward facing aspects of the lidar
import serial
import time
import math

from constants      import *        # Vehicle and course constants
from rplidar import *
from vehicleState    import *       # Everything we know about the vehicle
from OccupGrid_v5_1  import Grid

PORT_NAME = '/dev/ttyUSB0'

#scandata = None
#scan_iter= None


###############################################################################
# init_scan 
############################################################################### 
def init_lidar_scan():
    lidar = RPLidar( PORT_NAME )
    lidar.stop()
    time.sleep(0.01)
    lidar.reset()
    #clean_input()
    
    #info = lidar.get_info()
    #print (info)
    
    lidar.start_motor()
    lidar.start('express')
    
    return lidar
#end init_scan    

###############################################################################
# get_lidar_data_circular 
###############################################################################
def get_lidar_data(lidar, vehState, occGrid):             
    """ Returns nothing : it does update the lidar buffer in the vehicle state
    new_scan : bool - True if measures belongs to a new scan
    quality  : int  - Reflected laser pulse strength
    angle    : float- The measure heading angle in degree unit [0, 360)
    distance : float- Measured object distance related to the sensor's 
                          rotation center (mm). 
    """
    # get a local copy of the current angle
    #vehState.currentAngleLock.acquire()
    currentAngle = vehState.currentAngle
    #vehState.currentAngleLock.release()

    current_time = time.clock()

    if not lidar.scanning[0]:
        raise RPLidarException ('Scanning not started in scan2')
            
    dSize = lidar.scanning[1]

    #while (dSize==84) and (lidar._serial.inWaiting() >4*dSize):
    #    lidar._serial.read(dSize)
    #    continue

    while (lidar._serial.inWaiting() >= dSize):
        data = lidar._serial.read(dSize)

        # don't pay attention to excess data
        readings =  process_data(data, lidar)
        if len(readings) == 0:
            print("starting new scan")
            # we started a new scan, so wait
            return
            
        if (readings[0][LIDAR_READING_ERROR]):
            # Error occured getting data, clear out the serial buffer
            lidar.scanning[0] = False
            lidar.clean_input()
            lidar.scanning[0] = True
            print("error getting data")
        else:
            for reading  in readings:
                dist=reading[LIDAR_READING_DISTANCE]
                if reading[LIDAR_READING_DISTANCE] == 0:
                    dist = 12000.
                # TODO CHECK IF THE DATA ARE RELIABLE USING THE GRAVITY SENSOR
                # calculate the absolute angle and bound it to [0,2*pi)
                absoluteAngle = (reading[LIDAR_READING_ANGLE]/180*math.pi+currentAngle) % (math.pi*2)

                #scan_list.append((1, 0, reading[LIDAR_READING_ANGLE], dist))
                # use the absolute angle to index into the buffer
                # offset forces it to round to 1 to 180
                bufferIndex = int(round(absoluteAngle/(2*math.pi)* \
                    (len(vehState.lidarBuffer)-1)))
                #print("bufferI: ",bufferIndex," absAngle: ",absoluteAngle," angle ", reading[LIDAR_READING_ANGLE], "distance: ",dist/1000.);
                #vehState.lidarBufferLock.acquire()
                vehState.lidarBuffer[bufferIndex,:]=[current_time, 0, absoluteAngle, dist/1000.,1]
                #vehState.lidarBufferLock.release()
                occGrid.enterRange(vehState.iopCumDistance, vehState.iopSteerAngle, 
                           dist/1, reading[LIDAR_READING_ANGLE])    # data is in mm DAG - For testing
#                occGrid.enterRange(vehState.iopCumDistance, vehState.iopSteerAngle, 
#                           dist/10, reading[LIDAR_READING_ANGLE]) # data is in mm DAG - For real      
            # end for
        # end if .. else
    # end while
    return
# end def

###############################################################################
# start_lidar_scan 
###############################################################################   
def start_lidar_scan():
    if lidar != None:
        lidar.reset()
        lidar.start_motor()
        lidar.start('normal')
# end stop_scan    
    
###############################################################################
# stop_lidar_scan 
###############################################################################   
def stop_lidar_scan():
    if lidar != None:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()   
# end stop_scan    

############################################################################### 
# Initialize only what is necessary
###############################################################################
def initializations():
    
    # Vehicle State holds everything known about the current vehicle state
    vehState        = vehicleState()
    
    # start and initialize the RPLidar
    lidar = init_lidar_scan()
 
    occGrid       = Grid (ogResolution, ogNrows, ogNcols, ogStartDist, ogStartAngle)
    
    time.sleep(0.5) 

    return lidar, occGrid, vehState
# end initializations   

###############################################################################
# MAIN-LOOP TESTING
###############################################################################
if __name__ == "__main__":
    lidar, occGrid, vehState =initializations()

    if (1):
        try:
                time.sleep (0.10)  
                curr_time = time.time()
                print ( "------------- PRE-TIME %f" % (curr_time) )    
                scan_list = get_lidar_data(lidar, occGrid, vehState)
                print ( "measure time %f\n" % ( time.time() - curr_time ) )
            
                nScans = len(scan_list)
                for dataPt in scan_list:
                    newPt = dataPt[0]
                    qual  = dataPt[1]
                    angle = dataPt[2]
                    dist  = dataPt[3]
                    print ("New %d, qual %d, angle %d, dist %d\n" % (newPt, qual, angle, dist))
                # end for 
            # end while        
        except:
            print ("ERROR - exception occured\n")
            pass

    else:
        
        while (True):   
            time.sleep (0.1)  
            
            curr_time = time.time()
            print ( "------------- PRE-TIME %f" % (curr_time) )    
            scan_list = get_lidar_data(lidar, occGrid, vehState)
            print ( "measure time %f\n" % ( time.time() - curr_time ) )
            
            nScans = len(scan_list)
            newPtCnt = 0
            for dataPt in scan_list:
                newPt = dataPt[0]
                qual  = dataPt[1]
                angle = dataPt[2]
                dist  = dataPt[3]
                if (newPt):
                    newPtCnt += 1
                #print ("New %d, qual %d, angle %d, dist %d\n" % (newPt, qual, angle, dist))
            print ("NPts %d, NewPtCnt %d\n" % (nScans, newPtCnt))                
            # end for
            
        # end while               
        
    stop_lidar_scan()       
# end    
