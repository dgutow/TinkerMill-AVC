# this file covers all of the outward facing aspects of the lidar
import serial
import time
import math
import matplotlib.pyplot as plt

import constants as ct        # Vehicle and course constants
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
    
    plt.ion()
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
        # don't bother processing this packet if it will just get 
        # overwritten later
        if (lidar._serial.inWaiting() >= 12*dSize):
            continue
            
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
            return
        else:
            for reading  in readings:
                dist=reading[LIDAR_READING_DISTANCE]
                    
                # TODO CHECK IF THE DATA ARE RELIABLE USING THE GRAVITY SENSOR
                # calculate the absolute angle and bound it to [0,2*pi)
                # also convert from left hand coordinate system to right
                absoluteAngle = (-reading[LIDAR_READING_ANGLE]/180*math.pi+currentAngle) % (math.pi*2)

                #scan_list.append((1, 0, reading[LIDAR_READING_ANGLE], dist))
                # use the absolute angle to index into the buffer
                # offset forces it to round to 1 to 180
                bufferIndex = int(round(absoluteAngle/(2*math.pi)* \
                    (len(vehState.lidarBuffer)-1)))
                if reading[LIDAR_READING_DISTANCE] == 0:
                    dist = 12000#vehState.lidarBuffer[bufferIndex,LIDAR_BUFFER_DISTANCE]+1000.
                #print("bufferI: ",bufferIndex," absAngle: ",absoluteAngle," angle ", reading[LIDAR_READING_ANGLE], "distance: ",dist/1000.);
                #vehState.lidarBufferLock.acquire()
                vehState.lidarBuffer[bufferIndex,:]=[current_time, 0, absoluteAngle, dist/1000.,1]
                #vehState.lidarBufferLock.release()
                #occGrid.enterRange(vehState.iopCumDistance, vehState.iopSteerAngle, 
                #           dist/1, reading[LIDAR_READING_ANGLE])    # data is in mm DAG - For testing
                occGrid.enterRange(vehState.iopCumDistance, vehState.iopSteerAngle, 
                           dist/10, reading[LIDAR_READING_ANGLE]) # data is in mm DAG - For real      
            # end for
        # end if .. else
    # end while
    if ct.DEVELOPMENT:
        #print("start")
        #print(vehState.lidarBuffer[:,LIDAR_BUFFER_TIME]-vehState.lidarBuffer[1,LIDAR_BUFFER_TIME])
        #print("stop")
        points = np.zeros((vehState.lidarBuffer.shape[0],2))
        points = np.linspace(0,2*math.pi,360)
        #print(points.shape)
        points=np.append(points,np.sin(points))
        points=np.reshape(points,(360,2),'F')
        #print(points)
        points[0:360,0]=np.cos(points[0:360,0])
        #print(points)
        
        points[:,0]=np.multiply(points[:,0],vehState.lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE])
        points[:,1]=np.multiply(points[:,1],vehState.lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE])
        #for i in range(360):
        #    points[i,:]= points[i,:]*vehState.lidarBuffer[i,ct.LIDAR_BUFFER_DISTANCE]
        #print("break")
        #print(points)

        #plt.close()
        plt.cla()
        plt.plot(points[:,0],points[:,1],linestyle=' ',marker='.',markersize=5)
        plt.xlim((-12,12))
        plt.ylim((-12,12))
        plt.grid(True)

        #plt.show(block=False)
        #plt.show()
        #plt.draw()
        plt.pause(0.001)
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
    vehState = vehicleState()
    
    # start and initialize the RPLidar
    lidar = init_lidar_scan()
 
    occGrid = Grid (ogResolution, ogNrows, ogNcols, ogStartDist, ogStartAngle)
    
    time.sleep(0.5) 

    return lidar, occGrid, vehState
# end initializations   

###############################################################################
# MAIN-LOOP TESTING
###############################################################################
if __name__ == "__main__":
    lidar, occGrid, vehState =initializations()
    """
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
    """    
    stop_lidar_scan()       
# end    
