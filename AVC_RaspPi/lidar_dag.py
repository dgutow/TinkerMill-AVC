# this file covers all of the outward facing aspects of the lidar
import serial
import time
import math
import matplotlib.pyplot as plt
import numba

import constants as ct        # Vehicle and course constants
from rplidar import *
from vehicleState    import *       # Everything we know about the vehicle
from OccupGrid_v5_1  import Grid

PORT_NAME = '/dev/ttyUSB0'
PORT_NAME = 'COM4'

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
#@profile
def get_lidar_data(lidar, vehState, occGrid):             
    """ Returns nothing : it does update the lidar buffer in the vehicle state
    new_scan : bool - True if measures belongs to a new scan
    quality  : int  - Reflected laser pulse strength
    angle    : float- The measure heading angle in degree unit [0, 360)
    distance : float- Measured object distance related to the sensor's 
                          rotation center (mm). 
    """
    nPoints = 0
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
    
    # get the number of readings so that we don't chase an ever filling buffer
    numReadings=lidar._serial.in_waiting

    # cull down to approximately 360 readings
    while lidar._serial.in_waiting>=40*dSize:
        lidar._serial.read(((lidar._serial.in_waiting-12*dSize) //dSize )* dSize)

    # process the packets
    count = 0
    data = lidar._serial.read(((lidar._serial.in_waiting) //dSize )* dSize)
    while (84*(count+1) <= len(data)):

        #curTime = time.clock()
        #readings =  process_data(data, lidar)
        readings =  process_data(data[(84*count):(84*(count+1))], lidar)
        count =count+1
        #print("process time: ",(time.clock()-curTime))
        
        if len(readings) == 0:
            print("starting new scan")
            # we started a new scan, so wait
            return
            
        if type(readings) is not np.ndarray:
            # Error occured getting data, clear out the serial buffer
            lidar.scanning[0] = False
            lidar.clean_input()
            lidar.scanning[0] = True
            print("error getting data")
            return
        else:

            nPoints += readings.shape[0] # for diagnostics
            # transfer the data to the local buffer
            #curTime = time.clock()
            transferToBuffer(readings, current_time, vehState.lidarBuffer)
            #print("transfer: ",(time.clock()-curTime))
        # end if .. else
    # end while
    
    if ct.DEVELOPMENT:
        plotBuffer(vehState.lidarBuffer)
        while lidar._serial.in_waiting>4000:
            lidar._serial.read((lidar._serial.in_waiting //dSize )* dSize)
        
    return nPoints
# end def

###############################################################################
# transferToBuffer 
###############################################################################
@numba.jit()
def transferToBuffer(readings, current_time, lidarBuffer):

    readings[np.where(readings[:,LIDAR_READING_DISTANCE]== 0),LIDAR_READING_DISTANCE] = 12000
    # get the corresponding indices in the lidarBuffer
    # note this only works because the end array has the same number of entries
    bufferIndices = np.round_(readings[:,LIDAR_READING_ANGLE]).astype(int) % 360 
    #lidarBuffer[bufferIndices,:]=[current_time, 0, \
    #    readings[:,LIDAR_READING_ANGLE], readings[:,LIDAR_READING_DISTANCE]/1000.,1]
    length = bufferIndices.shape[0]
    i=0
    while i< length:
        lidarBuffer[bufferIndices[i],:]=[current_time, 0, \
            readings[i,LIDAR_READING_ANGLE], readings[i,LIDAR_READING_DISTANCE]/1000.,1]
        i=i+1


###############################################################################
# plotBuffer 
###############################################################################
#@numba.jit()
def plotBuffer(lidarBuffer):
        #print("start")
    #print(lidarBuffer[:,LIDAR_BUFFER_TIME]-lidarBuffer[1,LIDAR_BUFFER_TIME])
    #print("stop")
    points = np.zeros((lidarBuffer.shape[0],2))
    points = np.linspace(0,2*math.pi,360)
    #print(points.shape)
    points=np.append(points,np.sin(points))
    points=np.reshape(points,(360,2),'F')
    #print(points)
    points[0:360,0]=np.cos(points[0:360,0])
    #print(points)
    
    points[:,0]=np.multiply(points[:,0],lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE])
    points[:,1]=np.multiply(points[:,1],lidarBuffer[:,ct.LIDAR_BUFFER_DISTANCE])
    #for i in range(360):
    #    points[i,:]= points[i,:]*lidarBuffer[i,ct.LIDAR_BUFFER_DISTANCE]
    #print("break")
    #print(points)

    #plt.close()
    plt.cla()
    plt.plot(points[:,0],points[:,1],linestyle=' ',marker='.',markersize=5,color='k')
    plt.xlim((-12,12))
    plt.ylim((-12,12))
    plt.grid(True)

    #plt.show(block=False)
    plt.show()
    #plt.draw()
    plt.pause(0.001)

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

 

################################################################################
# MAIN-LOOP TESTING
################################################################################
if __name__ == "__main__":
    
    ############################################################################ 
    # Initialize only what is necessary
    ############################################################################
    def initializations():
    
        # Vehicle State holds everything known about the current vehicle state
        vehState = vehicleState()
    
        # start and initialize the RPLidar
        lidar = init_lidar_scan()
 
        occGrid = Grid (ogResolution, ogNrows, ogNcols, ogStartDist, ogStartAngle)
    
        time.sleep(1.0) 

        return lidar, occGrid, vehState
    # end initializations      

    ############################################################################ 
    # test code
    ############################################################################
    lidar, occGrid, vehState =initializations()
    curr_time = time.time()    
    print ( "------------- PRE-TIME %f" % (curr_time) ) 
    
    if (1):
        try:
            while (True):
                #time.sleep (0.10)  
                curr_time = time.time()
                nPoints = get_lidar_data(lidar, vehState, occGrid)
                print ( "\nget_lidar_data time %f, nPoints %d" % ( (time.time() - curr_time ), nPoints) )
                
                for dataPt in vehState.lidarBuffer:
                    print ("%6.3f \t%6.3f \t%6.3f" % (dataPt[0], dataPt[2], dataPt[3]) )
                    pass
            #end while
        except:
            print ("EXCEPTION: stopping scanner")
            stop_lidar_scan()

    else:
        
        while (True):   
            time.sleep (0.1)  
            
            curr_time = time.time()
            print ( "------------- PRE-TIME %f" % (curr_time) )    
            scan_list = get_lidar_data(lidar, vehState, occGrid)
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
