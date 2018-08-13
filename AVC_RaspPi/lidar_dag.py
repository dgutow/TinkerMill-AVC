import serial
from rplidar import RPLidar
import time


PORT_NAME = '/dev/ttyUSB0'


###############################################################################
# scan 
############################################################################### 
def scan():
    lidar = RPLidar(PORT_NAME )
    #info = lidar.get_info()
    #print (info)
    
    lidar.start_motor()
    scandata = lidar.iter_scans()
    
    
    while (True): 
        
        time.sleep (0.1)  
        
        curr_time = time.time()
        print ( "MAIN_LOOP - pre_time %f\n" % (curr_time) )
        #scan =  lidar.iter_scans()
        meas = next(scandata)
        #for meas in scan:
        
        print (len(meas))
        #print ( "MAIN_LOOP - number of meas %d\n" % (len(scan) )       )
        #meas =  scan[0]
        #print (list)
        print ( "MAIN_LOOP - post_time %f\n" % (curr_time) )

    
    
##################################################################an()    
#############
# stopScan 
###############################################################################   
""" 
def stopScan():
    lidar = RPLidar(portname) 
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()   
# end stopScan    
"""
###############################################################################
# MAIN-LOOP EXECUTION
###############################################################################
if __name__ == "__main__":
    ##### TEST # 1 
    #initializations()
    scan()    
# end    

"""
class LIDAR(object):
    def __init__(self, portname):
        self.portname = portname

    def scan(self):
        pointCloud = []
        lidar = RPLidar(self.portname)

        for i, measurement in enumerate(lidar.iter_measures()):
            for j, v in enumerate(measurement):
                if j == 0:
                    newscan = v
                if j == 1:
                    quality = v
                if j == 2:
                    angle = v
                if j == 3:
                    length = v

            # change angle to match orientation of vehicle
            angle = -1 * (angle - 90)

            pointCloud.append([angle, length])

            if i > 360:
               break



        return pointCloud
"""
