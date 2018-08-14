import serial
from rplidar import RPLidar
import time


PORT_NAME = '/dev/ttyUSB0'

lidar    = None
scandata = None
scan_iter= None
###############################################################################
# init_scan0
############################################################################### 
'''def init_scan0():
    global scandata
    global lidar
    
    lidar = RPLidar(PORT_NAME )
    #info = lidar.get_info()
    #print (info)
    
    lidar.start_motor()
    scandata = lidar.iter_scans()
#end init_scan    
'''
###############################################################################
# init_scan1 
############################################################################### 
def init_scan1():
    global scandata
    global lidar
    global scan_iter
    
    lidar = RPLidar(PORT_NAME )
    #info = lidar.get_info()
    #print (info)
    
    lidar.start_motor()
    #scan_iter = lidar.iter_measures()
#end init_scan    
    
###############################################################################
# scan0 
############################################################################### 
'''def scan0():

    while (True): 
        
        time.sleep (0.2)  
        
        curr_time = time.time()
        print ( "------------- PRE-TIME %f" % (curr_time) )
        meas = next(scandata)
        nMeas = len(meas)
        print ( "nMeas %d" % (nMeas) )
        #print (meas[0])

        print ( "measure time %f\n" % ( time.time() - curr_time ) )
#end scan()    
'''    
###############################################################################
# scan1 
############################################################################### 
def scan1():
    global scan_iter
    
    while (True): 
        
        time.sleep (0.2)  
        
        curr_time = time.time()
        print ( "------------- PRE-TIME %f" % (curr_time) )
        
        new_cnt = 0
        old_cnt = 0                              
        scan_list = []
        iterator = lidar.iter_measures('normal', 3000)
        for new_scan, quality, angle, distance in iterator:
            if new_scan:
                new_cnt += 1
                if len(scan_list) > 5:
                    break
            else:
                old_cnt += 1                                    
            if distance > 0:
                scan_list.append((quality, angle, distance))                
                
                
                
                
                
        print ( "new_cnt %d old_cnt %d" % (new_cnt, old_cnt) )
        print ( "measure time %f\n" % ( time.time() - curr_time ) )
    # end while
            
#end scan()        
    
###############################################################################
# stopScan 
###############################################################################   
def stopScan():
    if lidar != None:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()   
# end stopScan    

###############################################################################
# MAIN-LOOP TESTING
###############################################################################
if __name__ == "__main__":
    init_scan1()
    try:
        scan1() 
    except:
        pass
        
    stopScan()    
        
# end    


