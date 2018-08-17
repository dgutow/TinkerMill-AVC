import serial
from   rplidar import RPLidar
import time


PORT_NAME = '/dev/ttyUSB0'

lidar    = None
scandata = None
scan_iter= None

###############################################################################
# init_scan 
############################################################################### 
def init_lidar_scan():
    global lidar
    global scandata
    global scan_iter
    
    lidar = RPLidar(PORT_NAME )
    #info = lidar.get_info()
    #print (info)
    
    lidar.start_motor()
    #scan_iter = lidar.iter_measures()
#end init_scan    
       
###############################################################################
# scan1 
############################################################################### 
def get_lidar_scan():
    global scan_iter
    
    new_cnt     = 0
    old_cnt     = 0  
    zero_cnt    = 0    
    scan_list   = []
    
    iterator = lidar.iter_measures('normal', 10000)
    for new_scan, quality, angle, distance in iterator:
        if distance > 0:
            scan_list.append((new_scan, quality, angle, distance))  
        else:
            zero_cnt += 1
    
        if new_scan:
            new_cnt += 1
            if len(scan_list) > 5:
                break
        else:
            old_cnt += 1   

    # end for
            
    print ( "new_cnt %d old_cnt %d zero_cnt %d" % (new_cnt, old_cnt, zero_cnt) )

    return scan_list        
    
#end scan()        
    
###############################################################################
# stop_scan 
###############################################################################   
def stop_lidar_scan():
    if lidar != None:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()   
# end stop_scan    

###############################################################################
# MAIN-LOOP TESTING
###############################################################################
if __name__ == "__main__":
    init_lidar_scan()

    try:
        while (True):      
            time.sleep (0.05)  
            
            curr_time = time.time()
            print ( "------------- PRE-TIME %f" % (curr_time) )    
            scan_list = get_lidar_scan()
            print ( "measure time %f\n" % ( time.time() - curr_time ) )
            
            nScans = len(scan_list)
            for dataPt in scan_list:
               newPt = dataPt[0]
               qual  = dataPt[1]
               angle = dataPt[2]
               dist  = dataPt[3]
               #print ("New %d, qual %d, angle %d, dist %d\n" % (newPt, qual, angle, dist))
            # end for
            
        # end while        
  
    except:
        pass
        
    stop_lidar_scan()       
# end    


