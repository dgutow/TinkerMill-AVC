import serial
from   rplidar import *
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
    
    lidar = RPLidar( PORT_NAME )
    lidar.stop()
    time.sleep(0.01)
    lidar.reset()
    #clean_input()
    
    #info = lidar.get_info()
    #print (info)
    
    lidar.start_motor()
    lidar.start('normal')
#end init_scan    
            
###############################################################################
# get_lidar_data 
###############################################################################
def get_lidar_data():             # stolen from iter_measures()
    """ Returns a list of tuples:
    new_scan : bool - True if measures belongs to a new scan
    quality  : int  - Reflected laser pulse strength
    angle    : float- The measure heading angle in degree unit [0, 360)
    distance : float- Measured object distance related to the sensor's 
                          rotation center (mm). 
    """
    new_cnt     = 0
    old_cnt     = 0  
    zero_cnt    = 0    
    scan_list   = [] 
        
    if not lidar.scanning[0]:
        raise RPLidarException ('Scanning not started in scan2')
            
    dsize = lidar.scanning[1]            
    while (lidar._serial.inWaiting() >= dsize):
        data = lidar._serial.read(dsize)
        error, new_scan, quality, angle, distance =  process_data(data)
        if (error):
            # Error occured getting data, clear out the serial buffer
            lidar.scanning[0] = False
            lidar.clean_input()
            lidar.scanning[0] = True
        else:
            if distance > 0:
                scan_list.append((new_scan, quality, angle, distance))  
            else:
                zero_cnt += 1
    
            if new_scan:
                new_cnt += 1
            else:
                old_cnt += 1    
        # end if .. else
    # end while
    return scan_list        
# end def

###############################################################################
# process_data - Processes input raw data and returns measurement data
###############################################################################
def process_data (raw):         # was _process_scan(raw) in rplidar.py
    new_scan = bool(_b2i(raw[0]) & 0b1)
    inversed_new_scan = bool((_b2i(raw[0]) >> 1) & 0b1)
    if new_scan == inversed_new_scan:
        print ('New scan flags mismatch')
        return True, 0, 0, 0, 0
        
    check_bit = _b2i(raw[1]) & 0b1
    if check_bit != 1:
        print ('Check bit not equal to 1')
        return True, 0, 0, 0, 0
        
    quality  =   _b2i(raw[0]) >> 2        
    angle    = ((_b2i(raw[1]) >> 1) + (_b2i(raw[2]) << 7)) / 64.
    distance =  (_b2i(raw[3])       + (_b2i(raw[4]) << 8)) / 4.
    
    return False, new_scan, quality, angle, distance

###############################################################################
# _b2i - Converts byte to integer (for Python 2 compatability)
############################################################################### 
def _b2i(byte):
    return byte if int(sys.version[0]) == 3 else ord(byte)
    
###############################################################################
# clean_input - Clean input buffer by reading all available data
############################################################################### 
def clean_input():
    while (lidar._serial.inWaiting() > 0):    
        lidar._serial.read(1)
        
###############################################################################
# get_lidar_scan_old  - OBE
############################################################################### 
def get_lidar_scan_old():
    global scan_iter
    
    new_cnt     = 0
    old_cnt     = 0  
    zero_cnt    = 0    
    scan_list   = []
    try:
        iterator = lidar.iter_measures('normal', -1)
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
    except:
        print ( "GET_LIDAR_SCAN - ERROR exception" )     
        lidar.scanning[0] = False
        lidar.clean_input()
        lidar.scanning[0] = True   
            
    print ( "new_cnt %d old_cnt %d zero_cnt %d" % (new_cnt, old_cnt, zero_cnt) )
    return scan_list        
    
#end scan()   

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
# MAIN-LOOP TESTING
###############################################################################
if __name__ == "__main__":
    init_lidar_scan()
    
    if (1):
        try:
                time.sleep (0.10)  
                curr_time = time.time()
                print ( "------------- PRE-TIME %f" % (curr_time) )    
                scan_list = get_lidar_data()
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
            scan_list = get_lidar_data()
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


