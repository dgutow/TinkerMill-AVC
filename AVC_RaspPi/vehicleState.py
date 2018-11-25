# vehicleState.py
"""
 vehicleState.py 
 Class to store the current state of the robot
 Class to store all the possible run modes of the robot
 
 Written by David Gutow 8/2017
"""

#from rangeSensorPair import rangeSensorPair
from raceModes          import raceModes
from constants          import *        # Vehicle and course constants
#from rangeClass      import Range


###############################################################################
# Enumeration of the modes of the IOP
###############################################################################
IOP_MODE_NONE    = -1    # No mode yet
IOP_MODE_BIST    =  0    # In BIST/RESET mode
IOP_MODE_NORMAL  =  1    # In Normal mode
IOP_MODE_ESTOP   =  2    # In emergency stop mode         
    
###############################################################################
# class vehicleState - everything we know about the vehicle
###############################################################################
class vehicleState (object):

    mode               = raceModes()  # The current racing mode of the system
    
    timeSinceStart     = 0.0   # Seconds since the start signal arrived    
    timeAtStart        = 0.0   # Time when start signal arrived
    distAtStart        = 0.0   # Distance measured when start signal arrived
    compassAtStart     = 0.0   # Compass angle when start signal arrived
    
    # map state information
    currentAngleLock   = threading.lock() # a lock to prevent race conditions
    currentAngle       = 0.0
    
    # Telemetry coming from the IOP processor
    iopTime            = 0     # IOP current time
    iopMode            = IOP_MODE_NONE  # Mode of the IOP processor
    iopAcceptCnt       = 0     # Number of accepted commands                               
    iopBistStatus      = 0xFF  # This is the IOP BIST/ESTOP word                                

    iopSpeed           = 0.0   # Current speed (cm/sec)  
    iopSteerAngle      = 0.0   # Current angle of steering
    iopCumDistance     = 0.0   # Distance covered since start (cm)
     
    # Moved outside of the vehicle state
    #leftRangeSensors   = rangeSensorPair(50, 10, 0, 100, 500, False)
    #rightRangeSensors  = rangeSensorPair(50, 10, 0, 100, 500, True)
    
    iopSwitchStatus     = 0x00  # Bitfield of status of each bump switch
    iopStartSwitch      = False # Start switch been pushed   
   
    # The last seconds worth of scan ranges are stored in this buffer.   
    #iopRanges           = Range(40)
    # a circular buffer of the LIDAR readings
    lidarBuffer = [[0]*180 for i in range(5)]
    lidarBufferLock = threading.lock() # a lock to prevent race conditions

    iopBattVolt1        = 0.0   # Voltage of battery 1
    iopBattVolt2        = 0.0   # Voltage of battery 2
    
    iopAccelVert        = 0.0   # Value of the vertical accelerometer
    iopGyroHoriz        = 0.0   # Gyro value in horizontal plane
    iopCompassAngle     = 0.0   # Compass angle
    iopCameraAngle      = 0.0   # Vertical angle of the camera
    iopBrakeStatus      = 0.0   # Status of the motor brake
    iopSpare2           = 0.0   # spare    
    iopSpare3           = 0.0   # spare 
    
    # Occupancy grid and Histogram values
    histAngle           = 0.0   # Steer angle calculated from histogram
    leftWallDist        = 0.0   # Distance to left wall
    RightWallDist       = 0.0   # Ditto
    
    # timers
    lidar_get_data_time = 0.0
    grid_enter_data_time= 0.0
    hist_get_angle_time = 0.0
    grid_send_data_time = 0.0
    
    # These are the results of the range sensors
    controlAngle        = 0.0   # The calculated vehicle control angle
    controlDist         = 0.0   # The calculated vehicle dist (to the left wall)
    compassAngle        = 0.0   # The compass angle the last time these values were
                               # updated from good data
    
    # Desires coming out of the wall follower controller
    dsrdSpeed          = 0.0   # Desired speed of vehicle
    dsrdWallAngle      = 0.0   # Desired angle of vehicle (relative to walls)
    dsrdLeftWallDist   = 0.0
    dsrdRightWallDist  = 0.0
    
    # Last vision system reported obstacle
    obstacleType       = obstacle.NONE
    obstacleHcent      = 0.0
    obstacleVcent      = 0.0    
    obstacleIndex      = 0     # The index into obstacleSequence
    
    # Booleans indicating whether we passed/negotiated each obstacle
    finishedStopSign   = False # Completed the stop sign
    finishedPedestrian = False # Completed pedestrian
    finishedRamp       = False # Completed the ramp
    finishedHoop       = False # Completed the hoop
    finishedBarrels    = False # 
    
    # Heartbeat going to the IOP
    currHeartBeat      = 0
    
    # Bling
    currSoundInPlay    = 0
    currLightScene     = 0
    
    # Error handling
    errorString        = ""
    
    def __init__(self):
        self.mode.setMode(raceModes.NONE)
        self.timeSinceStart  = 0.0 
        self.distSinceStart  = 0.0   
        self.currCompassDir  = 0.0  
    #end   
# end class    
###############################################################################
