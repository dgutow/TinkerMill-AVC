# vehicleState.py
"""
 vehicleState.py 
 Class to store the current state of the robot
 Class to store all the possible run modes of the robot
 
 Written by David Gutow 8/2017
"""

from rangeSensorPair import rangeSensorPair
from raceModes       import raceModes
from rangeClass      import Range

###############################################################################
# class obstacle - an enumeration of the track obstacles we know about
###############################################################################
class obstacle (object):
    NONE         =  0
    PEDESTRIAN   =  80
    STOPSIGN     =  83 
    CROSSWALK    =  67
    RAMP         =  82
    HOOP         =  72 
    BARRELS      =  66  
    COURSE_END   =  69
    ALL          =  65
# end class  

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
    
    iopSwitchStatus    = 0x00  # Bitfield of status of each bump switch
    iopStartSwitch     = False # Start switch been pushed   
   
    # The last seconds worth of scan ranges are stored in this buffer.   
    iopRanges          = Range(40)
    
    iopBattVolt1       = 0.0   # Voltage of battery 1
    iopBattVolt2       = 0.0   # Voltage of battery 2
    
    iopAccelVert       = 0.0   # Value of the vertical accelerometer
    iopGyroHoriz       = 0.0   # Gyro value in horizontal plane
    iopCompassAngle    = 0.0   # Compass angle
    iopCameraAngle     = 0.0   # Vertical angle of the camera
    iopSpare2          = 0.0   # spare    
    iopSpare3          = 0.0   # spare 
    
    # These are the results of the two range sensors
    leftWallAngle      = 0.0   # The angle of the vehicle (relative to left wall)
    leftWallDist       = 0.0   # The calculated distance to left wall
    leftDataValid      = False # Data valid flag
    rightWallAngle     = 0.0   # The angle of the vehicle (relative to right wall)    
    rightWallDist      = 0.0   # The calculated distance to right wall
    rightDataValid     = False # Data Valid flag
    controlAngle       = 0.0   # The calculated vehicle control angle
    controlDist        = 0.0   # The calculated vehicle dist (to the left wall)
    compassAngle       = 0.0   # The compass angle the last time these values were
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
