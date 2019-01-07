"""
 Constants.py  - hold all the constants about the vehicle and the track
 
 Written by David Gutow 8/2017
"""

#from vehicleState    import *       # Everything we know about the vehicle
import math as math

DEVELOPMENT = True

###############################################################################
# SIM_TEENSY - Used when integrating with no Teensy connected.
###############################################################################
#SIM_TEENSY	= True              # When no Teensy
SIM_TEENSY	= False            # When there is a communicating Teensy

###############################################################################
# random physical constants
###############################################################################
METERS_PER_FOOT  = 0.3048
DEG_TO_RAD       = math.pi/180
RAD_TO_DEG       = 180 / math.pi

###############################################################################
# TCP/UDP communication parameters to the GUI host
###############################################################################
#RPI_IPADDR   = "127.0.0.1"
#GUI_IPADDR   = 'localhost'       
#GUI_IPADDR   = '10.2.124.96'   # TM wifi
#GUI_IPADDR   = '192.168.4.100' # 8266 access point

#RPI_IPADDR   = '10.2.124.96'   # the Rpi
#UDP_IPADDR   = '10.2.122.53'   # laptop

RPI_IPADDR   = '192.168.4.10'   # the Rpi
UDP_IPADDR   = '192.168.4.30'   # Daves laptop

RPI_TCPPORT  = 61432            # The TCP port for cmds/tlm to/from the Rpi
UDP_IOPPORT  = 61433            # The UDP port for tlm from IOP 
UDP_VISPORT  = 61434            # The UDP port for tlm from VIS

# For sending Occupancy Grid UDP telemetry
OCC_IPADD    = '192.168.4.30'   # Dave's laptop
#OCC_IPADD    = '192.168.4.101'   # TJ's laptop
UDP_OCCPORT  = 12346

###############################################################################
# Constants used throughout the code
###############################################################################

###############################################################################
# What we know abut the track
###############################################################################
trackWidth      = (16 * 12 * 2.54)      # Width between walls - 16 feet (cm)
trackLength     = 13116                 # Approx length of track (cm)
wheelBase       = 11 / 12 * METERS_PER_FOOT
vehicleWidth    = 9/12*METERS_PER_FOOT
###############################################################################
# class obstacle - an enumeration of the track obstacles we know about
# Course obstacle sequence - this array holds the sequence of obstacles which
# the vehicle will encounter
###############################################################################
OBSTACLE_NONE         =  0
OBSTACLE_PEDESTRIAN   =  80
OBSTACLE_STOPSIGN     =  83 
OBSTACLE_CROSSWALK    =  67
OBSTACLE_RAMP         =  82
OBSTACLE_HOOP         =  72 
OBSTACLE_BARRELS      =  66  
OBSTACLE_COURSE_END   =  69
OBSTACLE_ALL          =  65
# end class 

# The order we will face the obstacles: BARRELS RAMP HOOP PEDESTRIAN NONE
# COURSE_END ALL
obstacleSequence = []    
obstacleSequence.append (OBSTACLE_BARRELS)
obstacleSequence.append (OBSTACLE_RAMP)
obstacleSequence.append (OBSTACLE_HOOP)
obstacleSequence.append (OBSTACLE_PEDESTRIAN)
obstacleSequence.append (OBSTACLE_COURSE_END)    
obstacleSequence.append (OBSTACLE_ALL)   
    
###############################################################################   
""" 
Occupancy Grid constants:  We need to represent an area at least 52 feet wide 
(2 * 16 feet for the track and an additional 2 * 10 feet for the track veering
off to the right/left during turns).  The 16 feet track width is included twice 
since we always keep the car centered in the X center of the grid (Xcenter,0), 
and then the car can be either all the way to the right or all the way to left.  
The height of the map corresponds to ~33 feet high, the max range of the sensor. 
Since the grid cells are only bins we can set the resolution value fairly large.
"""
###############################################################################
ogResolution    = 10    # Size of each cell in the occupancy grid (cm)
ogNcols         = 160   # At 10 cm this corresponds to ~52 feet wide
ogNrows         = 100   # At 10 cm cells this is 33 feet (the range of the sensors)
ogStartDist     = 0.0   # Initial cumulative distance when system intitialized
ogStartAngle    = 0.0   # Initial angle of the vehicle when system initialized

###############################################################################
# Vehicle control constants:
###############################################################################
vcKprop         = 10.0  # Proportional constant
vcKint          = 1.0   # Integral constant
vcFast          = 200   # Distance for quick wall offset response
vcMed           = 300   # Distance for medium wall offset response
vcSlow          = 400   # Distance for slow wall offset response

###############################################################################
#The two rangeSensorPair constants:  
# Note - all angles are the angle of the sensors from a line perpindicular  
# to the vehicle sides.
###############################################################################
rsLeftFrontAng  = 30    # The angle of the left front sensor (deg)
rsLeftRearAng   = 5     # The angle of the left rear sensor (deg)
rsRightFrontAng = 30    # The anlge of the right front sensor (deg)
rsRightRearAng  = 5     # The angle of the right rear sensor (deg)
rsMinDistance   = 100   # The min distance accepted (cm)
rsMaxDistance   = 500   # The max distance accepted (cm)
rsLeftSide      = False # Used to distinguish the left side sensor pair
rsRigthSide     = True  # Used to distinguish the left side sensor pair
rsFRspacing     = 20    # The spacing between the front and rear sensors (cm)
rsLRspacing     = 15    # The spacing between the left and right sensor pairs

###############################################################################
# The various speed values:  
###############################################################################
speedMax        = 10     # Maximum speed we'll ever go
speedApproach   = 5     # The speed we'll approach obstacle with
speedRecov      = 5
speedHoop       = 3     # The speed we'll negotiate the hoop obstacle
speedRamp       = 5     # The speed we'll jump the ramp
speedPed        = 3     # The speed we'll negotiate the pedestrian
speedBarrels    = 3     # The speed we'll negotiate the barrels
speedMin        = 3     # The minimum speed (except zero) we'll ever go
speedZero       = 0     # Stopped

###############################################################################
# The commands to the IOP:  
###############################################################################
CMD_MOVE         = 'M'
CMD_TURN         = 'T'
CMD_ESTOP        = 'E'
CMD_HEARTBEAT    = 'H'
CMD_LIGHTING     = 'L'
CMD_SPEEDPID     = 'P'
CMD_TURNPID      = 'Q'
CMD_MODE         = 'D'
CMD_NOP          = 'N'
CMD_SCANSPEED    = 'S'
CMD_SCANANGLE    = 'A'
CMD_CAMANGLE     = 'V'
CMD_SCANENABLE   = 'C'
CMD_BRAKE        = 'B'

###############################################################################
# The lidar buffer columns:  
###############################################################################
LIDAR_BUFFER_TIME       = 0
LIDAR_BUFFER_QUALITY    = 1
LIDAR_BUFFER_ANGLE      = 2
LIDAR_BUFFER_DISTANCE   = 3
LIDAR_BUFFER_USED       = 4 # I am thinking that the test for this would be value % prime ==0

###############################################################################
# The lidar readings columns:  
###############################################################################
#LIDAR_READING_ERROR     = 0
LIDAR_READING_NEWSCAN   = 0
LIDAR_READING_QUALITY   = 1
LIDAR_READING_ANGLE     = 2
LIDAR_READING_DISTANCE  = 3

###############################################################################
# :  
###############################################################################
