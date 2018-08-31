"""
 Constants.py  - hold all the constants about the vehicle and the track
 
 Written by David Gutow 8/2017
"""

from vehicleState    import *       # Everything we know about the vehicle

###############################################################################
# SIM_TEENSY - Used when integrating with no Teensy connected.
###############################################################################
#SIM_TEENSY	= True              # When no Teensy
SIM_TEENSY	= False            # When there is a communicating Teensy

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
UDP_IPADDR   = '192.168.4.30'   # laptop

RPI_TCPPORT  = 61432            # The TCP port for cmds/tlm to/from the Rpi
UDP_IOPPORT  = 61433            # The UDP port for tlm from IOP 
UDP_VISPORT  = 61434            # The UDP port for tlm from VIS

# For sending Occupancy Grid UDP telemetry
OCC_IPADD    = '127.0.0.1'
UDP_OCCPORT  = 12346

###############################################################################
# Constants used throughout the code
###############################################################################

###############################################################################
# What we know abut the track
###############################################################################
trackWidth      = (16 * 12 * 2.54)      # Width between walls - 16 feet (cm)
trackLength     = 13116                 # Approx length of track (cm)

###############################################################################
# class obstacle - an enumeration of the track obstacles we know about
# Course obstacle sequence - this array holds the sequence of obstacles which
# the vehicle will encounter
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

# The order we will face the obstacles: BARRELS RAMP HOOP PEDESTRIAN NONE
# COURSE_END ALL
obstacleSequence = []    
obstacleSequence.append (obstacle.BARRELS)
obstacleSequence.append (obstacle.RAMP)
obstacleSequence.append (obstacle.HOOP)
obstacleSequence.append (obstacle.PEDESTRIAN)
obstacleSequence.append (obstacle.COURSE_END)    
obstacleSequence.append (obstacle.ALL)   
    
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
speedMax        = 8     # Maximum speed we'll ever go
speedApproach   = 6     # The speed we'll approach obstacle with
speedRecov      = 6
speedHoop       = 5     # The speed we'll negotiate the hoop obstacle
speedRamp       = 6     # The speed we'll jump the ramp
speedPed        = 5     # The speed we'll negotiate the pedestrian
speedBarrels    = 5     # The speed we'll negotiate the barrels
speedMin        = 5     # The minimum speed (except zero) we'll ever go
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
# :  
###############################################################################
