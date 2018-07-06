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
#GUI_IPADDR   = "127.0.0.1"
GUI_IPADDR   = 'localhost'       
#GUI_IPADDR   = ''     
GUI_MAINPORT = 61432    # The TCP port for cmds/tlm to/from the Rpi
GUI_IOPPORT  = 61433    # The UDP port for tlm from IOP 
GUI_VISPORT  = 61434    # Tje UDP port for tlm from VIS

###############################################################################
# Constants used throughout the code
###############################################################################

###############################################################################
# What we know abut the track
###############################################################################
trackWidth      = (16 * 12 * 2.54)      # Width between walls - 16 feet (cm)
trackLength     = 13116                 # Approx length of track (cm)

###############################################################################
# Course obstacle sequence - this array holds the sequence of obstacles which
# the vehicle will encounter
###############################################################################
trObstacle1     = obstacle.BARRELS      # The order we will face the obstacles
trObstacle2     = obstacle.RAMP         # BARRELS RAMP HOOP PEDESTRIAN NONE
trObstacle3     = obstacle.HOOP         # COURSE_END ALL
trObstacle4     = obstacle.PEDESTRIAN
trObstacle5     = obstacle.COURSE_END 
trObstacle6     = obstacle.ALL

obstacleSequence = []    
obstacleSequence.append (trObstacle1)
obstacleSequence.append (trObstacle2)
obstacleSequence.append (trObstacle3)
obstacleSequence.append (trObstacle4)
obstacleSequence.append (trObstacle5)    
obstacleSequence.append (trObstacle6)   
    
###############################################################################   
""" Occupancy Grid constants:  We need to represent an area at least 52 feet wide 
(2 * 16 feet for the track and an additional 2 * 10 feet for the track veering
off to the right/left during turns).  The 16 feet track width is included twice 
since we always keep the car centered in the X center of the grid (Xcenter,0), 
and then the car can be either all the way to the right or all the way to left.  
The height of the map corresponds to 16 feet high, the max range of the sensor. 
Since the grid cells are only bins we can set the resolution value fairly large.
"""
###############################################################################
ogResolution    = 15    # Size of each cell in the occupancy grid (cm)
ogNcols         = 104   # At 15 cm (6") this corresponds to 52 feet wide
ogNrows         = 32    # At 15 cm cells this is 16 feet (the range of the sensors)
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
speedMax        = 100   # Maximum speed we'll ever go
speedApproach   = 50    # The speed we'll approach obstacle with
speedHoop       = 50    # The speed we'll negotiate the hoop obstacle
speedRamp       = 60    # The speed we'll jump the ramp
speedPed        = 40    # The speed we'll negotiate the pedestrian
speedBarrels    = 30    # The speed we'll negotiate the barrels
speedMin        = 20    # The minimum speed (except zero) we'll ever go
speedZero       = 0     # Stopped

###############################################################################
