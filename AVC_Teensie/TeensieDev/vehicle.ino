///////////////////////////////////////////////////////////////////////////////
// vehicleControl - Functions associated with moving, steering and controlling
// the vehicle.  Devices include the drive motor, steering and the brake.
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Local header files
///////////////////////////////////////////////////////////////////////////////
#include "constants.h"
#include "telemetry.h"
#include "vehicle.h"
#include <Servo.h>

///////////////////////////////////////////////////////////////////////////////
// extern global variables - defined in the main file
///////////////////////////////////////////////////////////////////////////////
extern Telemetry telem;                // The telemetry class

///////////////////////////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////////////////////////

// Servo definitions for speed control
#define SPD_PIN_NUMBER        29
#define SPD_CONVERSION         5    // Convert from cm/second to microseconds
#define SPD_MID_PULSEWIDTH  1500    // Servo neutral position
#define SPD_MIN_PULSEWIDTH  1000
#define SPD_MAX_PULSEWIDTH  2000

// Servo definitions for turning servo
#define DIR_PIN_NUMBER        30
#define DIR_CONVERSION       -15    // Convert from cm/second to microseconds
#define DIR_MID_PULSEWIDTH  1500    // Servo neutral position
#define DIR_MIN_PULSEWIDTH  1000
#define DIR_MAX_PULSEWIDTH  2000

// Wheel encoder definitions
#define ENC_LEFT_PIN          11    // dag!
#define ENC_RIGHT_PIN         12

#define ENC_STEPS_PER_REV    6.0    // 6 poles per revolution
#define ENC_DEG_PER_STEP    60.0    // 6 poles per revolution
#define VEH_WHEEL_DIAM     114.3    // mm.  Corresponds to 4.5 inches
#define VEH_WHEEL_CIRCUM   (VEH_WHEEL_DIAM * 3.14159)
#define VEH_WHEEL_SPACING   25.4    // spacing between two front wheels 10" (cm) 

///////////////////////////////////////////////////////////////////////////////
// Global variables
///////////////////////////////////////////////////////////////////////////////
uint32  lt_nSteps         = 0;     // left wheel - total number of steps 
uint32  lt_StepTimes[3]   = {0,0,0};    // left wheel - time last 3 steps occurred 
                                   // [0]-last step,[1]-step before that, etc.
float   lt_linearSpeed    = 0;     // left wheel - linear speed (cm/sec) 
float   lt_rotateSpeed    = 0;     // left wheel - rotational speed deg/sec
float   lt_totalDist      = 0;     // left wheel - total distance covered

uint32  rt_nSteps         = 0;     // right wheel - total number of steps 
uint32  rt_StepTimes[3]   = {0,0,0};    // right wheel - time last 3 steps occurred
                                   // [0]-last step,[1]-step before that, etc.  
float   rt_linearSpeed    = 0;     // right wheel - linear speed (cm/sec) 
float   rt_rotateSpeed    = 0;     // right wheel - rotational speed deg/sec
float   rt_totalDist      = 0;     // right wheel - total distance covered 

uint32  veh_lastTimeMsec  = 0;     // time (msec) of last getTelem call
int32   veh_speed         = 0;     // cm/sec
int32   veh_turnRadius    = 0;     // cm
int32   veh_distance      = 0;     // commanded distance (cm)

int32   veh_moveDistance  = 0;     // Cumulative distance of move command 

Servo   spdServo;                  // Servo controlling the vehicle speed
Servo   dirServo;                  // Servo controlling the vehicle direction

///////////////////////////////////////////////////////////////////////////////
// Templates
///////////////////////////////////////////////////////////////////////////////
void veh_leftWheelInt  (void);
void veh_rightWheelInt (void);

///////////////////////////////////////////////////////////////////////////////
// initialize everything
///////////////////////////////////////////////////////////////////////////////
void veh_init()
{
    // Attach/Initialize the two servos
    spdServo.attach (SPD_PIN_NUMBER, SPD_MIN_PULSEWIDTH, SPD_MAX_PULSEWIDTH);
    dirServo.attach (DIR_PIN_NUMBER, DIR_MIN_PULSEWIDTH, DIR_MAX_PULSEWIDTH);
    
    // attach two interrupts
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN),  veh_leftWheelInt,  RISING );
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), veh_rightWheelInt, RISING );    
}

///////////////////////////////////////////////////////////////////////////////
// veh_move - start or continue a move command
///////////////////////////////////////////////////////////////////////////////
uint32_t veh_move (int16_t speed, uint16_t distance)
{
    if (telem.currMode != NORMAL)
    { 
        // Can't execute a move command unless in NORMAL mode.
        return (false);
    }
    
    veh_speed = speed;          // Open loop speed control
    telem.currSpeed = speed;
    veh_moveDistance = distance + telem.cumDistance;  
    
    // Convert from cm/sec to Servo microseconds 
    int32_t uSeconds = SPD_MID_PULSEWIDTH + (speed * SPD_CONVERSION); 
    spdServo.writeMicroseconds(uSeconds);  
    
    return (true);
}

///////////////////////////////////////////////////////////////////////////////
// veh_turn - start or continue a turn command
///////////////////////////////////////////////////////////////////////////////
uint32_t veh_turn (int16_t angle)
{
    if (telem.currMode != NORMAL)
    { 
        // Can't execute a turn command unless in NORMAL mode.
        return (false);
    }

    telem.currSteerAng = angle;
    
    // Convert from degrees to Servo microseconds 
    int32_t uSeconds = DIR_MID_PULSEWIDTH + (angle * DIR_CONVERSION); 
    dirServo.writeMicroseconds(uSeconds);      
     
    return (true);      
}

///////////////////////////////////////////////////////////////////////////////
// veh_brake - apply or release the brake
///////////////////////////////////////////////////////////////////////////////
uint32_t veh_brake (uint16_t onOff)
{
    if (onOff == 1)
    {
        // Apply the brake
        // todo
        return (true);       
    }
    else if (onOff == 0)
    {
        // Release the brake
        // todo
        return (true);         
    }
    
    // If neither of the above don't increment the accept counter
    return (false);
}

///////////////////////////////////////////////////////////////////////////////
// veh_estop - estop the vehicle
///////////////////////////////////////////////////////////////////////////////
uint32_t veh_estop ()
{
    // Shutdown the drive motor
    veh_move (0, 0);
    
    // Apply the brake
    veh_brake (1);
    return (true);
}

///////////////////////////////////////////////////////////////////////////////
// veh_getTelem - calculates and puts into telemetry all the vehicle values
///////////////////////////////////////////////////////////////////////////////
void veh_getTelem (uint32 currTimeMsec)
{
    uint32 l_nSteps;
    uint32 r_nSteps;
    uint32 l_StepTimes[3];
    uint32 r_StepTimes[3]; 

    // Get the last step times, but they may be changed by the ISR so 
    // get them if lt_nSteps changed    
    l_nSteps       = lt_nSteps;
    l_StepTimes[0] = lt_StepTimes[0];
    l_StepTimes[1] = lt_StepTimes[1];   
    l_StepTimes[2] = lt_StepTimes[2];
    if (l_nSteps != lt_nSteps)
    {
        // If they don't match an interrupt just occurred so get them again       
        l_StepTimes[0] = lt_StepTimes[0];
        l_StepTimes[1] = lt_StepTimes[1];   
        l_StepTimes[2] = lt_StepTimes[2];  
    }   

    // Get the last step times, but they may be changed by the ISR so 
    // get them if rt_nSteps changed 
    r_nSteps       = rt_nSteps;    
    r_StepTimes[0] = rt_StepTimes[0];
    r_StepTimes[1] = rt_StepTimes[1];   
    r_StepTimes[2] = rt_StepTimes[2];
    if (r_nSteps != rt_nSteps)
    {
        // If they don't match an interrupt just occurred so get them again       
        r_StepTimes[0] = rt_StepTimes[0];
        r_StepTimes[1] = rt_StepTimes[1];   
        r_StepTimes[2] = rt_StepTimes[2];  
    }
        
    // Calculate the current rotational speeds.    
    lt_rotateSpeed  = (ENC_DEG_PER_STEP * 1000) / (l_StepTimes[0]-l_StepTimes[1]);
    rt_rotateSpeed  = (ENC_DEG_PER_STEP * 1000) / (r_StepTimes[0]-r_StepTimes[1]);    
    
    // Now convert to linear speed
    lt_linearSpeed = (lt_rotateSpeed * VEH_WHEEL_CIRCUM ) / 360 ;
    rt_linearSpeed = (rt_rotateSpeed * VEH_WHEEL_CIRCUM ) / 360 ;
    
    // Time from the last getTelem call to this one
    uint32 dTime = currTimeMsec - veh_lastTimeMsec;
    veh_lastTimeMsec = currTimeMsec;
    
    // Fill in the telemetry fields
    veh_speed = (uint16) ((lt_linearSpeed + rt_linearSpeed) / 2);
    telem.currSpeed = veh_speed;
    
    // Cumulative distance is just average of left/right distance
    float lt_distance = ( (float) lt_nSteps *  VEH_WHEEL_CIRCUM) /  ENC_STEPS_PER_REV;
    float rt_distance = ( (float) rt_nSteps *  VEH_WHEEL_CIRCUM) /  ENC_STEPS_PER_REV;    
    telem.cumDistance = (uint16) ( (lt_distance + rt_distance) / 2.0 );    

    // Lets put the interrpt step counters in the spares for diagnostics
    telem.spare1 = lt_nSteps;
    telem.spare2 = rt_nSteps;    
}

///////////////////////////////////////////////////////////////////////////////
// veh_check - perform all the continuing maintenance on a move and/or turn 
// command.  Called by the main loop routine
///////////////////////////////////////////////////////////////////////////////
void veh_check (uint32 currTimeMsec)
{
  // check if we're at the move limit (but only if we're not moving)
  if (veh_speed > 0 && veh_moveDistance >= telem.cumDistance)
  {
        // Yep, we're past the move limit, stop the vehicle
        veh_move (0, 0);
  }      
      
}

///////////////////////////////////////////////////////////////////////////////
// veh_getSwitches - gets the status of all the switches.  Signals an estop
// if the estop button pushed.
///////////////////////////////////////////////////////////////////////////////
void veh_getSwitches (uint32 currTimeMsec)
{
  // read all switches
  // todo
  // if estop pushed 
  //    call veh_estop and fill in the reason for estop mode.
  // todo
  
}

///////////////////////////////////////////////////////////////////////////////
// veh_leftWheelInt - interrupt routine for the left wheel encoder
///////////////////////////////////////////////////////////////////////////////
void veh_leftWheelInt (void)
{
    lt_StepTimes[2] = lt_StepTimes[1];  // Shift the last step times up
    lt_StepTimes[1] = lt_StepTimes[0];
    lt_StepTimes[0] = millis();
    
    lt_nSteps++;  
}

///////////////////////////////////////////////////////////////////////////////
// veh_rightWheelInt - interrupt routine for the right wheel encoder
///////////////////////////////////////////////////////////////////////////////
void veh_rightWheelInt (void)
{
    rt_StepTimes[2] = rt_StepTimes[1];  // Shift the last step times up
    rt_StepTimes[1] = rt_StepTimes[0];
    rt_StepTimes[0] = millis();
    
    rt_nSteps++;  
}

///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////
