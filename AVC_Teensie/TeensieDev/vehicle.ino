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
#define TRN_PIN_NUMBER        30
#define TRN_CONVERSION        -5    // Convert from cm/second to microseconds
#define TRN_MID_PULSEWIDTH  1500    // Servo neutral position
#define TRN_MIN_PULSEWIDTH  1000
#define TRN_MAX_PULSEWIDTH  2000

// Servo definitions for brake
#define BRK_PIN_NUMBER        35
#define BRK_CONVERSION         5    // Convert from off to on
#define BRK_MID_PULSEWIDTH  1500    // Brake off position
#define BRK_MIN_PULSEWIDTH  1000
#define BRK_MAX_PULSEWIDTH  2000

// Wheel encoder definitions
#define ENC_LEFT_PIN          27    // Joe 7/9/2018
#define ENC_RIGHT_PIN         28

// The switches
#define VEH_START_SWITCH_PIN  22
#define VEH_ESTOP_SWITCH_PIN  23

#define ENC_STEPS_PER_REV    6.0    // 6 poles per revolution
#define ENC_DEG_PER_STEP    60.0    // 6 poles per revolution
#define VEH_WHEEL_DIAM     11.43    // 11.43 cm = 4.5"  7/20/2018
#define VEH_WHEEL_CIRCUM   (VEH_WHEEL_DIAM * 3.14159)   // cm
#define VEH_DIST_PER_STEP   (ENC_DEG_PER_STEP * VEH_WHEEL_CIRCUM / 360.0)

#define VEH_WHEEL_SPACING  25.24    // 25.24 cm = 9.938" spacing between 
                                    // front wheels 7/20/2018
// Estimation of max wheel encoder interrupts per second
// max speed  30 mph = 13.4 m/s = 13,400 mm/s
// Wheel circumference = 359.1 mm 
// 13,400/359.1 = 37.31 rps
// 6 pulses / rev -> 224 pps

///////////////////////////////////////////////////////////////////////////////
// Global variables
///////////////////////////////////////////////////////////////////////////////
// Globals for the left wheel
volatile uint32  lt_nSteps  = 0;    // total number of wheel encoder steps 
volatile uint32  lt_StepTimes[3] = {0,0,0};    // time of last 3 encoder steps 
volatile float lt_totalDist = 0;    // linear speed (cm/sec)
float   lt_linearSpeed      = 0;    // linear speed (cm/sec) 
float   lt_rotateSpeed      = 0;    // degrees/second 

// Globals for the right wheel
volatile uint32  rt_nSteps  = 0;   // total number of wheel encoder steps  
volatile uint32  rt_StepTimes[3] = {0,0,0};    // time of last 3 encoder steps
volatile float rt_totalDist = 0;    // linear speed (cm/sec) 
float   rt_linearSpeed      = 0;    // linear speed (cm/sec) 
float   rt_rotateSpeed      = 0;    // degrees/second 

int32   veh_cmd_speed       = 0;    // commanded speed from move cmd (cm/sec)
int32   veh_cmd_dist        = 0;    // command dist from last move (cm)
int32   veh_moveDistance    = 0;    // Cutoff dist from last move (cm) 
                                    // (relative to cumulative distance) 
int32   veh_turnRadius      = 0;    // cm

Servo   spdServo;                   // Servo controlling vehicle speed
Servo   trnServo;                   // Servo controlling vehicle steering
Servo   brkServo;                   // Servo controlling vehicle brake
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
    // Attach/Initialize the servos
    spdServo.attach (SPD_PIN_NUMBER, SPD_MIN_PULSEWIDTH, SPD_MAX_PULSEWIDTH);
    trnServo.attach (TRN_PIN_NUMBER, TRN_MIN_PULSEWIDTH, TRN_MAX_PULSEWIDTH);
    brkServo.attach (BRK_PIN_NUMBER, BRK_MIN_PULSEWIDTH, BRK_MAX_PULSEWIDTH);
    veh_move (0,0);
    veh_turn(0);
    veh_brake(0);
    
    // attach two interrupts
    pinMode(ENC_LEFT_PIN, INPUT);
    pinMode(ENC_RIGHT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN),  veh_leftWheelInt,  RISING );
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), veh_rightWheelInt, RISING );  

    // Set switches 
    pinMode(VEH_START_SWITCH_PIN,INPUT);
    pinMode(VEH_ESTOP_SWITCH_PIN,INPUT);    
    telem.switches = 0;   
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
    
    veh_cmd_speed       = speed;   
    veh_cmd_dist        = distance;
    telem.currSpeed     = speed;
    veh_moveDistance    = telem.cumDistance + veh_cmd_dist;  
    
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
    int32_t uSeconds = TRN_MID_PULSEWIDTH + (angle * TRN_CONVERSION); 
    trnServo.writeMicroseconds(uSeconds);      
     
    return (true);      
}

///////////////////////////////////////////////////////////////////////////////
// veh_brake - apply or release the brake.  Force should go between 0 and 100.
///////////////////////////////////////////////////////////////////////////////
uint32_t veh_brake (uint16_t force)
{   
    telem.brakeStatus = force;
    
    // Convert from degrees to Servo microseconds 
    int32_t uSeconds = BRK_MID_PULSEWIDTH + (force * BRK_CONVERSION); 
    brkServo.writeMicroseconds(uSeconds);      
     
    return (true);      
}

///////////////////////////////////////////////////////////////////////////////
// veh_estop - estop the vehicle
///////////////////////////////////////////////////////////////////////////////
uint32_t veh_estop ()
{
    // Shutdown the drive motor
    veh_move (0, 0);
    
    // Apply the brake
    veh_brake (100);
    return (true);
}

///////////////////////////////////////////////////////////////////////////////
// veh_getTelem - calculates and puts into telemetry all the vehicle values
///////////////////////////////////////////////////////////////////////////////
void veh_getTelem (uint32 currTimeMsec)
{
    // Get the switch status
    veh_getSwitches (currTimeMsec);
  
    // Fill in the telemetry fields
    telem.currSpeed = (uint16) ((lt_linearSpeed + rt_linearSpeed) / 2); 

    // Lets put the interrupt step counters in the spares for diagnostics
    telem.volt1 = lt_nSteps;
    telem.volt2 = rt_nSteps;    
}

///////////////////////////////////////////////////////////////////////////////
// veh_check - perform all the continuing maintenance on a move and/or turn 
// command.  Called by the main loop routine
///////////////////////////////////////////////////////////////////////////////
void veh_check (uint32 currTimeMsec)
{
    uint32 l_nSteps;
    uint32 r_nSteps;
    uint32 l_StepTimes[3];
    uint32 r_StepTimes[3]; 
    
    // Get the left last step times.   
    l_nSteps       = lt_nSteps;
    l_StepTimes[0] = lt_StepTimes[0];
    l_StepTimes[1] = lt_StepTimes[1];   
    l_StepTimes[2] = lt_StepTimes[2];
    
    // Get lt_nSteps again.  If they are different then an ISR just occurred 
    // changed the values.  Get them again.  Since an ISR just occurred a new
    // one cant' re-occur this quickly after the last one.
    if (l_nSteps != lt_nSteps)
    {     
        l_StepTimes[0] = lt_StepTimes[0];
        l_StepTimes[1] = lt_StepTimes[1];   
        l_StepTimes[2] = lt_StepTimes[2];  
    }   

    // Get the right last step times.
    r_nSteps       = rt_nSteps;    
    r_StepTimes[0] = rt_StepTimes[0];
    r_StepTimes[1] = rt_StepTimes[1];   
    r_StepTimes[2] = rt_StepTimes[2];
    
    // And get them again, yada, yada...
    if (r_nSteps != rt_nSteps)
    {      
        r_StepTimes[0] = rt_StepTimes[0];
        r_StepTimes[1] = rt_StepTimes[1];   
        r_StepTimes[2] = rt_StepTimes[2];  
    }
        
    // Calculate the current rotational speeds (deg/sec)    
    lt_rotateSpeed  = (ENC_DEG_PER_STEP * 1000) / (l_StepTimes[0]-l_StepTimes[1]);
    rt_rotateSpeed  = (ENC_DEG_PER_STEP * 1000) / (r_StepTimes[0]-r_StepTimes[1]);    
    
    // Now convert to linear speed (cm/sec)
    lt_linearSpeed = (lt_rotateSpeed * VEH_WHEEL_CIRCUM ) / 360 ;
    rt_linearSpeed = (rt_rotateSpeed * VEH_WHEEL_CIRCUM ) / 360 ;        
    
    // Our cumulative distance travelled is just the average of the two
    // total distances of our wheels.  
    telem.cumDistance = (int) ( (lt_totalDist + rt_totalDist) / 2 );
    
    // check if we're at the move limit (but only if we're not moving)
    if ( veh_cmd_speed > 0 && (telem.cumDistance >= veh_moveDistance) )
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
  telem.switches  =  digitalRead(VEH_START_SWITCH_PIN);         // bit 0
  
  bool estop = digitalRead(VEH_ESTOP_SWITCH_PIN);  
  if (estop) 
  {
    // Estop button was pushed, stop the vehicle and go into estop mode
    telem.switches |= 0x02;                                     // bit 1   
    veh_estop ();
    telem.currMode = ESTOP;
    telem.bist = ESTOP_BUTTON;
  }
}

///////////////////////////////////////////////////////////////////////////////
// veh_leftWheelInt - interrupt routine for the left wheel encoder
///////////////////////////////////////////////////////////////////////////////
void veh_leftWheelInt (void)
{
    lt_StepTimes[2] = lt_StepTimes[1];  // Shift the last step times up
    lt_StepTimes[1] = lt_StepTimes[0];
    lt_StepTimes[0] = millis(); 
    
    // Calc our new total distance
    lt_totalDist  += VEH_DIST_PER_STEP;
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
    
    // Calc our new total distance
    rt_totalDist  += VEH_DIST_PER_STEP;
    rt_nSteps++;  
}

///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////
