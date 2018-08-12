///////////////////////////////////////////////////////////////////////////////
// scanner - Functions associated with the rotating scanner.  
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Header files
///////////////////////////////////////////////////////////////////////////////
#include <StepControl.h>
// #include <ADC.h>
#include <math.h>
#include "telemetry.h"
#include "scanner.h"
#include "constants.h"
#include "TFMiniClass.h"

///////////////////////////////////////////////////////////////////////////////
// extern global variables - defined in the main file
///////////////////////////////////////////////////////////////////////////////
extern Telemetry telem;                // The telemetry class

///////////////////////////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////////////////////////
#define SCN_M0          24      // Pin for bit 0 of step divider
#define SCN_M1          25      // Pin for bit 1 of step divider
#define SCN_M2          26      // Pin for bit 2 of step divider

#define SCN_ENABLE       6      // Pin to enable the scanner
#define SCN_RESET        8      // Pin to reset the scanner
#define SCN_MOTOR1       2      // Pin for motor signal 1
#define SCN_MOTOR2       3      // Pin for motor signal 2

#define SCN_ZERO_PULSE  37      // Pin for zero signal/pulse 
#define SCN_STEP_PULSE   4      // Pin for step signal/pulse

#define SCN_SENSOR1     A15     // Pin for sensor 0 (~0 degrees)
#define SCN_SENSOR2     A12     // Pin for sensor 1 (~90 degrees)
#define SCN_SENSOR3     A14     // Pin for sensor 2 (~180 degrees)
#define SCN_SENSOR4     A13     // Pin for sensor 3 (~270 degrees)

#define SCN_DIAGNOSTICS         // Output diagnostics

const int SCN_DEBUG    = 0;       // Eliminate dag
///////////////////////////////////////////////////////////////////////////////
// local global variables
///////////////////////////////////////////////////////////////////////////////

Stepper motor(SCN_MOTOR1, SCN_MOTOR2);  // The stepper motor object
StepControl<> controller;               // The step controller

volatile int scn_stepCntr;              // Count of steps/pulses since zero
volatile int scn_zeroFlag;              // Flag indicating zero pulse occurred
volatile int scn_maxCnt;                // The maximum count this revolution
         int scn_uStepsPerInput;        // Number of uSteps output per one
                                        // pulse input (set by hw divider)
                                        
// Set scanner speed to 3200 (1600) steps per second.  There are 16 step micro-steps
// (pulses) per full step and 100 full steps per rev.                                        
         int scn_uStepsPerRev = 3200;   // Number of uSteps per 1 motor rev.                                    

TFMiniClass tfmini(TFPORT);             //The TFMini range finder
///////////////////////////////////////////////////////////////////////////////
// function templates
/////////////////////////////////////////////////////////////////////////////// 
void     scn_enable(int enable);
uint16_t scn_convertLongRange(float voltage);  
uint16_t scn_convertShortRange(float voltage);
int      scn_getAngle (void);
void     scn_stepIsr (void);
void     scn_zeroIsr (void);

///////////////////////////////////////////////////////////////////////////////
// scn_init - initialize the scanner and the ADC for the IR sensors
///////////////////////////////////////////////////////////////////////////////
void scn_init ()
{
    // initialize the I/O pins
    pinMode(SCN_M0, OUTPUT);
    pinMode(SCN_M1, OUTPUT);
    pinMode(SCN_M2, OUTPUT);    
    
    pinMode(SCN_ENABLE, OUTPUT);
    pinMode(SCN_RESET,  OUTPUT);    
    pinMode(SCN_MOTOR1, OUTPUT);
    pinMode(SCN_MOTOR2, OUTPUT);    
    
    pinMode(SCN_STEP_PULSE, INPUT_PULLUP);
    pinMode(SCN_ZERO_PULSE, INPUT);   
    
    // The TFMini is initialized in the class constructor
    TFPORT.begin(115200);    
    
    // SCANNER MOTOR SETUP. 
    motor.setMaxSpeed (scn_uStepsPerRev);   // Set pulses/steps per second
    motor.setAcceleration (800);            //
    motor.setInverseRotation(true);         //
    motor.setPullInSpeed (100);             // Step speed at startup  
    
    // Setup the motor step divider (which causes the step interrupt)
    // M0 M1 M2     motor step divisor
    // 0  0  0      1/1   200
    // 1  0  0      1/2   400
    // 0  1  0      1/4   800
    // 1  1  0      1/8  1600
    // 0  0  1      1/16 3200    <------
    // 1  0  1      1/32 6400
    // 0  1  1      1/32 
    // 1  1  1      1/32 
    digitalWrite(SCN_M0, 0);
    digitalWrite(SCN_M1, 0);
    digitalWrite(SCN_M2, 1); 
    scn_uStepsPerInput = 16;    
    
    // Initialize our globals
    scn_stepCntr = 0;
    scn_zeroFlag = false;    
    scn_maxCnt   = 0;

    // Lastly attach the two interrupts
    attachInterrupt(SCN_STEP_PULSE, scn_stepIsr, RISING);
    attachInterrupt(SCN_ZERO_PULSE, scn_zeroIsr, FALLING);         

    // Start scanning
    scn_setSpeed (1600);
    // scn_enable(1);
}

///////////////////////////////////////////////////////////////////////////////
// scn_getValues - gets the current scanner position and ranges and puts them
// into the telemetry.
///////////////////////////////////////////////////////////////////////////////
void scn_getValues (uint32_t currTime)
{
    static int counter = 0;
    
    int retVal = tfmini.getRange();
    if (retVal == 9)                        // We got a range message   
    {
        telem.scnDist1 = tfmini.rangeMsg.msg.distance;
        telem.scnDist2 = tfmini.rangeMsg.msg.strength;
        telem.scnDist3 = tfmini.nResync;
        telem.scnDist4 = tfmini.nDropped;         
        
        // Get the angle of the scanner
        scn_getAngle ();  
        counter = 0;        
        
        // We ostensibly have a valid packet from the TFMini.  Lets check
        if (tfmini.rangeMsg.msg.strength == 0)
        {
            // This seems to be an indicator of bad data
            tfmini.reset();
        }
    }
    else
    {
        // Not enough characters in buffer
        counter++;
        
        if (counter++ % 50 == 0)
        {
            telem.scnDist1 = 1234;
            telem.scnDist2 = retVal;
            telem.scnDist3 = tfmini.nResync;
            telem.scnDist4 = tfmini.nDropped;  
        } 
    }
}

///////////////////////////////////////////////////////////////////////////////
// SCN_GETANGLE - get the current position of the scanner in hundredths of
// a degree.  
///////////////////////////////////////////////////////////////////////////////
int scn_getAngle (void)
{
    int stepCnt = scn_stepCntr;        // Get the a angle count of the scanner
    if (stepCnt != scn_stepCntr)       // Get it again to make sure 
    {                                   // we werent' interrupted
        stepCnt = scn_stepCntr;        // An interrupt just occured so 
    }                                   // it won't occur again this soon.
    
    float angle = ((float) stepCnt * 360.0) / ((float) scn_uStepsPerRev);
    telem.scanAng = (uint16) stepCnt ; // (uint16) ( (angle / 10.0) + 0.5 );    
    return stepCnt;
    
    //float   currStepCnt     = (float) scn_stepCntr;   
    //float   numStepsPerRev  = (float) scn_uStepsPerRev / (float) scn_uStepsPerInput;
    //float   currAngle       = currStepCnt * 360 /  numStepsPerRev;
    //int     intCurrAngle    = (int) (currAngle * 100 + 0.5);
    //return (intCurrAngle);
}

///////////////////////////////////////////////////////////////////////////////
// SCN_ENABLE - enable/start the scanner.  
///////////////////////////////////////////////////////////////////////////////
void scn_enable(int enable)
{
    if (enable)
    {
        digitalWrite(SCN_RESET,  1);      // dag - should this go down?
        digitalWrite(SCN_ENABLE, 0);
        controller.rotateAsync(motor);                    
    }
    else
    {
        digitalWrite(SCN_RESET,  0); 
        digitalWrite(SCN_ENABLE, 1);
        // controller.rotateAsync(motor);           
    }
}

///////////////////////////////////////////////////////////////////////////////
// SCN_SETSPEED - set the rotational speed of the scanner.  
///////////////////////////////////////////////////////////////////////////////
void scn_setSpeed (int speed)
{
    scn_uStepsPerRev = speed;   
    scn_enable(0);    
    motor.setMaxSpeed (scn_uStepsPerRev);  
    scn_enable(1);        
}

///////////////////////////////////////////////////////////////////////////////
// SCN_ZEROISR - ISR for the zero pulse.  The zero pulse simply resets the 
// step counter (in the stepIsr).  
///////////////////////////////////////////////////////////////////////////////
void scn_zeroIsr (void)
{
    scn_zeroFlag = true;
}

///////////////////////////////////////////////////////////////////////////////
// SCN_STEPISR - ISR for the motor step pulse.  The actual pulse has been 
// divided by a hardware divider and set by the M0, M1, m2 pins. Every time
// a step occurs this interrupt fires which simply increments the step counter.
// If a zero pulse had occurred we just reset the step counter.  By doing all
// the step counter manipulation here we eliminate a potential race condition
// between the two interrupts. 
///////////////////////////////////////////////////////////////////////////////
void scn_stepIsr (void)
{
    if (scn_zeroFlag)       // If the zero pulse occurred zero the step counter
    { 
        scn_maxCnt   = scn_stepCntr;
        scn_stepCntr = 0; 
        scn_zeroFlag = false;
    }     
    else                    // Otherwise just increment the step counter
    {
        scn_stepCntr++;              
    } 
}

///////////////////////////////////////////////////////////////////////////////
// DEAD CODE
///////////////////////////////////////////////////////////////////////////////
