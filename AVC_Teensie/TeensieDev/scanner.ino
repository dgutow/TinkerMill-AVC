///////////////////////////////////////////////////////////////////////////////
// scanner - Functions associated with the rotating scanner.  
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Header files
///////////////////////////////////////////////////////////////////////////////
#include <StepControl.h>
#include <ADC.h>
#include "telemetry.h"
#include "scanner.h"
#include <math.h>

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

#define SCN_SENSOR_0    A15     // Pin for sensor 0 (~0 degrees)
#define SCN_SENSOR_1    A12     // Pin for sensor 1 (~90 degrees)
#define SCN_SENSOR_2    A14     // Pin for sensor 2 (~180 degrees)
#define SCN_SENSOR_3    A13     // Pin for sensor 3 (~270 degrees)

#define TIME_STEP_PIN   27      // Timing - Pin/output to time the step ISR
#define TIME_ZERO_PIN   13      // Timing - Pin/LED to watch the zero ISR
#define TIME_ADC_PIN    28      // Timing - Pin/output to time the ADC

#define SCN_DIAGNOSTICS         // Output diagnostics

///////////////////////////////////////////////////////////////////////////////
// local global variables
///////////////////////////////////////////////////////////////////////////////
extern  ADC *adc;                       // The ADC class
extern  ADC::Sync_result result;
extern  ADC::Sync_result result1;

Stepper motor(SCN_MOTOR1, SCN_MOTOR2);  // The stepper motor object
StepControl<> controller;               // The step controller

volatile int scn_stepCntr;              // Count of steps/pulses since zero
volatile int scn_zeroFlag;              // Flag indicating zero pulse occurred
volatile int scn_maxCnt;                // The maximum count this revolution
         int scn_uStepsPerInput;        // Number of uSteps output per one
                                        // pulse input (set by hw divider)
         int scn_uStepsPerRev = 1600;   // Number of uSteps per 1 motor rev.                                    
float valued1;
float valued2;
float valued3;
float valued4;

///////////////////////////////////////////////////////////////////////////////
// scn_init - initialize the scanner and the ADC for the IR sensors
///////////////////////////////////////////////////////////////////////////////
void scn_init ()
{
    // initialize the I/O pins
    pinMode(M0, OUTPUT);
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);    
    
    pinMode(SCN_ENABLE, OUTPUT);
    pinMode(SCN_RESET,  OUTPUT);    
    pinMode(SCN_MOTOR1, OUTPUT);
    pinMode(SCN_MOTOR2, OUTPUT);    
    
    pinMode(SCN_STEP_PULSE, INPUT_PULLUP);
    pinMode(SCN_ZERO_PULSE, INPUT);   

    pinMode(TIME_STEP_PIN, OUTPUT);
    pinMode(TIME_ZERO_PIN, OUTPUT);     //
    pinMode(TIME_ADC_PIN, OUTPUT);

    // Setup the ADC
    adc->setReference(ADC_REFERENCE::REF_3V3, ADC_0);
    adc->setReference(ADC_REFERENCE::REF_3V3, ADC_1);
    adc->setResolution(12, ADC_0);
    adc->setResolution(12, ADC_1);
    adc->setAveraging (16, ADC_0);
    adc->setAveraging (16, ADC_1);
    
    // Setup the motor step controller. Set speed to 1600 steps per second.
    // But we're doing 16 step micro-stepping, so 100 full steps per rev.
    motor.setMaxSpeed (scn_uStepsPerRev);   
    motor.setAcceleration (800);
    motor.setInverseRotation(true);
    motor.setPullInSpeed (100);    
    
    // Setup the motor step divider (which causes the step interrupt)
    // M0 M1 M2     motor step divisor
    // 0  0  0      1/1   200
    // 1  0  0      1/2   400
    // 0  1  0      1/4   800
    // 1  1  0      1/8  1600
    // 0  0  1      1/16 3200    <-
    // 1  0  1      1/32 6400
    // 0  1  1      1/32 
    // 1  1  1      1/32 
    digitalWrite(M0, 0);
    digitalWrite(M1, 0);
    digitalWrite(M2, 1); 
    scn_uStepsPerInput = 16;    // 
    
    // Initialize our globals
    scn_stepCntr = 0;
    scn_zeroFlag = false;    
    scn_maxCnt   = 0;

    // Lastly attach the two interrupts
    attachInterrupt(SCN_STEP_PULSE, scn_stepIsr, RISING);
    attachInterrupt(SCN_ZERO_PULSE, scn_zeroIsr, FALLING);  

    // Start scanning
    scn_enable();
}

///////////////////////////////////////////////////////////////////////////////
// sid_getValues - gets the current scanner position and ranges and puts them
// into the telemetry.
///////////////////////////////////////////////////////////////////////////////
void scn_getValues (uint32_t currTime)
{
    int errorFlag = 0;
    int rightFront= 0;
    int rightRear = 0;
    int leftFront = 0;
    int leftRear  = 0;
    
    // reads both front and rear sensors on the right side
    errorFlag = scn_readPair (distanceRF, distanceRR, &rightFront, &rightRear);
    if (~errorFlag)
    {
        telem.irRF = rightFront;
        telem.irRR = rightRear; 
    }
    
    if (IR_DEBUG == 1)
    {
        DBGPORT.print("Pin: ");
        DBGPORT.print(distanceRF);
        DBGPORT.print(", value ADC0: ");
        DBGPORT.println(rightFront);
        DBGPORT.print("Pin: ");
        DBGPORT.print(distanceRR);
        DBGPORT.print(", value ADC1: ");
        DBGPORT.println(rightRear); 
    }
    
    // reads both front and rear sensors on the left side
    errorFlag = scn_readPair (distanceLF, distanceLR, &leftFront, &leftRear);
    if (~errorFlag)
    {
        telem.irLF = leftFront;
        telem.irLR = leftRear; 
    }
    
    if (IR_DEBUG == 1)
    {
        DBGPORT.print("Pin: ");
        DBGPORT.print(distanceLF);
        DBGPORT.print(", value ADC0: ");
        DBGPORT.println(leftFront);
        DBGPORT.print("Pin: ");
        DBGPORT.print(distanceLR);
        DBGPORT.print(", value ADC1: ");
        DBGPORT.println(leftRear); 
    }
  
    /* fail_flag contains all possible errors,
      They are defined in  ADC_Module.h as

      ADC_ERROR_OTHER
      ADC_ERROR_CALIB
      ADC_ERROR_WRONG_PIN
      ADC_ERROR_ANALOG_READ
      ADC_ERROR_COMPARISON
      ADC_ERROR_ANALOG_DIFF_READ
      ADC_ERROR_CONT
      ADC_ERROR_CONT_DIFF
      ADC_ERROR_WRONG_ADC
      ADC_ERROR_SYNCH

      You can compare the value of the flag with those masks to know what's the error.
    */
}

///////////////////////////////////////////////////////////////////////////////
// scn_readPair - reads a coupled pair of ADC channels, either the two left
// or two right IR sensors
///////////////////////////////////////////////////////////////////////////////
int scn_readPair (int sensor1, int sensor2, int* dist1, int* dist2)
{
        // reads both front and rear sensors at the same time
    if (adc->startSynchronizedSingleRead(sensor1, sensor2))
    {
        while (adc->isConverting(ADC_0) || adc->isConverting(ADC_1))
        {
            ;               // wait for conversions to complete
        }
        
        ADC::Sync_result result = adc->readSynchronizedSingle();
        
        float sensor1val = map(result.result_adc0, 0, adc->getMaxValue(ADC_0), 0, 5000);
        float sensor2val = map(result.result_adc1, 0, adc->getMaxValue(ADC_1), 0, 5000);
        *dist1 = 1.0 / (((sensor1val - 1125.0) / 1000.0) / 137.5);
        *dist2 = 1.0 / (((sensor2val - 1125.0) / 1000.0) / 137.5);
    }
    
    // Any error occur?
    int failFlag0 = adc->adc0->fail_flag;
    if (failFlag0) 
    {
        DBGPORT.print("ADC0 error flags: 0x");
        DBGPORT.println(failFlag0, HEX);
        if (failFlag0 == ADC_ERROR_COMPARISON) 
        {
            adc->adc0->fail_flag &= ~ADC_ERROR_COMPARISON; // clear that error
            DBGPORT.println("Comparison error in ADC0");
        }
    }
    
#   if ADC_NUM_ADCS>1
    int failFlag1 = adc->adc1->fail_flag;
    if (failFlag1) 
    {
        DBGPORT.print("ADC1 error flags: 0x");
        DBGPORT.println(failFlag1, HEX);
        if (failFlag1 == ADC_ERROR_COMPARISON) 
        {
            adc->adc1->fail_flag &= ~ADC_ERROR_COMPARISON; // clear that error
            DBGPORT.println("Comparison error in ADC1");
        }
    }
    
    return (failFlag0 | failFlag1);   
#   else
    return (failFlag0);  
#   endif

}

///////////////////////////////////////////////////////////////////////////////
// SCN_GETANGLE - get the current position of the scanner in hundredths of
// a degree.  
///////////////////////////////////////////////////////////////////////////////
int scn_getAngle (void)
{
    float   currStepCnt     = (float) scn_stepCntr;   
    float   numStepsPerRev  = (float) scn_uStepsPerRev / (float) scn_uStepsPerInput;
    float   currAngle       = currStepCnt * 360 /  numStepsPerRev;
    int     intCurrAngle    = (int) (currAngle * 100 + 0.5);
    return (intCurrAngle);
}

///////////////////////////////////////////////////////////////////////////////
// SCN_ENABLE - enable/start the scanner.  
///////////////////////////////////////////////////////////////////////////////
void scn_enable (void)
{
  digitalWrite(SCN_RESET,  1);      // dag - should this go down?
  digitalWrite(SCN_ENABLE, 0);
  controller.rotateAsync(motor);    
}

///////////////////////////////////////////////////////////////////////////////
// SCN_SETSPEED - set the rotational speed of the scanner.  
///////////////////////////////////////////////////////////////////////////////
void scn_setSpeed (int speed)
{
    motor.setMaxSpeed (speed);   
    scn_uStepsPerRev = speed;
}

///////////////////////////////////////////////////////////////////////////////
// SCN_ZEROISR - ISR for the zero pulse.  The zero pulse simply resets the 
// step counter (in the stepIsr).  
///////////////////////////////////////////////////////////////////////////////
void scn_zeroIsr (void)
{
    scn_zeroFlag = true;
    digitalWriteFast(ledPin, !digitalReadFast(ledPin));
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
    digitalWriteFast(TIME_STEP_PIN, 1);
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
    digitalWriteFast(TIME_STEP_PIN, 0);
}

///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////
