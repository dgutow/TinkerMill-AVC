///////////////////////////////////////////////////////////////////////////////
// sideSensors - Functions associated with the side IR sensors.  
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Header files
///////////////////////////////////////////////////////////////////////////////
#include <ADC.h>
#include "telemetry.h"
#include "sideSensors.h"

///////////////////////////////////////////////////////////////////////////////
// extern global variables - defined in the main file
///////////////////////////////////////////////////////////////////////////////
extern Telemetry telem;                // The telemetry class

///////////////////////////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////////////////////////
#define CAM_PIN_NUMBER        39
#define CAM_CONVERSION       -15    // Convert from angle to microseconds
#define CAM_MID_PULSEWIDTH  1500    // Servo neutral position
#define CAM_MIN_PULSWIDTH   1000
#define CAM_MAX_PULSEWIDTH  2000

#if 0
// Defined in constants.h
const int distanceRF = A15;
const int distanceRR = A12;
const int distanceLF = A14;
const int distanceLR = A13;
#endif

const int IR_DEBUG = 0;

///////////////////////////////////////////////////////////////////////////////
// local global variables
///////////////////////////////////////////////////////////////////////////////
extern ADC *adc;
extern ADC::Sync_result result;
extern ADC::Sync_result result1;

float valued1;
float valued2;
float valued3;
float valued4;

///////////////////////////////////////////////////////////////////////////////
// sid_init - initialize the side sensor ADCs
///////////////////////////////////////////////////////////////////////////////
void sid_init ()
{
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_0);
  adc->setReference(ADC_REFERENCE::REF_3V3, ADC_1);
  adc->setResolution(12, ADC_0);
  adc->setResolution(12, ADC_1);
  adc->setAveraging (16, ADC_0);
  adc->setAveraging (16, ADC_1);
}

///////////////////////////////////////////////////////////////////////////////
// sid_getValues - gets the current camera angle and places it into telemetry
///////////////////////////////////////////////////////////////////////////////
void sid_getValues (uint32_t currTime)
{
    int errorFlag = 0;
    int rightFront= 0;
    int rightRear = 0;
    int leftFront = 0;
    int leftRear  = 0;
    
    // reads both front and rear sensors on the right side
    errorFlag = sid_readPair (distanceRF, distanceRR, &rightFront, &rightRear);
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
    errorFlag = sid_readPair (distanceLF, distanceLR, &leftFront, &leftRear);
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
// sid_readPair - reads a coupled pair of ADC channels, either the two left
// or two right IR sensors
///////////////////////////////////////////////////////////////////////////////
int sid_readPair (int frontSensor, int rearSensor, int* frontDist, int* rearDist)
{
        // reads both front and rear sensors at the same time
    if (adc->startSynchronizedSingleRead(frontSensor, rearSensor))
    {
        while (adc->isConverting(ADC_0) || adc->isConverting(ADC_1))
        {
            ;               // wait for conversions to complete
        }
        
        ADC::Sync_result result = adc->readSynchronizedSingle();
        
        float frontValue = map(result.result_adc0, 0, adc->getMaxValue(ADC_0), 0, 5000);
        float rearValue  = map(result.result_adc1, 0, adc->getMaxValue(ADC_1), 0, 5000);
        *frontDist = 1.0 / (((frontValue - 1125.0) / 1000.0) / 137.5);
        *rearDist  = 1.0 / (((rearValue  - 1125.0) / 1000.0) / 137.5);
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
// adc0_isr - If you enable interrupts make sure to call readSingle() to clear 
// the interrupt.
///////////////////////////////////////////////////////////////////////////////
void adc0_isr() 
{
  adc->adc0->readSingle();
}
