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

#ifdef TEENSIE_35
#define SCN_SENSOR1     A15     // Pin for sensor 0 (~0 degrees)
#define SCN_SENSOR2     A12     // Pin for sensor 1 (~90 degrees)
#define SCN_SENSOR3     A14     // Pin for sensor 2 (~180 degrees)
#define SCN_SENSOR4     A13     // Pin for sensor 3 (~270 degrees)
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
    int sensor1   = 0;
    int sensor2   = 0;
    int sensor3   = 0;
    int sensor4   = 0;
    
    // reads both front and rear sensors on the right side
    errorFlag = sid_readPair (SCN_SENSOR1, SCN_SENSOR2, &sensor1, &sensor2);
    if (~errorFlag)
    {
        telem.scnDist1 = sensor1;
        telem.scnDist2 = sensor2; 
    }
    
    if (IR_DEBUG == 1)
    {
        DBGPORT.print("Pin: ");
        DBGPORT.print(sensor1);
        DBGPORT.print(", value ADC0: ");
        DBGPORT.println(sensor1);
        DBGPORT.print("Pin: ");
        DBGPORT.print(sensor1);
        DBGPORT.print(", value ADC1: ");
        DBGPORT.println(sensor2); 
    }
    
    // reads both front and rear sensors on the left side
    errorFlag = sid_readPair (SCN_SENSOR3, SCN_SENSOR4, &sensor3, &sensor4);
    if (~errorFlag)
    {
        telem.scnDist1 = sensor3;
        telem.scnDist2 = sensor4; 
    }
    
    if (IR_DEBUG == 1)
    {
        DBGPORT.print("Pin: ");
        DBGPORT.print(sensor3);
        DBGPORT.print(", value ADC0: ");
        DBGPORT.println(sensor3);
        DBGPORT.print("Pin: ");
        DBGPORT.print(sensor4);
        DBGPORT.print(", value ADC1: ");
        DBGPORT.println(sensor4); 
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
