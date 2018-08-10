///////////////////////////////////////////////////////////////////////////////
// Main file for the AVC racer teensie control program
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Local header files
///////////////////////////////////////////////////////////////////////////////
#include "taskScheduler.h"
#include "constants.h"
#include "telemetry.h"
#include "commands.h"
#include "heartbeat.h"
#include "status.h"
#include "vehicle.h"
#include "camera.h"

#ifdef TEENSIE_35
#include "scanner.h"
//	#include "sideSensors.h"
//	#include "nineDof.h"
#endif

//#include "bling.h"

///////////////////////////////////////////////////////////////////////////////
// Original Libraries/System header files
///////////////////////////////////////////////////////////////////////////////
#ifdef TEENSIE_35
    #include <Wire.h>
    #include <Metro.h>
    #include <Servo.h>
    #include <ADC.h>
    #include <Time.h>
    #include <AccelStepper.h>
#endif

///////////////////////////////////////////////////////////////////////////////
// Global variables
///////////////////////////////////////////////////////////////////////////////
uint32_t currTimeMsec;        // Milliseconds since we started up

extern   Telemetry telem;     // instantiated in telemetry.ino

#ifdef TEENSIE_35
    ADC *adc = new ADC();         // the analog to digital converter class
    ADC::Sync_result result;
    ADC::Sync_result result1;
#endif

///////////////////////////////////////////////////////////////////////////////
// Function templates for the taskList
///////////////////////////////////////////////////////////////////////////////
void checkForCmds    (uint32_t currTimeMsec);
void sendTelemToEsp  (uint32_t currTimeMsec);   
void sendTelemToHost (uint32_t currTimeMsec); 
void blinkLed        (uint32_t currTimeMsec);

///////////////////////////////////////////////////////////////////////////////
// taskList - An array of 'task's that are initialized to the list of functions
// to run and the interval to run them.
// - The first value (func) is the name of the function to run.  It must be
//   the name of a function which returns void and takes a uint32_t parameter
//   (the current time in millseconds)
// - The second value (timeDelta) is the time between executions of the function.
//   timeDelta is given in milliseconds. 
// - The third value (timeOffset) is the offset from the timeDelta boundary that
//   this function will run.  For example, a function with a timeDelta of 100
//   (msecs) and a timeOffset of 10 will run at times of 110, 210, 310, etc.
// - The fourth value (nextExecTime) is for internal use and should be set to
//   0.  (It is overwritten in the initExecuteTasks function)
///////////////////////////////////////////////////////////////////////////////
task taskList[] = { {checkForCmds,        20,     0,      0}, // did we get a new command?
                    {veh_check,           50,     0,      0}, // continual servicing of vehicle systems
                    {scn_getValues,       40,     0,      0},   
                    {cam_getTelem,        40,     0,      0},                     
                    {veh_getTelem,        40,     0,      0},                                                  
                    {tlm_sendToHost,      40,     1,      0},
                    //{hb_check,          10,     1,      0}, // check// get the side sensor values                      
                    //{blinkLed,           500,     1,      0},                    
                    }; 

#if 0
task taskList[] = { {checkForCmds,      10,     0,      0}, // did we get a new command?
                    {veh_check,         50,     0,      0}, // continual servicing of vehicle systems
                    {hb_check,          10,     1,      0}, // check if heartbeat failure
                    {sid_getValues,     25,     0,      0}, // get the side sensor values
                    {scn_getValues,     25,     0,      0}, // get the scanner values
                    {nin_getValues,     25,     0,      0}, // get the nine DOF values
                    {sts_getValues,     25,     0,      0}, // get the battery status's                                      
                    {tlm_sendToHost,    25,     0,      0},
                    {tlm_sendToEsp,     1000,   0,      0},
                    {blinkLed,          2000,   1,      0},                    
                    };
#endif


///////////////////////////////////////////////////////////////////////////////
// initialization
///////////////////////////////////////////////////////////////////////////////

void setup()
{
  
   //DBGPORT.begin(115200);
   //RPIPORT.begin(115200);
   
   tlm_init ();              // Initialize the serial ports - must be done first!
   veh_init();
   cam_init ();
   scn_init();      

   //adc_init ();            // initialize the ADC system   
   //scn_init ();            // Initialize the scanner
   //sts_init ();            // initialize the battery status's   
   //nin_init ();            // initialize the  nine DOF
   
   initExecuteTasks ();
    
   // Original setup code - Connect all the devices to their respective pins
   pinMode(ENA,OUTPUT);//output
   pinMode(IN1,OUTPUT);
   pinMode(IN2,OUTPUT);

   analogWrite (ENA,0);
   digitalWrite(IN1,LOW); 
   digitalWrite(IN2,HIGH);//setting motorA's directon
   
   // Enable the LED for blinking
   pinMode(TEENSIE_LED, OUTPUT);
   digitalWrite(TEENSIE_LED, 1);
   
   // Enable the wifi access pt on the 8266
   // pinMode (ENABLE_WIFI, OUTPUT);
   // digitalWrite(ENABLE_WIFI, 1); 
   
   // Perform any BIST tests
   
   delay(5000);                        // Original was 5000
}

///////////////////////////////////////////////////////////////////////////////
// The main loop
///////////////////////////////////////////////////////////////////////////////

void loop()
{                 
   currTimeMsec = millis();
   executeTasks (currTimeMsec);
}

///////////////////////////////////////////////////////////////////////////////
// uint_swap - swaps the bytes.
///////////////////////////////////////////////////////////////////////////////
unsigned int uint_swap(unsigned int val)
{
  unsigned int retval = (val >> 8) | (val << 8);
  return (retval);
}

///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// blinkLed - blink the onboard TEENSIE_LED
///////////////////////////////////////////////////////////////////////////////
void blinkLed (uint32_t currTimeMsec)
{
  static int blinkVal = 0;
      
  if (blinkVal == 0)
  {
    digitalWrite(TEENSIE_LED, HIGH);
    blinkVal = 1;
  }
  else
  {
    digitalWrite(TEENSIE_LED, LOW);
    blinkVal = 0;
  }
}

///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////
