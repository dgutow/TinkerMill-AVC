///////////////////////////////////////////////////////////////////////////////
// telemetry - Functions associated with the telemetry structure.  
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Local header files
///////////////////////////////////////////////////////////////////////////////
#include "telemetry.h"
#include "constants.h"

///////////////////////////////////////////////////////////////////////////////
// extern global variables - defined in the main file
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// local global variables
///////////////////////////////////////////////////////////////////////////////
Telemetry telem;            // The telemetry class

int32_t  nBytesSent;       // Number of bytes sent to host

///////////////////////////////////////////////////////////////////////////////
// tlm_init - initialize the telemetry system
///////////////////////////////////////////////////////////////////////////////
void tlm_init ()
{
   // Initialize the serial ports 
   RPIPORT.begin(115200);
   DBGPORT.begin(115200);
   
#  ifdef TEENSIE_35    
    ESPPORT.begin(115200);
#  endif   
   
   // initialize the structure
   telem.pktId          = 0x11111111;
   telem.time           = millis();
   telem.currMode       = BIST;
   telem.acceptCntr     = 0;
   telem.bist           = BIST_SUCCESSFUL;
   telem.currSpeed      = 0;
   telem.currSteerAng   = 0;
   telem.cumDistance    = 0;
   telem.irLF           = 0;
   telem.irLR           = 0;
   telem.irRF           = 0;
   telem.irRR           = 0;
   telem.switches       = 0;
   telem.sensorAng      = 0;
   telem.sensor         = 0;
   telem.sensorDist     = 0;
   telem.volt1          = 0;
   telem.volt2          = 0;
   telem.accel          = 0;
   telem.gyro           = 0;
   telem.compass        = 0;
   telem.cameraAngle    = 0;
   telem.brakeStatus    = 0;
   telem.spare1         = 0; 
   telem.spare2         = 0;    
}

///////////////////////////////////////////////////////////////////////////////
// tlm_sendToHost - sends the telemetry structure to the Rpi as a binary message
///////////////////////////////////////////////////////////////////////////////
void tlm_sendToHost (uint32_t currTime)
{
  telem.pktId   = 0x11111111;
  telem.time    = currTime;
  RPIPORT.write ((byte*) &telem, sizeof(telem));

  DBGPORT.print("Time: ");         DBGPORT.print(currTime); 
  DBGPORT.print(",\tMode: ");      DBGPORT.print(telem.currMode);
  DBGPORT.print(",\tCmd Cnt: ");   DBGPORT.println(telem.acceptCntr);
}

///////////////////////////////////////////////////////////////////////////////
// tlm_sendToEsp - sends the telemetry structure to the esp as a text message
///////////////////////////////////////////////////////////////////////////////
void tlm_sendToEsp (uint32_t currTime)
{
#ifdef TEENSIE_35    
  ESPPORT.print   ("Bytes Sent: ");          ESPPORT.println (nBytesSent);
  ESPPORT.print   ("Current Time: ");        ESPPORT.println(telem.time);
  ESPPORT.print   ("Mode: ");                ESPPORT.println(telem.currMode);
  ESPPORT.print   ("Accept Count: ");        ESPPORT.println(telem.acceptCntr);
  ESPPORT.print   ("Lft Front Distance: ");  ESPPORT.println(telem.irLF);
  ESPPORT.print   ("Lft Rear Distance: ");   ESPPORT.println(telem.irLR);
  ESPPORT.print   ("Rght Front Distance: "); ESPPORT.println(telem.irRF);
  ESPPORT.print   ("Rght Rear Distance: ");  ESPPORT.println(telem.irRR);  
#endif  
}
