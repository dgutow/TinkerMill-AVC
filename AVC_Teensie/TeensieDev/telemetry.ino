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
   
   // initialize the structure
   telem.pktId          = 0x44454144;
   telem.time           = millis();
   telem.currMode       = BIST;
   telem.acceptCntr     = 0;
   telem.bist           = BIST_SUCCESSFUL;
   telem.currSpeed      = 0;        // cm/sec
   telem.currSteerAng   = 0;        // degress (- is left, + is right)
   telem.cumDistance    = 0;        // cm
   telem.scnDist1       = 0;
   telem.scnDist2       = 0;
   telem.scnDist3       = 0;
   telem.scnDist4       = 0;
   telem.switches       = 0;
   telem.scanAng        = 0;
   telem.rejectCntr     = 0;
   telem.rejectReason   = 0;
   telem.volt1          = 0;
   telem.volt2          = 0;
   telem.accel          = 0;
   telem.gyro           = 0;
   telem.compass        = 0;
   telem.cameraAngle    = 0;
   telem.brakeStatus    = 0;
   telem.leftEncoder    = 0; 
   telem.rightEncoder   = 0;    
}

///////////////////////////////////////////////////////////////////////////////
// tlm_sendToHost - sends the telemetry structure to the Rpi as a binary message
///////////////////////////////////////////////////////////////////////////////
void tlm_sendToHost (uint32_t currTime)
{
  telem.pktId   = 0x44454144;
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
    ;
}
