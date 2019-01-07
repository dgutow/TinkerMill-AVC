///////////////////////////////////////////////////////////////////////////////
// telemetry.h
///////////////////////////////////////////////////////////////////////////////

#ifndef  TELEMETRY_H
#define TELEMETRY_H

#include "constants.h"
///////////////////////////////////////////////////////////////////////////////
// Enumerations 
///////////////////////////////////////////////////////////////////////////////
enum Mode
{
    BIST   = 0,         // Built in self-test mode
    NORMAL = 1,         // Normal, run mode
    ESTOP  = 2,         // Emergency stop mode
};

enum BistStatus
{
    BIST_SUCCESSFUL      = 0x00,     // BIST successful
    SCAN_CALIB_FAILED    = 0x01,     // unable to calibrate the scanner
    LT_WHEEL_ENC_FAILED  = 0x02,     // No pulses from left wheel
    RT_WHEEL_ENC_FAILED  = 0x04,     // No pulsed from right wheel   
    // etc...
};

enum EstopReason
{
    NONE         = 0,   // Not in estop mode
    COMMANDED    = 1,   // commanded by host 
    ESTOP_BUTTON = 2,   // estop button pushed
    BUMP_BUTTON  = 3,   // bumper switch activated
    HEARTBEAT    = 4,   // Missed heartbeat
};

///////////////////////////////////////////////////////////////////////////////
// Telemetry class
///////////////////////////////////////////////////////////////////////////////
class Telemetry
{
public:    
    uint32 pktId;             // Identifier for this packet
    uint32 time;
    uint16 currMode;          // Enumeration of Mode
    uint16 acceptCntr;
    uint16 bist;
    int16  currSpeed;
    int16  currSteerAng;
    uint16 cumDistance;
    int16  scnDist1;
    int16  scnDist2;
    int16  scnDist3;
    int16  scnDist4;
    uint16 switches;
    uint16 scanAng;
    uint16 rejectCntr;
    uint16 rejectReason;
    uint16 volt1;
    uint16 volt2;
    int16  accel;
    int16  gyro;
    int16  compass;
    int16  cameraAngle;
    uint16 brakeStatus;
    uint16 leftEncoder;    
    uint16 rightEncoder;
};
 
///////////////////////////////////////////////////////////////////////////////
// Function templates 
///////////////////////////////////////////////////////////////////////////////

void tlm_init();
void tlm_sendToHost (uint32 currTimeMsec);
void tlm_sendToEsp  (uint32 currTimeMsec);
 
#endif    //  TELEMETRY_H
