///////////////////////////////////////////////////////////////////////////////
// telemetry.h
///////////////////////////////////////////////////////////////////////////////

#ifndef  TELEMETRY_H
#define TELEMETRY_H

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
    uint32_t pktId;             // Identifier for this packet
    uint32_t time;
    uint16_t currMode;          // Enumeration of Mode
    uint16_t acceptCntr;
    uint16_t bist;
    int16_t  currSpeed;
    int16_t  currSteerAng;
    uint16_t cumDistance;
    uint16_t scnDist1;
    uint16_t scnDist2;
    uint16_t scnDist3;
    uint16_t scnDist4;
    uint16_t switches;
    uint16_t scanAng;
    uint16_t rejectCntr;
    uint16_t rejectReason;
    uint16_t volt1;
    uint16_t volt2;
    int16_t  accel;
    int16_t  gyro;
    int16_t  compass;
    int16_t  cameraAngle;
    uint16_t brakeStatus;
    uint16_t spare1;    
    uint16_t spare2;
};
 
///////////////////////////////////////////////////////////////////////////////
// Function templates 
///////////////////////////////////////////////////////////////////////////////

void tlm_init();
void tlm_sendToHost (uint32_t currTimeMsec);
void tlm_sendToEsp  (uint32_t currTimeMsec);
 
#endif    //  TELEMETRY_H