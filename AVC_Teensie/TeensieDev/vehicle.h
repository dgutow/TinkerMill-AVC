///////////////////////////////////////////////////////////////////////////////
// Vehicle.h 
///////////////////////////////////////////////////////////////////////////////

#ifndef VEHICLE_H
#define VEHICLE_H

///////////////////////////////////////////////////////////////////////////////
// Function templates 
///////////////////////////////////////////////////////////////////////////////

void   veh_init();
uint32 veh_move  (int16 speed, uint16 distance);
uint32 vehurn  (int16 angle, uint16 timeMsec);
uint32 veh_brake (uint16 onOff);
uint32 veh_estop ();

void   veh_check       (uint32 currTimeMsec);
void   veh_getSwitches (uint32 currTimeMsec);
void   veh_getTelem    (uint32 currTimeMsec);

void   veh_leftWheelInt  (void);
void   veh_rightWheelInt (void);

#endif    //  HEARTBEAT_H
