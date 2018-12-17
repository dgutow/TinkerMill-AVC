///////////////////////////////////////////////////////////////////////////////
// Vehicle.h 
///////////////////////////////////////////////////////////////////////////////

#ifndef VEHICLE_H
#define VEHICLE_H

///////////////////////////////////////////////////////////////////////////////
// Function templates 
///////////////////////////////////////////////////////////////////////////////

void veh_init();
uint32_t veh_move  (int16_t speed, uint16_t distance);
uint32_t veh_turn  (int16_t angle, uint16_t timeMsec);
uint32_t veh_brake (uint16_t onOff);
uint32_t veh_estop ();

void veh_check       (uint32_t currTimeMsec);
void veh_getSwitches (uint32_t currTimeMsec);
void veh_getTelem    (uint32_t currTimeMsec);

void veh_leftWheelInt  (void);
void veh_rightWheelInt (void);

#endif    //  HEARTBEAT_H
