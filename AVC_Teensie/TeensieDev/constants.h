///////////////////////////////////////////////////////////////////////////////
// constants.h
///////////////////////////////////////////////////////////////////////////////

#ifndef CONSTANTS_H
#define CONSTANTS_H

///////////////////////////////////////////////////////////////////////////////
// TEENSIE_35 - define this only if compiling for the teensie 3.5
///////////////////////////////////////////////////////////////////////////////
#define TEENSIE_35


///////////////////////////////////////////////////////////////////////////////
// With or without the '_t'
///////////////////////////////////////////////////////////////////////////////
#define uint32  uint32_t
#define int32   int32_t
#define uint16  uint16_t
#define int16   int16_t
#define uint8   uint8_t
#define int8    int8_t

///////////////////////////////////////////////////////////////////////////////
// The serial ports
///////////////////////////////////////////////////////////////////////////////
#define RPIPORT   Serial         // Serial port connected to the Raspberry Pi 
#define DBGPORT   Serial2        // Serial port for textual debugging
#define TFPORT    Serial1        // Serial port connected to the TFMini

///////////////////////////////////////////////////////////////////////////////
// Pin assignments
///////////////////////////////////////////////////////////////////////////////
#define MOTOR_BATT_PIN_NUMBER       A8  // batteries supplying drive motor
#define ELECT_BATT_PIN_NUMBER       A9  // batteries supplying the electronics

//const int ENA       = 10;    //connected to Arduino's port 5(output pwm)
//const int IN1       = 9;     //connected to Arduino's port 10
//const int IN2       = 8;     // reset the stepper driver
//
const int longFront       = A0;    // Analog input - scanner long range sensor
const int shortFront      = A1;    // Analog input - scanner short range sensor

#ifdef TEENSIE_35
    const int TEENSIE_LED   = 13;   // LED on the board. teensie 3.2 - led 13
#else
    const int TEENSIE_LED   = 6;    // LED on the board. teensie 2++ - led 6
#endif

const int ENABLE_WIFI = 7;          // Enables the WIFI on the 8266
const int lrsr_Sel = 8;
const int Run = 6;
const int l_Wheel = 11;
const int r_Wheel = 12;


#endif    // CONSTANTS_H


