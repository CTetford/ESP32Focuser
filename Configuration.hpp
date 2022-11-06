#pragma once

#define BEETLE_CM32U4 0001
#define BEETLE_ESP32C3 0002

//#define BOARD BEETLE_ESP32C3 
#define BOARD BEETLE_CM32U4

const int directionPin = 14;
const int stepPin      = 3;
const int enablePin    = 11;
const int UART_TX      = 15;
const int UART_RX      = 16;

//const int encoderPin1  = 2;
//const int encoderPin2  = 15;
//const int encoderMotorstepsRelation = 5;

#define HALF_STEP 4 // Microstep setting to use for Moonlight "half" step configuration
#define FULL_STEP 2 // Microstep setting to use for Moonlight "full" step configuration


#define PULLEY_RATIO 80.0f / 12.0f //Pulley teeth ratio
#define MOTOR_SPR 200.0f // Steps-per-revolution of stepper motor
#define FOCUSER_SPR PULLEY_RATIO * MOTOR_SPR

#define Speed1 static_cast<int>(FOCUSER_SPR * HALF_STEP * 1.0f / 60.0f) //Steps per second - input as RPM / 60.0f
#define Speed2 static_cast<int>(FOCUSER_SPR * HALF_STEP * 2.0f / 60.0f) //Steps per second - input as RPM / 60.0f
#define Speed3 static_cast<int>(FOCUSER_SPR * HALF_STEP * 4.0f / 60.0f) //Steps per second - input as RPM / 60.0f
#define Speed4 static_cast<int>(FOCUSER_SPR * HALF_STEP * 8.0f / 60.0f) //Steps per second - input as RPM / 60.0f
#define Speed5 static_cast<int>(FOCUSER_SPR * HALF_STEP * 16.0f / 60.0f) //Steps per second - input as RPM / 60.0f
#define SC_DEFAULT_ACCEL 50 //Steps per second ^2
#define SC_MAX_SPEED_FULL_STEP Speed4
#define SC_MAX_SPEED_HALF_STEP Speed5

#define INVERT_DIRECTION 1 // Default is 0. Set to 1 to invert direction.
#define RMS_CURRENT 2000 // current setting of the stepper

/*
NOTE from Celestron:
The Celestron 8-9.25-11-14 focusers all use same 0.75mm thread pitch for the focus shaft.
The focus motor moves in 1000 steps per motor revolution
Each motor revolution moves the mirror 750 microns.
Thus each step on the motor movement moved the mirror .75 micron."

Critical focus zone CFZ = f-ratio^2 * 2.2
11" Hyperstar - 7.942 micrometers
11" F/10 - 220 micrometers
Sharpstar 61 EDPHII - 44.55 micrometers

11" focuser: 750 microns/rev / (PULLEY_RATIO * MOTOR_SPR * MICROSTEP) = 0.5625 / MICROSTEP

*/
