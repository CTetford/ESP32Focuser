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

const int HALF_STEP = 256; // Microstep setting to use for Moonlight "half" step configuration
const int FULL_STEP = 128; // Microstep setting to use for Moonlight "full" step configuration

#define SC_MAX_SPEED_FULL_STEP 50000
#define SC_MAX_SPEED_HALF_STEP 100000

const int Speed1 = 500;
const int Speed2 = 1000;
const int Speed3 = 3000;
const int Speed4 = 5000;
const int Speed5 = 7000;

#define INVERT_DIRECTION 1 // Default is 0. Set to 1 to invert direction.
#define RA_MOTOR_CURRENT 200 // current setting of the stepper
#define RMS_CURRENT  RA_MOTOR_CURRENT / 1.414f // rms_current calculated from peak current setting