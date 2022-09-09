#pragma once

const int directionPin = 5;
const int stepPin      = 9;
const int enablePin    = 7;
const int UART_TX      = 6;
const int UART_RX      = 4;

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

