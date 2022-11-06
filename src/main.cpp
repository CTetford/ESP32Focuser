#include <Arduino.h>
//#include "LM335.h"
#include "Moonlite.h"
#include "StepperControl.h"
//#include <ESP32Encoder.h>

#include "../Configuration.hpp"

//const int temperatureSensorPin = 3;

unsigned long timestamp;

//LM335 TemperatureSensor(temperatureSensorPin);
StepperControl Motor(stepPin, directionPin, enablePin);
Moonlite SerialProtocol;
// ESP32Encoder encoder;

float temp = 0;
long pos = 0;

void processCommand()
{
  switch (SerialProtocol.getCommand().commandID)
  {
    case ML_C:
      // Initiate temperature convertion
      // Not implemented
      break;
    case ML_FG:
      // Goto target position
      Motor.goToTargetPosition();
      break;
    case ML_FQ:
      // Motor stop movement
      Motor.stopMovement();
      break;
    case ML_GB:
      // Set the Red Led backligth value
      // Dump value necessary to run the official moonlite software
      SerialProtocol.setAnswer(2, 0x00);
      break;
    case ML_GC:
      // Return the temperature coefficient
      SerialProtocol.setAnswer(2, (long)Motor.getTemperatureCompensationCoefficient());
      break;
    case ML_GD:
      // Return the current motor speed
      switch (Motor.getSpeed())
      {
        case Speed1:
          SerialProtocol.setAnswer(2, (long)0x20);
          break;
        case Speed2:
          SerialProtocol.setAnswer(2, (long)0x10);
          break;
        case Speed3:
          SerialProtocol.setAnswer(2, (long)0x8);
          break;
        case Speed4:
          SerialProtocol.setAnswer(2, (long)0x4);
          break;
        case Speed5:
          SerialProtocol.setAnswer(2, (long)0x2);
          break;
        default:
          SerialProtocol.setAnswer(2, (long)0x20);
          break;
      }
      break;
    case ML_GH:
      // Return the current stepping mode (half or full step)
      SerialProtocol.setAnswer(2, (long)(Motor.getStepMode() == HALF_STEP ? 0xFF : 0x00));
      break;
    case ML_GI:
      // get if the motor is moving or not
      SerialProtocol.setAnswer(2, (long)(Motor.isInMove() ? 0x01 : 0x00));
      break;
    case ML_GN:
      // Get the target position
      SerialProtocol.setAnswer(4, (long)(Motor.getTargetPosition()));
      break;
    case ML_GP:
      // Return the current position
      SerialProtocol.setAnswer(4, (long)(Motor.getCurrentPosition()));
      break;
    case ML_GT:
      // Return the temperature
      //SerialProtocol.setAnswer(4, (long)((TemperatureSensor.getTemperature() * 2)));
      SerialProtocol.setAnswer(4, (long)(20 * 2));
      break;
    case ML_GV:
      // Get the version of the firmware
      SerialProtocol.setAnswer(2, (long)(0x01));
      break;
    case ML_SC:
      // Set the temperature coefficient
      Motor.setTemperatureCompensationCoefficient(SerialProtocol.getCommand().parameter);
      break;
    case ML_SD:
      // Set the motor speed
      switch (SerialProtocol.getCommand().parameter)
      {
        case 0x02:
          Motor.setSpeed(Speed5);
          break;
        case 0x04:
          Motor.setSpeed(Speed4);
          break;
        case 0x08:
          Motor.setSpeed(Speed3);
          break;
        case 0x10:
          Motor.setSpeed(Speed2);
          break;
        case 0x20:
          Motor.setSpeed(Speed1);
          break;
        default:
          break;
      }
      break;
    case ML_SF:
      // Set the stepping mode to full step
      Motor.setStepMode(FULL_STEP);
      if (Motor.getSpeed() >= SC_MAX_SPEED_FULL_STEP)
      {
        Motor.setSpeed(SC_MAX_SPEED_FULL_STEP);
      }
      break;
    case ML_SH:
      // Set the stepping mode to half step
      Motor.setStepMode(HALF_STEP);
      break;
    case ML_SN:
      // Set the target position
      //encoder.setCount(SerialProtocol.getCommand().parameter * encoderMotorstepsRelation);
      Motor.setTargetPosition(SerialProtocol.getCommand().parameter);
      break;
    case ML_SP:
      // Set the current motor position
      //encoder.setCount(SerialProtocol.getCommand().parameter * encoderMotorstepsRelation);
      Motor.setCurrentPosition(SerialProtocol.getCommand().parameter);
      break;
    case ML_PLUS:
      // Activate temperature compensation focusing
      Motor.enableTemperatureCompensation();
      break;
    case ML_MINUS:
      // Disable temperature compensation focusing
      Motor.disableTemperatureCompensation();
      break;
    case ML_PO:
      // Temperature calibration
      //TemperatureSensor.setCompensationValue(SerialProtocol.getCommand().parameter / 2.0);
      break;
    default:
      break;
  }
}

/* 
void SetupEncoder()
{
  delay(1);
  // Enable the weak pull down resistors
	ESP32Encoder::useInternalWeakPullResistors=UP;
  // set starting count value
	encoder.clearCount();
  // Attach pins for use as encoder pins
	encoder.attachSingleEdge(encoderPin1, encoderPin2);
}
*/

void setup()
{
  SerialProtocol.init(9600);
  // Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  //delay(3000);
  //Serial.println("Begin debugging");

  // Open serial port with TMC2209
  #if BOARD == BEETLE_CM32U4
  Motor.initDriver(UART_RX, UART_TX, 0.11f, 0b00, RMS_CURRENT);
  #elif BOARD == BEETLE_ESP32C3
  Serial1.begin(115200, SERIAL_8N1, UART_TX, UART_RX);  
  Motor.initDriver(&Serial1, 0.11f, 0b00, RMS_CURRENT);    
  #endif

  // SetupEncoder();
}

/* 
void HandleHandController()
{
  long targetPosition = Motor.getTargetPosition();
  long encoderPosition = encoder.getCount() / encoderMotorstepsRelation;
  Motor.setTargetPosition(encoderPosition);  
  if(targetPosition != encoderPosition)
  {
    Motor.goToTargetPosition();
  }
  if (!Motor.isInMove())
  {
    Motor.goToTargetPosition();
  }
  while(Motor.isInMove())
  {
    Motor.Manage();
  }
}
*/

void loop()
{
   /*
  if (!Motor.isInMove())
  {
   
    TemperatureSensor.Manage();
    if (Motor.isTemperatureCompensationEnabled() && ((millis() - timestamp) > 30000))
    {
     // Motor.setCurrentTemperature(TemperatureSensor.getTemperature());
      Motor.setCurrentTemperature(20);
      Motor.compensateTemperature();
      timestamp = millis();
    }
  }
  */

  // HandleHandController();

  Motor.Manage();
  SerialProtocol.Manage();

  if (SerialProtocol.isNewCommandAvailable())
  {
    processCommand();
  }
}
