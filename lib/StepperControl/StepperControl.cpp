/*
StepperControl_A4988.h - - Driver for a Stepper Motor with controler A4988- Version 1.0

History:
Version 1.0 - Author Jean-Philippe Bonnet
    First release
Version 2.0 - Author Cameron Tetford
    Changed stepmode control to be UART-based for TMC2209 drivers

This file is part of the StepperControl_A4988 library.

StepperControl_A4988 library is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

StepperControl_A4988 library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with StepperControl library.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "StepperControl.h"
#include "../../Configuration.hpp"

//------------------------------------------------------------------------------------
// Constructors:
StepperControl::StepperControl(int stepPin, int directionPin, int enablePin)
{
  this->stepPin = stepPin;
  this->directionPin = directionPin;
  this->enablePin = enablePin;

  pinMode(stepPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  digitalWrite(directionPin, LOW);
  digitalWrite(stepPin, LOW);
  digitalWrite(enablePin, HIGH);

  #if INVERT_DIRECTION == 0
  this->direction = SC_CLOCKWISE;
  #elif INVERT_DIRECTION == 1
  this->direction = SC_COUNTER_CLOCKWISE;
  #endif
  this->inMove = false;
  this->startPosition = 0;
  this->currentPosition = 0;
  this->targetPosition = 0;
  this->moveMode = SC_MOVEMODE_SMOOTH;
  this->stepMode = FULL_STEP;
  this->acceleration = SC_DEFAULT_ACCEL;
  this->speed = Speed3;
  this->lastMovementTimestamp = 0;
  this->accelTimestamp = 0;
  this->targetSpeedReached = false;
  this->positionTargetSpeedReached = 0;
  this->lastCompensatedTemperature = 0;
  this->temperatureCompensationIsInit = false;
  this->temperatureCompensationIsEnabled = false;
  this->temperatureCompensationCoefficient = 0;
  this->currentTemperature = 0;
}

#if BOARD == BEETLE_CM32U4
void StepperControl::initDriver(int RX_PIN, int TX_PIN, float rsense, byte driveraddress, int rms_current)
{
  this->driver = new TMC2209Stepper(RX_PIN, TX_PIN, rsense, driveraddress);
  this->driver->begin();
  delay(1000);
  this->driver->rms_current(rms_current);        // Set motor RMS current
  this->driver->I_scale_analog(false);
  this->driver->microsteps(stepMode);
  this->driver->en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  this->driver->pwm_autoscale(true);     // Needed for stealthChop
  //Serial.println("driver initialized");
}
#elif BOARD == BEETLE_ESP32C3
void StepperControl::initDriver(HardwareSerial *serial, float rsense, byte driveraddress, int rms_current)
{
  this->driver = new TMC2209Stepper(serial, rsense, driveraddress);
  this->driver->rms_current(rms_current);        // Set motor RMS current
  this->driver->I_scale_analog(false);
  this->driver->microsteps(stepMode);
  this->driver->en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  this->driver->pwm_autoscale(true);     // Needed for stealthChop
}
#endif
//------------------------------------------------------------------------------------
// Setters
void StepperControl::setTargetPosition(long position)
{
  this->targetPosition = position;
}

void StepperControl::setCurrentPosition(long position)
{
  this->currentPosition = position;
}

void StepperControl::setDirection(int direction)
{
  this->direction = direction;
}

void StepperControl::setStepMode(int stepMode)
{
  this->stepMode = stepMode;
  this->driver->microsteps(stepMode);
}

void StepperControl::setMoveMode(int moveMode)
{
  this->moveMode = moveMode;
}

void StepperControl::setSpeed(unsigned int speed)
{
  if (this->stepMode == FULL_STEP && speed >= SC_MAX_SPEED_FULL_STEP)
  {
    this->targetSpeed = SC_MAX_SPEED_FULL_STEP;
  }
  else if (this->stepMode == HALF_STEP && speed >= SC_MAX_SPEED_HALF_STEP)
  {
    this->targetSpeed = SC_MAX_SPEED_HALF_STEP;
  }
  else
  {
    this->targetSpeed = speed;
  }
}

void StepperControl::setTemperatureCompensationCoefficient(int coef)
{
  this->temperatureCompensationCoefficient = coef;
}

void StepperControl::setCurrentTemperature(float curTemp)
{
  this->currentTemperature = curTemp;
  if (!this->temperatureCompensationIsInit)
  {
    this->lastCompensatedTemperature = this->currentTemperature;
    this->temperatureCompensationIsInit = true;
  }
}

//------------------------------------------------------------------------------------
// Getters
long StepperControl::getCurrentPosition()
{
  return this->currentPosition;
}

long StepperControl::getTargetPosition()
{
  return this->targetPosition;
}

int StepperControl::getDirection()
{
  return this->direction;
}

int StepperControl::getStepMode()
{
  return this->stepMode;
}

int StepperControl::getMoveMode()
{
  return this->moveMode;
}

unsigned int StepperControl::getSpeed()
{
  return this->speed;
}

int StepperControl::getTemperatureCompensationCoefficient()
{
  return this->temperatureCompensationCoefficient;
}

//------------------------------------------------------------------------------------
// Other public members
void StepperControl::Manage()
{
  if (this->inMove)
  {
    this->moveMotor();
  }
  else if (this->temperatureCompensationIsEnabled)
  {
  }
  else
  {
    // disable motor by timeout
    if((micros() - this->lastMovementTimestamp) >= 1000000)
    {
      digitalWrite(this->enablePin, HIGH);
    }
  }
}

void StepperControl::goToTargetPosition()
{
  if (this->currentPosition != this->targetPosition)
  {
    if (this->moveMode == SC_MOVEMODE_SMOOTH)
    {
      this->speed = 0;
      this->targetSpeedReached = false;
      this->positionTargetSpeedReached = 0;
    }
    else
    {
      this->speed = this->targetSpeed;
    }
    this->startPosition = this->currentPosition;
    digitalWrite(this->enablePin, LOW);
    this->inMove = true;
  }
}

void StepperControl::stopMovement()
{
  this->inMove = false;
  this->speed = 0;
  this->positionTargetSpeedReached = 0;
}

int StepperControl::isInMove()
{
  return this->inMove;
}

void StepperControl::compensateTemperature()
{
  long correction = 0;

  if (this->temperatureCompensationIsInit && !this->inMove)
  {
    correction = (long)(1.0 * (this->lastCompensatedTemperature - this->currentTemperature) * (float)this->temperatureCompensationCoefficient);

    if (correction)
    {
      this->lastCompensatedTemperature = this->currentTemperature;
      this->dbg_correction = this->getCurrentPosition() + (long)correction;
      this->setTargetPosition(this->getCurrentPosition() + (long)correction);
      this->goToTargetPosition();
    }
  }
}

//------------------------------------------------------------------------------------
// Privates
void StepperControl::moveMotor()
{
  if (this->moveMode == SC_MOVEMODE_SMOOTH)
  {
    this->calculateSpeed();
  }

  if ((this->targetPosition != this->currentPosition))
  {
    if ((this->speed != 0) && (micros() - this->lastMovementTimestamp) >= ((unsigned long)((1 / ((float)this->speed + 1)) * 1000000)))
    {
      if ((this->targetPosition - this->currentPosition) > 0)
      {
        if (this->direction == SC_CLOCKWISE)
        {
          digitalWrite(this->directionPin, LOW);
        }
        else
        {
          digitalWrite(this->directionPin, HIGH);
        }
        this->currentPosition++;
      }
      else
      {
        if (this->direction == SC_CLOCKWISE)
        {
          digitalWrite(this->directionPin, HIGH);
        }
        else
        {
          digitalWrite(this->directionPin, LOW);
        }
        this->currentPosition--;
      }
      digitalWrite(this->stepPin, HIGH);
      delayMicroseconds(1);
      digitalWrite(this->stepPin, LOW);
      delayMicroseconds(1);

      if (this->speed >= this->targetSpeed)
      {
        if (!this->targetSpeedReached)
        {
          this->positionTargetSpeedReached = this->startPosition - this->currentPosition;
          if (this->positionTargetSpeedReached < 0)
          {
            this->positionTargetSpeedReached *= -1;
          }
        }
        this->speed = this->targetSpeed;
        this->targetSpeedReached = true;
      }
      this->lastMovementTimestamp = micros();
    }
  }
  else
  {
    this->stopMovement();
  }
}

void StepperControl::calculateSpeed()
{
  if ((millis() - this->accelTimestamp) >= 50)
  {
    long midway = (this->targetPosition - this->startPosition);
    // avoid miday == 0 in case of movement of only one step
    if (abs(midway) == 1)
    {
      midway *= 2;
    }

    midway /= 2;

    if (midway > 0)
    {
      midway += this->startPosition;
      if (!this->targetSpeedReached && (this->currentPosition < midway))
      {
        this->speed += this->acceleration;
      }
      else
      {
        if ((!this->targetSpeedReached && (this->currentPosition > midway)) || (this->currentPosition >= (this->targetPosition - this->positionTargetSpeedReached)))
        {

          if ((this->targetPosition != this->currentPosition) && (this->speed > this->acceleration))
          {
            this->speed -= this->acceleration;
          }
          else
          {
            this->speed = this->acceleration;
          }
        }
      }
    }
    else
    {
      midway = this->startPosition - -1 * midway;
      if (!this->targetSpeedReached && (this->currentPosition > midway))
      {
        this->speed += this->acceleration;
      }
      else
      {
        if ((!this->targetSpeedReached && (this->currentPosition < midway)) || (this->currentPosition <= (this->positionTargetSpeedReached + this->targetPosition)))
        {

          if ((this->targetPosition != this->currentPosition) && (this->speed > this->acceleration))
          {
            this->speed -= this->acceleration;
          }
          else
          {
            this->speed = this->acceleration;
          }
        }
      }
    }
    this->accelTimestamp = millis();
  }
}

bool StepperControl::isTemperatureCompensationEnabled()
{
  return this->temperatureCompensationIsEnabled;
}

void StepperControl::enableTemperatureCompensation()
{
  this->temperatureCompensationIsInit = false;
  this->temperatureCompensationIsEnabled = true;
}

void StepperControl::disableTemperatureCompensation()
{
  this->temperatureCompensationIsEnabled = false;
}

bool StepperControl::connectToDriver()
{
    //Serial.println("[STEPPERS]: Testing UART Connection to driver...");
    for (int i = 0; i < 3; i++)
    {
        if (this->driver->test_connection() == 0)
        {
            //Serial.println("[STEPPERS]: UART connection to driver successful.");
            return true;
        }
        else
        {
            delay(500);
        }
    }
    //Serial.println("[STEPPERS]: UART connection to driver failed.");
    return false;
}