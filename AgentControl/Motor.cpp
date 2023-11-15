#include "Motor.h"

// Defines pins for the motor and if the control should be inverted (defaults to false).
// Generally, motors that face the same direction should have the same inverse value.
Motor::Motor(int speedPin, int directionPin, int hallEffectPin, bool inverse) {
  this->speedPin = speedPin;
  this->directionPin = directionPin;
  this->inverse = inverse;
  this->hallEffectPin = hallEffectPin;
  this->init();
}

void Motor::init() {
  pinMode(this->speedPin, OUTPUT);
  pinMode(this->directionPin, OUTPUT);
  if (this->hallEffectPin > 0)
  {
    pinMode(this->hallEffectPin, INPUT);
  }
  this->speed = 0;
  this->direction = false;
  this->lastHallReading = millis();
  this->prevHallState = false;
  this->currentRPM = 0;
  this->saturatedTimeStart = 0;
  this->prevDesiredRPM = 0;
  this->prevRPMTime = 0;
}

// Controls the speed and direction of the motor based on the input.
// Input should be between -255 and 255.
void Motor::drive(int speed) {
  this->observedSpeed = speed;

  // Sets direction based off the sign of the input.
  // Changes direction and speed according to motor inversion status.
  if (!this->inverse)
  {
    this->direction = speed <= 0;
    this->speed = min(abs(speed), 255);
  }
  else
  {
    this->direction = speed > 0;
    this->speed = min(abs(speed), 255);
  }
  if (this->speed >= 255 && this->saturatedTimeStart == 0)
  {
    this->saturatedTimeStart = millis();
  }
  else if (this->speed < 255)
  {
    this->saturatedTimeStart = 0;
  }
  
  digitalWrite(this->directionPin, this->direction);
  analogWrite(this->speedPin, this->speed);
}

void Motor::setRPM(float desiredRPM)
{
  if (desiredRPM == 0)
  {
    this->drive(0);
    return;
  }
  if (desiredRPM != this->prevDesiredRPM)
  {
    this->prevDesiredRPM = desiredRPM;
    this->prevRPMTime = millis();
    this->drive((desiredRPM / Motor::MaxRPM) * 255);
  }
  if (millis() - this->prevRPMTime > 250)
  {
    this->prevRPMTime = millis();
    float currRPM = this->getRPM() * ((this->inverse ^ this->direction) ? -1 : 1);//(this->speed < 0 ? -1 : 1);
    if (desiredRPM < currRPM)
    {
      this->drive(this->observedSpeed - 1);
  //    --this->speed;
    }
    else if (desiredRPM > currRPM)
    {
      this->drive(this->observedSpeed + 1);
  //    ++this->speed;
    }
  //  this->drive(this->speed);
  }
}

// Returns a value from -255 to 255, indicating speed and direction relative to robot, accounting for inversion.
// 255 is full speed forward, -255 is full speed reverse, and 0 is not moving.
int Motor::getObservedSpeed() {
  return observedSpeed;
}

// Returns the raw PWM data sent to the motor.
// Should be between 0 and 255.
int Motor::getRawSpeed() {
  return this->speed;
}

// Returns a boolean indicating the value sent to the direction pin of the motor.
bool Motor::getDirection() {
  return this->direction;
}

float Motor::getRPM()
{
  if (this->hallEffectPin > 0)
  {
    bool currHallState = digitalRead(this->hallEffectPin) == LOW;
    if (currHallState != this->prevHallState)
    {
      this->prevHallState = currHallState;
      if (currHallState)
      {
        float currentMPR = (millis() - this->lastHallReading) / 1000.0 * HallReadingsPerRev / 60.0; // minutes per rev
        this->lastHallReading = millis();
        if (currentMPR == 0)
          this->currentRPM = 0;
        else
        {
          this->currentRPM = 1.0 / currentMPR;
          // += and /= for averaging to get smoother readings
//          this->currentRPM += 1.0 / currentMPR;
//          this->currentRPM /= 2.0;
        }
      }
    }
    else if (millis() - this->lastHallReading >= 500)
    {
      this->currentRPM = 0;
    }
  }
  return this->currentRPM;
}

bool Motor::isSaturated()
{
  return this->saturatedTimeStart != 0 && millis() - this->saturatedTimeStart >= 500;
}

float Motor::returnRPM()
{
  return this->currentRPM;
}
