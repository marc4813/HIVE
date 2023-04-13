#include "Motor.h"

// Defines pins for the motor and if the control should be inverted (defaults to false).
// Generally, motors that face the same direction should have the same inverse value.
Motor::Motor(int speedPin, int directionPin, bool inverse) {
  this->speedPin = speedPin;
  this->directionPin = directionPin;
  this->inverse = inverse;
  this->init();
}

void Motor::init() {
  pinMode(this->speedPin, OUTPUT);
  pinMode(this->directionPin, OUTPUT);
  this->speed = 0;
  this->direction = false;
}

// Controls the speed and direction of the motor based on the input.
// Input should be between -255 and 255.
void Motor::drive(int speed) {
  this->observedSpeed = speed;

  // Sets direction based off the sign of the input.
  // Changes direction and speed according to motor inversion status.
  if (!this->inverse) {
    this->direction = speed <= 0;
    this->speed = constrain(speed > 0 ? speed : 255 + speed, 0, 255);
  } else {
    this->direction = speed > 0;
    this->speed = constrain(speed > 0 ? 255 - speed : -speed, 0, 255);
  }
  
  digitalWrite(this->directionPin, this->direction);
  analogWrite(this->speedPin, this->speed);
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
