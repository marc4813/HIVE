#include "Mecanum.h"

// Defines the drivetrain by all four motors
Mecanum::Mecanum(Motor &m1, Motor &m2, Motor &m3, Motor &m4): m1(m1), m2(m2), m3(m3), m4(m4) {
  this->m1 = m1; // Front left motor
  this->m2 = m2; // Front right motor
  this->m3 = m3; // Back left motor
  this->m4 = m4; // Back right motor
  this->init();
}

void Mecanum::init() {
  // x, y, and z should be between -1.0 and 1.0
  this->x = 0; // Forward/reverse
  this->y = 0; // Strafing
  this->z = 0; // Turning
}

// Controls each motor according to the command velocity inputs.
// Inputs should be between -1.0 and 1.0.
void Mecanum::drive(float x, float y, float z) {
  this->x = x;
  this->y = y;
  this->z = z;

  // Calculates desired velocity of each motor based on all inputs
  float m1v = x - y - z;
  float m2v = x + y + z;
  float m3v = x + y - z;
  float m4v = x - y + z;

  // scales all values to be between -1.0 and 1.0
  scaleMotors(&m1v, &m2v, &m3v, &m4v);

//  float maxRPM = this->getMaxRPM();
//  this->m1.setRPM(m1v * maxRPM);
//  this->m2.setRPM(m2v * maxRPM);
//  this->m3.setRPM(m3v * maxRPM);
//  this->m4.setRPM(m4v * maxRPM);

  // Sends speed commands to each motor.
  // Input should be between -255 and 255.
  this->m1.drive(m1v);
  this->m2.drive(m2v);
  this->m3.drive(m3v);
  this->m4.drive(m4v);
}

// Stops all motors
void Mecanum::stop()
{
  this->drive(0, 0, 0);
}

void Mecanum::update()
{
  this->drive(this->x, this->y, this->z);
}

// Takes calculated velocities of each motor and scales so that all are between -1.0 and 1.0.
// Then multiply by 255 for PWM use later.
void Mecanum::scaleMotors(float* m1v, float* m2v, float* m3v, float* m4v) {
  float max = 1;
  if (abs(*m1v) > max) max = abs(*m1v);
  if (abs(*m2v) > max) max = abs(*m2v);
  if (abs(*m3v) > max) max = abs(*m3v);
  if (abs(*m4v) > max) max = abs(*m4v);
  
//  *m1v /= max;
//  *m2v /= max;
//  *m3v /= max;
//  *m4v /= max;

  
  *m1v *= 255.0 / max;
  *m2v *= 255.0 / max;
  *m3v *= 255.0 / max;
  *m4v *= 255.0 / max;
}

float Mecanum::getMaxRPM()
{
  float maxRPM = Motor::MaxRPM; // must not be static
  float tempRPM;

  if (this->m1.isSaturated() && (tempRPM = this->m1.getRPM()) < maxRPM && tempRPM > 0.05 * Motor::MaxRPM)
  {
    maxRPM = tempRPM;
  }
  if (this->m2.isSaturated() && (tempRPM = this->m2.getRPM()) < maxRPM && tempRPM > 0.05 * Motor::MaxRPM)
  {
    maxRPM = tempRPM;
  }
  if (this->m3.isSaturated() && (tempRPM = this->m3.getRPM()) < maxRPM && tempRPM > 0.05 * Motor::MaxRPM)
  {
    maxRPM = tempRPM;
  }
  if (this->m4.isSaturated() && (tempRPM = this->m4.getRPM()) < maxRPM && tempRPM > 0.05 * Motor::MaxRPM)
  {
    maxRPM = tempRPM;
  }
  return maxRPM;
}

// Returns x velocity that was sent to the drivetrain
float Mecanum::getXVel() {
  return this->x;
}

// Returns y velocity that was sent to the drivetrain
float Mecanum::getYVel() {
  return this->y;
}

// Returns z velocity that was sent to the drivetrain
float Mecanum::getZVel() {
  return this->z;
}
