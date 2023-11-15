#include "MinIMU9.h"

MinIMU9::MinIMU9()
{
//  init();
}

bool MinIMU9::init()
{
  bool results[2];
  results[0] = this->imu.init();
  results[1] = this->mag.init();
  if (results[0])
  {
    this->imu.enableDefault();
    this->calculateIMUError();
  }
  if (results[1])
  {
    this->mag.enableDefault();
  }
  this->gyro_time[0] = this->gyro_time[1] = this->gyro_time[2] = millis();
  this->isOpened = results[0] && results[1];
  return this->isOpened;
}

bool MinIMU9::isOpen()
{
  return this->isOpened;
}

void MinIMU9::readAcc()
{
  this->imu.readAcc();
  // values in g
  this->a.x = (this->imu.a.x - this->accelErrors[0]) * 0.061 / 1000.0;
  this->a.y = (this->imu.a.y - this->accelErrors[1]) * 0.061 / 1000.0;
  this->a.z = (this->imu.a.z - this->accelErrors[2]) * 0.061 / 1000.0;
}

void MinIMU9::readGyro()
{
  this->imu.readGyro();

  // calculating z angle immediately after getting gyro readings because z is typically more important
  this->g.z = (this->imu.g.z - this->gyroErrors[2]) * 8.75 / 1000.0; // degrees per second
//  this->angles.z += g.z * (millis() - this->gyro_time[2]) / 1000.0; // degrees
//  this->gyro_time[2] = millis();

  this->g.y = (this->imu.g.y - this->gyroErrors[1]) * 8.75 / 1000.0; // degrees per second
//  this->angles.y += this->g.y * (millis() - this->gyro_time[1]) / 1000.0; // degrees
//  this->gyro_time[1] = millis();
  
  this->g.x = (this->imu.g.x - this->gyroErrors[0]) * 8.75 / 1000.0; // degrees per second
//  this->angles.x += this->g.x * (millis() - this->gyro_time[0]) / 1000.0; // degrees
//  this->gyro_time[0] = millis();
}

void MinIMU9::readMag()
{
  this->mag.read();
  // values in gauss
  this->m.x = (this->mag.m.x - this->magErrors[0]) / 6842.0;
  this->m.y = (this->mag.m.y - this->magErrors[1]) / 6842.0;
  this->m.z = (this->mag.m.z - this->magErrors[2]) / 6842.0;
}

void MinIMU9::read()
{
  this->readAcc();
  this->readGyro();
  this->readMag();
}

void MinIMU9::resetGyroX()
{
  this->angles.x = 0;
}

void MinIMU9::resetGyroY()
{
  this->angles.y = 0;
}

void MinIMU9::resetGyroZ()
{
  this->angles.z = 0;
}

void MinIMU9::resetGyro()
{
  this->resetGyroX();
  this->resetGyroY();
  this->resetGyroZ();
}

void MinIMU9::calculateIMUError() {
  const int iterations = 250*5;

  for (int i = 0; i < 3; i++)
  {
    this->accelErrors[i] = 0;
    this->gyroErrors[i] = 0;
    this->magErrors[i] = 0;
  }

  for (int i = 0; i < iterations; i++) {
    this->imu.readAcc();
    this->accelErrors[0] += this->imu.a.x;
    this->accelErrors[1] += this->imu.a.y;
    this->accelErrors[2] += this->imu.a.z - (G / (0.061 / 1000.0)); // Z acceleration should be experiencing 1g

    this->imu.readGyro();
    this->gyroErrors[0] += this->imu.g.x;
    this->gyroErrors[1] += this->imu.g.y;
    this->gyroErrors[2] += this->imu.g.z;

    this->readMag();
    this->magErrors[0] += this->mag.m.x;
    this->magErrors[1] += this->mag.m.y;
    this->magErrors[2] += this->mag.m.z;

    delay(5000.0 / iterations); // a total of 5 seconds is used for measuring the IMU values
//    unsigned long t = millis();
//    while (millis() - t < 5000.0 / iterations);
  }
  for (int i = 0; i < 3; i++) {
    this->accelErrors[i] /= iterations;
    this->gyroErrors[i] /= iterations;
    this->magErrors[i] /= iterations;
  }
}
