#include "MinIMU9.h"

MinIMU9::MinIMU9()
{
//  init();
}

bool MinIMU9::init()
{
  bool results[2];
  results[0] = imu.init();
  results[1] = mag.init();
  if (results[0])
  {
    imu.enableDefault();
    calculateIMUError();
  }
  if (results[1])
  {
    mag.enableDefault();
  }
  gyro_time[0] = gyro_time[1] = gyro_time[2] = millis();
  isOpened = results[0] && results[1];
  return isOpened;
}

bool MinIMU9::isOpen()
{
  return isOpened;
}

void MinIMU9::readAcc()
{
  imu.readAcc();
  // values in g
  a.x = (imu.a.x - accelErrors[0]) * 0.061 / 1000.0;
  a.y = (imu.a.y - accelErrors[1]) * 0.061 / 1000.0;
  a.z = (imu.a.z - accelErrors[2]) * 0.061 / 1000.0;
}

void MinIMU9::readGyro()
{
  imu.readGyro();

  // calculating z angle immediately after getting gyro readings because z is typically more important
  g.z = (imu.g.z - gyroErrors[2]) * 8.75 / 1000.0; // degrees per second
  angles.z += g.z * (millis() - gyro_time[2]) / 1000.0; // degrees
  gyro_time[2] = millis();

  g.y = (imu.g.y - gyroErrors[1]) * 8.75 / 1000.0; // degrees per second
  angles.y += g.y * (millis() - gyro_time[1]) / 1000.0; // degrees
  gyro_time[1] = millis();
  
  g.x = (imu.g.x - gyroErrors[0]) * 8.75 / 1000.0; // degrees per second
  angles.x += g.x * (millis() - gyro_time[0]) / 1000.0; // degrees
  gyro_time[0] = millis();
}

void MinIMU9::readMag()
{
  mag.read();
  // values in gauss
  m.x = mag.m.x / 6842.0;
  m.y = mag.m.y / 6842.0;
  m.z = mag.m.z / 6842.0;
}

void MinIMU9::read()
{
  readAcc();
  readGyro();
  readMag();
}

void MinIMU9::resetGyroX()
{
  angles.x = 0;
}

void MinIMU9::resetGyroY()
{
  angles.y = 0;
}

void MinIMU9::resetGyroZ()
{
  angles.z = 0;
}

void MinIMU9::resetGyro()
{
  resetGyroX();
  resetGyroY();
  resetGyroZ();
}

void MinIMU9::calculateIMUError() {
  const int iterations = 250*5;

  for (int i = 0; i < iterations; i++) {
    imu.readAcc();
    accelErrors[0] += imu.a.x;
    accelErrors[1] += imu.a.y;
    accelErrors[2] += imu.a.z - (G / (0.061 / 1000.0)); // Z acceleration should be experiencing 1g

    imu.readGyro();
    gyroErrors[0] += imu.g.x;
    gyroErrors[1] += imu.g.y;
    gyroErrors[2] += imu.g.z;

    delay(5000.0 / iterations); // a total of 5 seconds is used for measuring the IMU values
//    unsigned long t = millis();
//    while (millis() - t < 5000.0 / iterations);
  }
  for (int i = 0; i < 3; i++) {
    accelErrors[i] /= iterations;
    gyroErrors[i] /= iterations;
  }
}
