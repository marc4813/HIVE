#ifndef SD1_MinIMU_H // this is so that the header file isn't included more than once
#define SD1_MinIMU_H

#include "Arduino.h"
#include<Wire.h>
#include<LIS3MDL.h>
#include<LSM6.h>

#define G 9.81 // m/s^2

class MinIMU9 {
  private:
    float accelErrors[3];
    float gyroErrors[3];
    unsigned long gyro_time[3];
    bool isOpened = false;
    
  public:
    template <typename T> struct vector
    {
      T x, y, z;
    };
    LSM6 imu;
    LIS3MDL mag;
    MinIMU9();
    bool init();
    void calculateIMUError();
    vector<float> a; // accelerometer readings
    vector<float> g; // gyro readings
    vector<float> m; // magnetometer readings
    vector<float> angles; // gyro angle estimations
    void readAcc();
    void readGyro();
    void readMag();
    void read();
    void resetGyroX();
    void resetGyroY();
    void resetGyroZ();
    void resetGyro();
    bool isOpen();
};

#endif
