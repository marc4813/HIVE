#ifndef SD1_MECANUM_H // this is so that the header file isn't included more than once
#define SD1_MECANUM_H

#include "Motor.h"

class Mecanum {
  private:
    float x, y, z;
    void init();
    void scaleMotors (float* m1v, float* m2v, float* m3v, float* m4v);
  public:
    Motor m1, m2, m3, m4;
    Mecanum(Motor &m1, Motor &m2, Motor &m3, Motor &m4);
    void drive(float x, float y, float z);
    void stop();
    float getXVel();
    float getYVel();
    float getZVel();
};

#endif
