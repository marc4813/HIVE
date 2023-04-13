#ifndef SD1_MOTOR_H // this is so that the header file isn't included more than once
#define SD1_MOTOR_H

#include "Arduino.h"

class Motor {
  private:
    int speedPin;
    int directionPin;
    bool inverse;
    int speed;
    int observedSpeed;
    bool direction;
    void init();

  public:
    Motor(int speedPin, int directionPin, bool inverse = false);
    void drive(int speed);
    int getObservedSpeed();
    int getRawSpeed();
    bool getDirection();
};

#endif
