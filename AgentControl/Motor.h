#ifndef SD1_MOTOR_H // this is so that the header file isn't included more than once
#define SD1_MOTOR_H

#include "Arduino.h"

class Motor {
  private:
    int speedPin;
    int directionPin;
    bool inverse;
    int hallEffectPin;
    bool newDriver; //TEMP
    int speed;
    int observedSpeed;
    bool direction;
    long lastHallReading;
    void init();

  public:
    Motor(int speedPin, int directionPin, int hallEffectPin = -1, bool inverse = false);
    static int HallReadingsPerRev;
    static float MaxRPM;
    void drive(int speed);
    int getObservedSpeed();
    int getRawSpeed();
    bool getDirection();
};

#endif
