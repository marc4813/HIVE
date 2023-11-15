#ifndef SD1_MOTOR_H // this is so that the header file isn't included more than once
#define SD1_MOTOR_H

#include "Arduino.h"

class Motor {
  private:
    int speedPin;
    int directionPin;
    bool inverse;
    int hallEffectPin;
    int speed;
    int observedSpeed;
    bool direction;
    unsigned long lastHallReading;
    bool prevHallState;
    float currentRPM;
    unsigned long saturatedTimeStart;
    float prevDesiredRPM;
    unsigned long prevRPMTime;
    void init();

  public:
    Motor(int speedPin, int directionPin, int hallEffectPin = -1, bool inverse = false);
    static int HallReadingsPerRev;
    static constexpr float MaxRPM = 30;
    void drive(int speed);
    void setRPM(float desiredRPM);
    int getObservedSpeed();
    int getRawSpeed();
    bool getDirection();
    float getRPM();
    bool isSaturated();
    float returnRPM();
};

#endif
