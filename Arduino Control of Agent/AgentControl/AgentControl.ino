#include "Mecanum.h"

// PS2 Controller Arduino Library: https://github.com/madsci1016/Arduino-PS2X/blob/master/PS2X_lib/PS2X_lib.cpp
#include <PS2X_lib.h>

// Define pins for motor controllers
#define m1_pwm 4
#define m1_gpio 30
#define m2_pwm 7
#define m2_gpio 34
#define m3_pwm 2
#define m3_gpio 22
#define m4_pwm 3
#define m4_gpio 26

// Define pins for PS2 controller breakout board
#define ps2_data 10
#define ps2_cmd 11
#define ps2_att 12
#define ps2_clk 13

PS2X ps2x;
int error = 1;

// Motor(int speedPin, int directionPin, [bool inverse = false])
Motor m1 = Motor(m1_pwm, m1_gpio);
Motor m2 = Motor(m2_pwm, m2_gpio, true);
Motor m3 = Motor(m3_pwm, m3_gpio);
Motor m4 = Motor(m4_pwm, m4_gpio, true);

// Mecanum(Motor &m1, Motor &m2, Motor &m3, Motor &m4)
Mecanum drivetrain = Mecanum(m1, m2, m3, m4);

void setup() {
  Serial.begin(9600);
  // Attempts to connect to PS2 controller a maximum of 50 times
  for (int i = 0; i < 50 && error; i++) {
    // config_gamepad(clock, command, attention, data, pressures?, rumble?)
    error = ps2x.config_gamepad(ps2_clk, ps2_cmd, ps2_att, ps2_data, false, false);
    delay(500);
  }
}

void loop() {
  if (error == 1) {
    Serial.println("Error, no controller");
    return; // Skip if no controller
  }
  ps2x.read_gamepad(); // Must be called frequently to read updated values
  float lx = 2 * (ps2x.Analog(PSS_LX) / 255.0) - 1; // Left stick x-axis
  float ly = -(2 * (ps2x.Analog(PSS_LY) / 255.0) - 1); // Left stick y-axis
  float rx = 2 * (ps2x.Analog(PSS_RX) / 255.0) - 1; // Right stick x-axis
  // Above values are adjusted to be between -1.0 and 1.0

  // Use of the D-Pad on the PS2 controller will override the left stick.
  // This is to test with digital power (max on or off), while the stick allows for analog power.
  float pad_x = 0.0;
  float pad_y = 0.0;
  if (ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN)) {
    pad_y = ps2x.Button(PSB_PAD_UP) ? 1.0 : -1.0;
  } else if (ps2x.Button(PSB_PAD_RIGHT) || ps2x.Button(PSB_PAD_LEFT)) {
    pad_x = ps2x.Button(PSB_PAD_RIGHT) ? 1.0 : -1.0;
  }

  // Controls the drivetrain based on either the D-Pad or the left stick.
  // Right stick still works for turning in either case.
  // drivetrain.drive(float x, float y, float z)
  if (pad_x || pad_y) {
    drivetrain.drive(pad_y, pad_x, rx); // x and y intentionally swapped
  } else {
    drivetrain.drive(ly, lx, rx); // x and y intentionally swapped
  }

  // For debugging purposes via the Serial Monitor:
//  printCommandVelocities();
//  printRawSpeeds();
//  printObservedSpeeds();

  delay(50);
}

// Displays command velocities that are sent to the drivetrain.
// Values should be between -1.0 and 1.0.
void printCommandVelocities() {
  Serial.print("XVel: ");
  Serial.print(drivetrain.getXVel());
  Serial.print("\tYVel: ");
  Serial.print(drivetrain.getYVel());
  Serial.print("\tZVel: ");
  Serial.println(drivetrain.getZVel());
}

// Displays the raw PWM data sent to each motor.
// Values will be between 0 and 255, where either 0 or 255 can be max speed.
void printRawSpeeds() {
  Serial.print("M1R: ");
  Serial.print(drivetrain.m1.getRawSpeed());
  Serial.print("\tM2R: ");
  Serial.print(drivetrain.m2.getRawSpeed());
  Serial.print("\tM3R: ");
  Serial.print(drivetrain.m3.getRawSpeed());
  Serial.print("\tM4R: ");
  Serial.println(drivetrain.m4.getRawSpeed());
}

// Displays the desired speeds of each motor relative to the robot.
// 255 is full speed forward, -255 is full speed reverse, and 0 is not moving.
void printObservedSpeeds() {
  Serial.print("M1O: ");
  Serial.print(drivetrain.m1.getObservedSpeed());
  Serial.print("\tM2O: ");
  Serial.print(drivetrain.m2.getObservedSpeed());
  Serial.print("\tM3O: ");
  Serial.print(drivetrain.m3.getObservedSpeed());
  Serial.print("\tM4O: ");
  Serial.println(drivetrain.m4.getObservedSpeed());
}
