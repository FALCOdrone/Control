#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Servo.h>
#include "common/pinDef.h"

#define MAX_THRUST 2000
#define ZERO_THRUST 1000

class Motor {
  private:
    Servo motor; // Servo object to control the motor
    int pin;     // Motor pin

  public:
    Motor(int pin) {
        pin = pinNumber;
    }

    // motor inizialization
    void initialize() {
        motor.attach(pin);
        motor.writeMicroseconds(ZERO_THRUST);
    }

    // Motors drive function
    void driveMotors(float value) {
        int pulseWidth = map(value, 0, 3560, 1000, 2000);
        motor.writeMicroseconds(pulseWidth);
    }

    void throttleCut() {
        motor.writeMicroseconds(ZERO_THRUST);
    }
};

// Motor objects declaration and corresponding motor pin
Motor MOTOR_FR(ESC1);
Motor MOTOR_FL(ESC2);
Motor MOTOR_RR(ESC3);
Motor MOTOR_RL(ESC4);

#endif