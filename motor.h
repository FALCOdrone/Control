#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "common/pinDef.h"

#define MAX_THRUST 2000
#define ZERO_THRUST 100  // check what is the actual zero thrust for us (it was 1000 before i changed it to 100)

class Motor {
  private:
    Servo motor; // Servo object to control the motor
    int pin;     // Motor pin

  public:
    Motor(int pinNumber) {
        pin = pinNumber;
    }

    // motor inizialization
    void initialize() {
        motor.attach(pin,1000,2000);
        motor.write(0);
        delay(1000);
    }

    // Motors drive function
    void driveMotors(float value) {
        int pulseWidth = map(value, 0, 3560, 1000, 2000); //----------- mettere al posto di , 3560
        motor.writeMicroseconds(pulseWidth);
    }

    void throttleCut() {  //it doesnt work for some reason
        //motor.writeMicroseconds(map(ZERO_THRUST, 0, 3560, 1000, 2000));
        motor.writeMicroseconds(ZERO_THRUST);
    }
};

// Motor objects declaration and corresponding motor pin
Motor MOTOR_FR(ESC1);
Motor MOTOR_FL(ESC2);
Motor MOTOR_RR(ESC3);
Motor MOTOR_RL(ESC4);

#endif