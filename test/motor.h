#ifndef MOTOR_H
#define MOTOR_H

//#include <Arduino.h>
#include <PWMServo.h>
//#include "common/pinDef.h"
#include "pinDef.h"

#define MAX_THRUST 2000
#define ZERO_THRUST 1000  

class Motor {
  private:
    PWMServo motor; // Servo object to control the motor
    int pin;     // Motor pin

  public:
    Motor(int pinNumber) {
        pin = pinNumber;
    }

    // motor inizialization
    void initialize() {
        motor.attach(pin,1000,2000);
        //motor.writeMicroseconds(ZERO_THRUST);
        //motor.writeMicroseconds(MAX_THRUST);
        motor.write(0);
        delay(1000);
        //motor.write(6);
        //delay(800);
        //motor.write(8);
        //delay(1000);
        //motor.write(6);
    }

    // Motors drive function
    void driveMotors(float value) {
        int pulseWidth = map(value, 0, 3560, 1000, 2000); //----------- mettere al posto di , 3560
        //motor.writeMicroseconds(pulseWidth);
        motor.write(pulseWidth);
    }

    void throttleCut() {
        //motor.writeMicroseconds(map(ZERO_THRUST, 0, 3560, 1000, 2000));
        //motor.writeMicroseconds(ZERO_THRUST);
        motor.write(pulseWidth);
    }
};

// Motor objects declaration and corresponding motor pin
Motor MOTOR_FR(ESC1);
Motor MOTOR_FL(ESC2);
Motor MOTOR_RR(ESC3);
Motor MOTOR_RL(ESC4);

#endif