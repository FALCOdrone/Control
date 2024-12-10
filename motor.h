#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Servo.h>
#include "common/pinDef.h"

#define ESC1
#define ESC2
#define ESC3
#define ESC4


class Motor {
    private:
        /* null */
    public:
        void initialize();
        void driveMotors(float thrust[4]); // thrust vector function 
        // respectively, float[0] = FR , float[1] = FL, float[2] = RR, float[3] = RL,
        // where FR is front right, RR is rear right and so on
        void throttleCut();
};

#endif