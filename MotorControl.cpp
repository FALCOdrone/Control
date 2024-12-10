#include <Motor.h>

void Motor::initialize(){
    Servo ESC1;
    Servo ESC2;
    Servo ESC3;
    Servo ESC4;
}

void Motor::driveMotors(float thrust[]){
    //Motor FR (front right)
    ESC1.writeMicroseconds(map(thrust[0], 0, 3560, 1000, 2000));
    //Motor FL
    ESC2.writeMicroseconds(map(thrust[1], 0, 3560, 1000, 2000));
    //Motor RR (rear right)
    ESC3.writeMicroseconds(map(thrust[2], 0, 3560, 1000, 2000));
    //Motor RL
    ESC4.writeMicroseconds(map(thrust[3], 0, 3560, 1000, 2000));
}