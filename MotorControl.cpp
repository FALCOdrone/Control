#include <Motor.h>

unsigned long lastMillis = 0;

Servo MOTOR_FR;
Servo MOTOR_FL;
Servo MOTOR_RR;
Servo MOTOR_RL;

void Motor::initialize(){
    MOTOR_FR.writeMicroseconds(ZERO_THRUST);
    MOTOR_FL.writeMicroseconds(ZERO_THRUST);
    MOTOR_RR.writeMicroseconds(ZERO_THRUST);
    MOTOR_RL.writeMicroseconds(ZERO_THRUST);
}

void Motor::driveMotors(float thrust[]){
    //Motor FR (front right)
    MOTOR_FR.writeMicroseconds(map(thrust[0], 0, 3560, 1000, 2000));
    //Motor FL
    MOTOR_FL.writeMicroseconds(map(thrust[1], 0, 3560, 1000, 2000));
    //Motor RR (rear right)
    MOTOR_RR.writeMicroseconds(map(thrust[2], 0, 3560, 1000, 2000));
    //Motor RL
    MOTOR_RL.writeMicroseconds(map(thrust[3], 0, 3560, 1000, 2000));
}

void parseCommand(String command) {  // send in serial something like "thrust[1]=200" or "start test"
    if (command.startsWith("thrust[")) {
        // finds the motor index (FR=0, FL=1, RR = 2, RL = 3)
        int startIndex = command.indexOf('[') + 1;
        int endIndex = command.indexOf(']');
        int motorIndex = command.substring(startIndex, endIndex).toInt();

        // find the value in the string and converts it and saves it to float variable value
        int equalIndex = command.indexOf('=');
        float value = command.substring(equalIndex + 1).toFloat();

        // thrust vector update
        if (motorIndex >= 0 && motorIndex < 4) {
            thrust[motorIndex] = value;
        }
    }
    else if (command == "start test")
    {
        int k = 0;
        while (k <= 1)
        {
            thurst[0] = k*100;
            Motor::driveMotors(thrust);
            delay(300);
            k+=1;
        }
    }
    else
    {
        Motor::throttleCut();
    }
}

void Motor::throttleCut(){
    MOTOR_FR.writeMicroseconds(ZERO_THRUST);
    MOTOR_FL.writeMicroseconds(ZERO_THRUST);
    MOTOR_RR.writeMicroseconds(ZERO_THRUST);
    MOTOR_RL.writeMicroseconds(ZERO_THRUST);
}


void setup(){
    Serial.begin(9600);

    // motor pin attachment and ESC and motors inizialization
    MOTOR_FR.attach(ESC1);  //where as dfined int pindef.h, ESCn are the pins
    MOTOR_FL.attach(ESC2); 
    MOTOR_RR.attach(ESC3); 
    MOTOR_RL.attach(ESC4);

    Motor::initialize();
}

void loop(){
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        parseCommand(command);
        Motor::driveMotors(thrust);
    }
}



