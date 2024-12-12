#include <Motor.h>

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
    Serial.available() >0;
    String command = Serial.readStringUntil('\n');
    if (command == "start")
    //{
    //    Motor::driveMotors();
    //}
    parseCommand(command);
    Motor::driveMotors(thrust);
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

void Motor::throttleCut(){
    MOTOR_FR.writeMicroseconds(ZERO_THRUST);
    MOTOR_FL.writeMicroseconds(ZERO_THRUST);
    MOTOR_RR.writeMicroseconds(ZERO_THRUST);
    MOTOR_RL.writeMicroseconds(ZERO_THRUST);
}

void parseCommand(String command) {  // send in serial something like thrust[1]=200
    if (command.startsWith("thrust[")) {
        // Trova l'indice
        int startIndex = command.indexOf('[') + 1;
        int endIndex = command.indexOf(']');
        int motorIndex = command.substring(startIndex, endIndex).toInt();

        // Trova il valore
        int equalIndex = command.indexOf('=');
        float value = command.substring(equalIndex + 1).toFloat();

        // Aggiorna il vettore thrust
        if (motorIndex >= 0 && motorIndex < 4) {
            thrust[motorIndex] = value;
        }

    }
}