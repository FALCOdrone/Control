#include "motor.h"

float thrust[4] = {0, 0, 0, 0};


void setup() {
    Serial.begin(9600);

    // Motor inizialization (pin attachment and ESC inizialization)
    MOTOR_FR.initialize();
    MOTOR_FL.initialize();
    MOTOR_RR.initialize();
    MOTOR_RL.initialize();
}

void parseCommand(String command) {
    if (command.startsWith("thrust[")) {
        // Finds motor index
        int startIndex = command.indexOf('[') + 1;
        int endIndex = command.indexOf(']');
        int motorIndex = command.substring(startIndex, endIndex).toInt();

        // Finds thrust value
        int equalIndex = command.indexOf('=');
        float value = command.substring(equalIndex + 1).toFloat();

        // Thrust vector update
        if (motorIndex >= 0 && motorIndex < 4) {
            thrust[motorIndex] = value;
        }
        // Updates motors with the thurst value
        MOTOR_FR.driveMotors(thrust[0]);
        MOTOR_FL.driveMotors(thrust[1]);
        MOTOR_RR.driveMotors(thrust[2]);
        MOTOR_RL.driveMotors(thrust[3]);
    }
    if (command == "start test") {
        for (int k = 0; k <= 10; k++) {
            thrust[0] = k * 100;
            thrust[1] = k * 100;
            thrust[2] = k * 100;
            thrust[3] = k * 100;
            MOTOR_FR.driveMotors(thrust[0]);
            MOTOR_FL.driveMotors(thrust[1]);
            MOTOR_RR.driveMotors(thrust[2]);
            MOTOR_RL.driveMotors(thrust[3]);
            delay(500);
        }
        for (int k = 10; k >= 0; k--) {
            thrust[0] = k * 100;
            thrust[1] = k * 100;
            thrust[2] = k * 100;
            thrust[3] = k * 100;
            MOTOR_FR.driveMotors(thrust[0]);
            MOTOR_FL.driveMotors(thrust[1]);
            MOTOR_RR.driveMotors(thrust[2]);
            MOTOR_RL.driveMotors(thrust[3]);
            delay(500);
        }
    }
    if (command == "s") {  // s to stop the motors
        MOTOR_FR.throttleCut();
        MOTOR_FL.throttleCut();
        MOTOR_RR.throttleCut();
        MOTOR_RL.throttleCut();
    }
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        parseCommand(command);
    }
}