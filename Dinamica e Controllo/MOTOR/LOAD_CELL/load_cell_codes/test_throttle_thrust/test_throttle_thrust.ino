/*
The script will sweep between the throttle minVal and maxVal
in the discrete quantity stepQty.
At each step a settlingTime will be waited.

To reduce noise samplesAvg will be averaged.
*/

#include <Arduino.h>

// *** LOAD CELL ****
#include "HX711.h"                         // Include the library for interfacing with the HX711 load cell amplifier
HX711 scale(2, 3);                         // Create an instance of the HX711 class with data and clock pins connected to digital pins 2 and 3
const float calibration_factor = -111.18;  // Initialize a calibration factor to adjust the raw readings from the load cell
float thrust;                              // Variable to store the weight measurement in desired units

// *** ESC PARAMETERS ****
const int throttlePin = 4;  // PIN FOR OUTING ESC THROTTLE VALUE
int u = 0;                  // throttle value to give the ESC

// *** TEST PARAMETERS ****
const int minVal = 0;    // throttle minimum value.
const int maxVal = 255;  // throttle maxmum value.
const int stepVal = 5;
const int settlingTime = 3000;  // Settling time before measurement input change [ms]
const int samplesAvg = 10;      // Number of samples to average
float measurements[samplesAvg];
float sum;
int count = 0;
bool keyPressed = false;
bool flagEnd = false;

// EMERGENCY BUTTON
const int emergencyButtonPin = 5;  // Pin per il pulsante di emergenza
bool emergencyActivated = false;   // Flag per indicare se l'emergenza è stata attivata

void setup() {
    Serial.begin(9600);

    /* CALIBRATING LOAD CELL TO CALIBRATION FACTOR */
    Serial.println("HX711 calibration on-going");  // Print initialization message to serial monitor
    Serial.println("Remove all weight from scale ...");
    scale.set_scale();  // Set the scale to a default ratio (1.0) without calibration
    delay(500);
    scale.set_scale(calibration_factor);  // Set the scale with the current calibration factor
    delay(500);
    Serial.println("LOAD CELL Green Light.");

    // CODE ESC
    pinMode(throttlePin, OUTPUT);
    Serial.println("ESC Green Light.");

    // EMERGENCY BUTTON
    pinMode(emergencyButtonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(emergencyButtonPin), emergency(), FALLING);

    // LUNCH PYTHON CODE
    Serial.println("Lunch python code to read from Serial and then press 'p'.");
    while (!keyPressed) {
        if (Serial.available()) {
            char key = Serial.read();  // Read the incoming byte
            if (key == 'p') {
                keyPressed = true;
            }
        }
    }
    Serial.println("START");
    keyPressed = false;

    thrust = scale.get_units();
    if (thrust < 0) {
        thrust = 0.00;
    }

    analogWrite(throttlePin, u);
    Serial.print(u);
    Serial.print(",");
    Serial.println(thrust);
}

void emergency() {
    // Emergency button ISR
    emergencyActivated = true;    // Set the emergency flag
    analogWrite(throttlePin, 0);  // Turn off the motor
    Serial.println("Emergency! Motor off");
    while (true) {
        // Loop 2 infinity
    }
}

// LOOP
void loop() {
    // IF NEW AVERAGING, INCREMENT THE THROTTLE
    if (count == 0 && !flagEnd) {
        u += stepVal;
        analogWrite(throttlePin, u);

        // Waitnig for settlingTime, with emergency stop
        unsigned long startTime = millis();
        while (millis() - startTime < settlingTime) {
        }
    }

    // DO MEASURE
    thrust = scale.get_units();
    if (thrust < 0) {
        thrust = 0.00;
    }
    measurements[count] = thrust;

    if (count == (samplesAvg - 1)) {
        for (int i = 0; i < samplesAvg; i++) {
            sum += measurements[i];
        }
        float average_thrust = sum / samplesAvg;

        // PRINT
        Serial.print(u);
        Serial.print(",");
        Serial.println(average_thrust);

        if (u >= maxVal) {
            flagEnd = true;
            analogWrite(throttlePin, 0);
            while (true) {
                // Loop 2 infinity
            }
        }

        // init
        average_thrust = 0;
        sum = 0;
        count = -1;
    }

    count++;
    delay(500);
}