#include "HX711.h"  // Include the library for interfacing with the HX711 load cell amplifier

HX711 scale(2, 3);  // Create an instance of the HX711 class with data and clock pins connected to digital pins 2 and 3

  /* INSERT CALIBRATION FACTOR FROM CALIBRATION CODE
  
  
  
   */

float calibration_factor = -111.18; // Initialize a calibration factor to adjust the raw readings from the load cell
float units;  // Variable to store the weight measurement in desired units

float throttle;  // Percentage of throttle applied

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud rate

  /* CALIBRATING LOAD CELL TO CALIBRATION FACTOR*/
  Serial.println("HX711 Measuring Thrust sketch");  // Print initialization message to serial monitor
  Serial.println("Remove all weight from scale...");
  scale.set_scale();  // Set the scale to a default ratio (1.0) without calibration
  scale.set_scale(calibration_factor);  // Set the scale with the current calibration factor
  Serial.println("Green Light.");
}






void loop() {
  

  units = scale.get_units();  // Read the weight measurement from the scale
  if (units < 0) {  // If the reading is negative (usually due to noise), set it to zero
    units = 0.00;
  }
  
  Serial.println(units);  // Print the weight measurement to the serial monitor and plotter

  if(Serial.available()) {  // Check if any characters are available in the serial buffer
    char temp = Serial.read();  // Read the character from the serial buffer
    if (temp == 'a') {  // If the character is 'a', increase the calibration factor by 5
      calibration_factor += 5;
    }
    else if(temp == 's') {  // If the character is 'z', decrease the calibration factor by 5
      calibration_factor += 10;
    } 
    else if(temp == 'z') {  // If the character is 'z', decrease the calibration factor by 5
      calibration_factor -= 5;
    }    
    else if(temp == 'z') {  // If the character is 'z', decrease the calibration factor by 5
      calibration_factor -= 10;
    }
  }
  delay(500);  // Delay for 1 second before looping again
}

