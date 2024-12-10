#include "HX711.h"  // Include the library for interfacing with the HX711 load cell amplifier

HX711 scale(2, 3);  // Create an instance of the HX711 class with data and clock pins connected to digital pins 2 and 3

float calibration_factor = -111.18; // Initialize a calibration factor to adjust the raw readings from the load cell
float units;  // Variable to store the weight measurement in desired units

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud rate
  Serial.println("HX711 calibration sketch");  // Print initialization message to serial monitor
  Serial.println("Remove all weight from scale");
  Serial.println("After readings begin, place known weight on scale");
  scale.set_scale();  // Set the scale to a default ratio (1.0) without calibration
  scale.tare();  // Reset the scale to zero by subtracting the current reading from the offset
  delay(2000);
  Serial.println("Scale set to default ratio");
  Serial.println("Press key to increase calibration factor: a = +5 | s =+10");
  Serial.println("Press key to increase calibration factor: z = -5 | x =-10");
  delay(2000);
}

void loop() {
  scale.set_scale(calibration_factor);  // Set the scale with the current calibration factor

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

