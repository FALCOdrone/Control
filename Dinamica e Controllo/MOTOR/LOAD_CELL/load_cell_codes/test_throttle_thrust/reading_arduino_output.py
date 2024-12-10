#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 24 22:52:17 2023

@author: andrea
"""
import serial

# Define the serial port and baud rate
port = '/dev/cu.usbmodem14201'  # Replace with your Arduino's serial port
baud_rate = 9600

# Open the serial port
ser = serial.Serial(port, baud_rate)

# Open a file to save the data
file_name = 'data_arduino.csv'
file = open(file_name, 'w')

try:
    while True:
        # Read the data from the serial port
        line = ser.readline().decode().strip()

        # Print the data to the console
        print(line)

        # Save the data to the file
        file.write(line + '\n')

except KeyboardInterrupt:
    # Close the file and serial port when interrupted
    file.close()
    ser.close()

