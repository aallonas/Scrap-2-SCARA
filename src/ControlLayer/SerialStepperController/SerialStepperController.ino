/***************************************************************
 * File:          SerialStepperController.ino
 * Author:        ALLONAS Alexandre
 * Dates:
 *    Creation:   27/07/2025
 *    Update:     27/07/2025
 * Description:   Arduino code for controlling a SCARA robot arm
 *                using stepper motors and receiving joint space
 *                trajectories from a host (e.g., ROS).
 * 
 * Hardware:      - Arduino Uno
 *                - CNC Shield V3
 *                - Stepper Motor Drivers (e.g., A4988, DRV8825)
 *                - NEMA 17 Stepper Motors
 *                - 12/36v Power Supply
 *                - Optional: endstops, serial comms
 * 
 * Libraries:     - AccelStepper
 *                - ros
 *                - std_msgs/String
 * 
 * Version:       1.0
 ***************************************************************/

#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/String.h>

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
