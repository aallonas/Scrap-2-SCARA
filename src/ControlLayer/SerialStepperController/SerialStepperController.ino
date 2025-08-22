/***************************************************************
 * File:          SerialStepperController.ino
 * Author:        ALLONAS Alexandre
 * Dates:
 *    Creation:   27/07/2025
 *    Update:     17/08/2025
 * Description:   Arduino code for controlling a SCARA robot arm
 *                using stepper motors and receiving end effector
 *                positions from ROS2 via serial.
 * 
 * Hardware:      - Arduino Uno
 *                - CNC Shield V3
 *                - Stepper Motor Drivers (e.g., A4988, DRV8825)
 *                - NEMA 17 Stepper Motors
 *                - 12/36v Power Supply
 *                - Optional: endstops, serial comms
 * 
 * Libraries:     - AccelStepper
 * 
 * Version:       1.0
 ***************************************************************/


#include <AccelStepper.h>
#include <math.h>

// Stepper pins
#define STEP_PIN_1 2
#define STEP_PIN_2 3
#define STEP_PIN_3 4
#define STEP_PIN_4 12

#define DIR_PIN_1 5
#define DIR_PIN_2 6
#define DIR_PIN_3 7
#define DIR_PIN_4 13

// Stepper instances
AccelStepper stepper_1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper_2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
AccelStepper stepper_3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);
AccelStepper stepper_4(AccelStepper::DRIVER, STEP_PIN_4, DIR_PIN_4);

// Joint variables
double q1 = 0;
double q2 = 0;
double q3 = 0;
double q4 = 0;

// Robot link lengths (mm)
const double l1 = 100;
const double l2 = 100;

// Conversion factors 
const double steps_per_rad = 200;  // steps per radian
const double steps_per_mm  = 100;  // steps per mm (to be updated with gear ratios)



void setup() {
  Serial.begin(115200);

  // Set max speed and acceleration
  stepper_1.setMaxSpeed(1000);
  stepper_2.setMaxSpeed(1000);
  stepper_3.setMaxSpeed(1000);
  stepper_4.setMaxSpeed(1000);

  stepper_1.setAcceleration(200);
  stepper_2.setAcceleration(200);
  stepper_3.setAcceleration(200);
  stepper_4.setAcceleration(200);
}



void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    if (line.startsWith("POSE")) {
      float px, py, pz, qx, qy, qz, qw;
      int effector;
      sscanf(line.c_str(), "POSE,%f,%f,%f,%f,%f,%f,%f,%d",
             &px, &py, &pz, &qx, &qy, &qz, &qw, &effector);

      // Inverse kinematics
      double D = (pow(px,2) + pow(py,2) - pow(l1,2) - pow(l2,2)) / (2*l1*l2);
      q2 = acos(D);
      q1 = atan2(py, px) - atan2(l2*sin(q2), l1 + l2*cos(q2));

      // Extract yaw from quaternion
      double yaw = atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));

      // SCARA orientation constraint
      q3 = yaw - (q1 + q2);

      // Vertical prismatic
      q4 = pz;

      // Send to steppers (convert to steps)
      stepper_1.moveTo(q1 * steps_per_rad);
      stepper_2.moveTo(q2 * steps_per_rad);
      stepper_3.moveTo(q3 * steps_per_rad);
      stepper_4.moveTo(q4 * steps_per_mm);
    }
  }

  // Run steppers towards target
  stepper_1.run();
  stepper_2.run();
  stepper_3.run();
  stepper_4.run();
}
