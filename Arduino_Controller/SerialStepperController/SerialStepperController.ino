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

const uint8_t HEADER = 0xAA;
const uint8_t FOOTER = 0x55;
const size_t PACKET_SIZE = 18;

long targets[4] = {0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
  // Optional: set max speed and acceleration for each stepper
  stepper_1.setMaxSpeed(1000); stepper_1.setAcceleration(500);
  stepper_2.setMaxSpeed(1000); stepper_2.setAcceleration(500);
  stepper_3.setMaxSpeed(1000); stepper_3.setAcceleration(500);
  stepper_4.setMaxSpeed(1000); stepper_4.setAcceleration(500);
}

void loop() {
  // Handle incoming serial data
  if (Serial.available() >= PACKET_SIZE) {
    // Look for header
    if (Serial.peek() == HEADER) {
      uint8_t packet[PACKET_SIZE];
      Serial.readBytes(packet, PACKET_SIZE);
      if (packet[PACKET_SIZE - 1] == FOOTER) {
        // Parse 4 int32 targets (little-endian)
        for (int i = 0; i < 4; ++i) {
          long val = 0;
          for (int b = 0; b < 4; ++b) {
            val |= ((long)packet[1 + i * 4 + b]) << (8 * b);
          }
          targets[i] = val;
        }
        // Set stepper targets
        stepper_1.moveTo(targets[0]);
        stepper_2.moveTo(targets[1]);
        stepper_3.moveTo(targets[2]);
        stepper_4.moveTo(targets[3]);
      }
    } else {
      Serial.read(); // Discard until header found
    }
  }

  // Run steppers
  stepper_1.run();
  stepper_2.run();
  stepper_3.run();
  stepper_4.run();

  // Periodically send current positions
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 100) { // 10Hz
    sendPositions();
    lastSend = millis();
  }
}

void sendPositions() {
  uint8_t packet[PACKET_SIZE];
  packet[0] = HEADER;
  long positions[4] = {
    stepper_1.currentPosition(),
    stepper_2.currentPosition(),
    stepper_3.currentPosition(),
    stepper_4.currentPosition()
  };
  for (int i = 0; i < 4; ++i) {
    for (int b = 0; b < 4; ++b) {
      packet[1 + i * 4 + b] = (positions[i] >> (8 * b)) & 0xFF;
    }
  }
  packet[PACKET_SIZE - 1] = FOOTER;
  Serial.write(packet, PACKET_SIZE);
}