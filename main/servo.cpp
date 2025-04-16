#include "servo.h"

void servoBegin() {
  pinMode(SERVO_PIN, OUTPUT);
}

void servo(int angle) {
  // Calculate the pulse duration based on the angle
  // For example, if a 180-degree range is used, the pulse might range from 1ms to 2ms
  int pulseWidth = map(angle, 0, 180, 1000, 2000);  // Map angle to microseconds

  // Send the pulse to the servo pin
  // This part depends on your Cytron board and microcontroller
  // Example using digitalWrite for Arduino
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(SERVO_PIN, LOW);
  delayMicroseconds(20000 - pulseWidth);  // Wait for the rest of the cycle
}