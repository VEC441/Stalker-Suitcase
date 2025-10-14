#include <Arduino.h>
#include "motors.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Motor test starting...");

  // Initialize motors (pins are already set up in motors.cpp)
  initMotors();
  stopA();
  stopB();
  delay(1000);
}

void loop() {
  Serial.println("Forward for 2 seconds...");
  forwardA(200);
  forwardB(200);
  delay(2000);

  Serial.println("Stop for 1 second...");
  stopA();
  stopB();
  delay(1000);

  Serial.println("Backward for 2 seconds...");
  backwardA(200);
  backwardB(200);
  delay(2000);

  Serial.println("Stop for 1 second...");
  stopA();
  stopB();
  delay(1000);
}
