#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// === Pin definitions ===
const int ENA = 21;  // Motor A enable (PWM)
const int IN1 = 22;  // Motor A direction 1
const int IN2 = 23;  // Motor A direction 2
const int ENB = 5;   // Motor B enable (PWM)
const int IN3 = 18;  // Motor B direction 1
const int IN4 = 19;  // Motor B direction 2

// === Motor control functions ===
void initMotors();
void forwardA(uint8_t spd);
void backwardA(uint8_t spd);
void stopA();
void brakeA();

void forwardB(uint8_t spd);
void backwardB(uint8_t spd);
void stopB();
void brakeB();

void stopAll();

#endif

