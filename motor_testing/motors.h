#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// ====== Pin Definitions ======
#define ENA 21   // PWM / Enable for Motor A
#define IN1 22   // Direction 1 for Motor A
#define IN2 23   // Direction 2 for Motor A

#define ENB 5    // PWM / Enable for Motor B
#define IN3 18   // Direction 1 for Motor B
#define IN4 19   // Direction 2 for Motor B

// ====== Function Declarations ======
void initMotors();
void forwardA(uint8_t speed);
void backwardA(uint8_t speed);
void stopA();
void brakeA();

void forwardB(uint8_t speed);
void backwardB(uint8_t speed);
void stopB();
void brakeB();

#endif


