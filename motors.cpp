#include "motors.h"

// === PWM configuration ===
const int pwmFreq = 1000; // 1 kHz
const int pwmRes  = 8;    // 8-bit (0–255)

// === Internal helpers ===
void setSpeedA(uint8_t spd) { ledcWrite(ENA, spd); }
void setSpeedB(uint8_t spd) { ledcWrite(ENB, spd); }

// === Motor A controls ===
void forwardA(uint8_t spd) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  setSpeedA(spd);
}

void backwardA(uint8_t spd) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  setSpeedA(spd);
}

void stopA() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  setSpeedA(0);
}

void brakeA() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  setSpeedA(0);
}

// === Motor B controls ===
void forwardB(uint8_t spd) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setSpeedB(spd);
}

void backwardB(uint8_t spd) {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setSpeedB(spd);
}

void stopB() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  setSpeedB(0);
}

void brakeB() {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  setSpeedB(0);
}

void stopAll() {
  stopA();
  stopB();
}

// === Initialization ===
void initMotors() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcAttach(ENA, pwmFreq, pwmRes);
  ledcAttach(ENB, pwmFreq, pwmRes);

  stopAll();
}

