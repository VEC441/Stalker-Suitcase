#include "motors.h"

// ====== PWM Parameters ======
const int pwmFreq = 1000; // 1 kHz
const int pwmRes  = 8;    // 8-bit resolution (0–255)

// ====== Speed Control Variables ======
const int BASE_SPEED = 200;
// const int TURN_ADJUST = 140;  

// ---- Internal helper functions ----
static void setSpeedA(uint8_t spd) { ledcWrite(ENA, spd); }
static void setSpeedB(uint8_t spd) { ledcWrite(ENB, spd); }

// ---- Initialization ----
void initMotors() {
  // Set direction pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Attach PWM to enable pins (ESP32 ledc API)
  ledcAttach(ENA, pwmFreq, pwmRes);
  ledcAttach(ENB, pwmFreq, pwmRes);

  // Stop both motors initially
  stopAll();
}

// ---- Motor A control ----
void forwardA(uint8_t spd)  { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  setSpeedA(spd); }
void backwardA(uint8_t spd) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); setSpeedA(spd); }
void stopA()                { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  setSpeedA(0);   }
void brakeA()               { digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH); setSpeedA(0);   }

// ---- Motor B control ----
void forwardB(uint8_t spd)  { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  setSpeedB(spd); }
void backwardB(uint8_t spd) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); setSpeedB(spd); }
void stopB()                { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  setSpeedB(0);   }
void brakeB()               { digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH); setSpeedB(0);   }

// ---- Suitcase control ----
void goForward()  {forwardA(BASE_SPEED); forwardB(BASE_SPEED);}
void goReverse()  {backwardA(BASE_SPEED); backwardB(BASE_SPEED);}
void turnRight()  {backwardA(BASE_SPEED); forwardB(BASE_SPEED);}
void turnLeft()   {forwardA(BASE_SPEED); backwardB(BASE_SPEED);}
void stopAll()    {stopA(); stopB();}