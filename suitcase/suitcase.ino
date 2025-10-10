#include <esp_now.h>
#include <WiFi.h>
#include "motors.h"

// ========== Data Structure ==========
typedef struct user_pos {
  float heading;         // User heading (deg)
  uint32_t step_count;   // Steps taken by user
  uint32_t last_update;  // Timestamp of last packet
} user_pos_t;

user_pos_t userPos;

// ========== Constants ==========
#define TIMEOUT_MS 2000  // Timeout for communication loss

// ========== Function Prototypes ==========
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
void updateFollowLogic();

// ========== Setup ==========
void setup() {
  Serial.begin(115200);

  // Initialize motors (your modular motor driver)
  initMotors();

  // Initialize Wi-Fi in STA mode for ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(onReceive);

  Serial.println("Suitcase receiver ready.");
}

// ========== Main Loop ==========
void loop() {
  // If user data is stale, stop motors
  if (millis() - userPos.last_update > TIMEOUT_MS) {
    stopAll();
  } else {
    updateFollowLogic();
  }

  delay(50);
}

// ========== ESP-NOW Receive ==========
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(user_pos_t)) return;

  memcpy(&userPos, incomingData, sizeof(user_pos_t));
  userPos.last_update = millis();

  Serial.printf("Recv | Heading: %.2f | Steps: %u\n",
                userPos.heading, userPos.step_count);
}

// ========== Follow Logic ==========
void updateFollowLogic() {
  static uint32_t last_step = 0;
  bool user_moved = (userPos.step_count > last_step);
  last_step = userPos.step_count;

  if (!user_moved) {
    stopAll();
    return;
  }

  // Calculate heading error (simplified)
  float heading_error = userPos.heading;
  if (heading_error > 180) heading_error -= 360;
  if (heading_error < -180) heading_error += 360;

  const int BASE_SPEED = 180;   // Adjust as needed
  const int TURN_ADJUST = 60;   // Adjust for turning strength

  if (abs(heading_error) < 15) {
    // Move straight
    forwardA(BASE_SPEED);
    forwardB(BASE_SPEED);
  } else if (heading_error > 15) {
    // Turn right
    forwardA(BASE_SPEED - TURN_ADJUST);
    forwardB(BASE_SPEED + TURN_ADJUST);
  } else if (heading_error < -15) {
    // Turn left
    forwardA(BASE_SPEED + TURN_ADJUST);
    forwardB(BASE_SPEED - TURN_ADJUST);
  }

  // Debug output
  Serial.printf("Move | Heading error: %.1f | L:%d R:%d\n",
                heading_error, BASE_SPEED, TURN_ADJUST);
}
