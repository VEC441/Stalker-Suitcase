#include <esp_now.h>
#include <WiFi.h>
#include "motors.h"

// ========== Data Structure (from user ESP32) ==========
typedef struct struct_message {
  uint8_t msg_type;      // 0=MOVE, 1=STEP, 2=STATUS, 3=ACTIVITY, 4=STABILITY
  float data1;           // distance or confidence
  float data2;           // direction
  float data3;           // velocity magnitude
  float data4;           // heading
  uint32_t step_count;
  char text[20];
  uint32_t timestamp;
} struct_message;

struct_message incomingData;

// ========== User State ==========
typedef struct user_pos {
  float heading;         // User heading (deg)
  bool is_moving;        // Movement state
  uint32_t last_update;  // Timestamp of last packet
} user_pos_t;

user_pos_t userPos;

// ========== Constants ==========
#define TIMEOUT_MS 2000  // Timeout for communication loss
#define MOVEMENT_THRESHOLD 0.10  // Same as user ESP32

// ========== Function Prototypes ==========
void onReceive(const esp_now_recv_info *info, const uint8_t *data, int len);
void updateFollowLogic();

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  delay(2000);

  // Initialize motors
  initMotors();

  // Initialize Wi-Fi in STA mode for ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(onReceive);

  Serial.println("✓ Suitcase receiver ready.");
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
void onReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len != sizeof(struct_message)) return;

  memcpy(&incomingData, data, sizeof(struct_message));
  userPos.last_update = millis();

  // Extract only what we need
  userPos.heading = incomingData.data4;

  // Determine if user is moving
  if (incomingData.msg_type == 0 || incomingData.msg_type == 2) {  // MOVE or STATUS
    userPos.is_moving = (incomingData.data3 > MOVEMENT_THRESHOLD);
  }

  Serial.printf("Recv | Heading: %.2f | Vel: %.2f | Moving: %s\n",
                userPos.heading,
                incomingData.data3,
                userPos.is_moving ? "YES" : "NO");
}

// ========== Follow Logic ==========
void updateFollowLogic() {
  if (!userPos.is_moving) {
    stopAll();
    return;
  }

  // Calculate heading error (simplified)
  float heading_error = userPos.heading;
  if (heading_error > 180) heading_error -= 360;
  if (heading_error < -180) heading_error += 360;


  if (abs(heading_error) < 45) {
    // Move straight
    goForward();
  } else if (heading_error < 45) {
    // Turn right
    turnRight();
  } else if (heading_error > -45) {
    // Turn left
    turnLeft();
  }

  // Debug output
  Serial.printf("Move | Heading error: %.1f | Moving: %s\n",
                heading_error, userPos.is_moving ? "YES" : "NO");
}
