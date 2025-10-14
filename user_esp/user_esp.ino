/*
 * USER ESP32 - BNO085 Data Transmitter with Tare on Startup
 * Reads BNO085 IMU data and sends via ESP-NOW to suitcase
 * Tares (zeros) heading on startup
 * 
 * Upload this to the ESP32 that the USER carries
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_BNO08x.h>

// BNO085 SPI Pins
#define BNO08X_CS    5
#define BNO08X_INT   27
#define BNO08X_RESET 26

// IMPORTANT: Replace with your SUITCASE ESP32's MAC address
// To find MAC: Upload code to suitcase ESP32 first, it will print its MAC
uint8_t suitcaseAddress[] = {0x44, 0x1D, 0x64, 0xF6, 0x73, 0x74};  // Suitcase ESP's MAC address

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Data structure to send (keep it small for fast transmission)
typedef struct struct_message {
  uint8_t msg_type;      // 0=MOVE, 1=STEP, 2=STATUS, 3=ACTIVITY, 4=STABILITY
  float data1;           // Multi-purpose field
  float data2;
  float data3;
  float data4;
  uint32_t step_count;
  char text[20];         // For activity/stability names
  uint32_t timestamp;
} struct_message;

struct_message outgoingData;

// User state
struct UserState {
  float heading;
  float relative_x;
  float relative_y;
  float velocity_mag;
  bool is_moving;
  bool is_walking;
  uint32_t step_count;
  uint32_t last_step_time;
  String activity;
  String stability;
} user;

// Movement parameters
const float MOVEMENT_THRESHOLD = 0.15;
const float WALKING_SPEED_MAX = 2.5;
const uint32_t STEP_TIMEOUT = 2000;

float velocity_x = 0.0;
float velocity_y = 0.0;
unsigned long last_update_time = 0;

float quat_i = 0, quat_j = 0, quat_k = 0, quat_real = 1.0;
bool quat_valid = false;

// ESP-NOW callback (compatible with ESP32 core 3.x)
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Optional: Monitor send success
  if (status != ESP_NOW_SEND_SUCCESS) {
    // Uncomment for debugging:
    // Serial.println("⚠ Send failed!");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     USER ESP32 - IMU TRANSMITTER      ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  // Initialize WiFi in Station mode (required for ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  Serial.print("User MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW initialization failed!");
    return;
  }
  Serial.println("✓ ESP-NOW initialized");
  
  // Register send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register suitcase as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, suitcaseAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;  // Important: Specify interface
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add suitcase peer!");
    Serial.println("⚠ Make sure you set the correct MAC address!");
    Serial.print("   Trying to add: ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                  suitcaseAddress[0], suitcaseAddress[1], suitcaseAddress[2],
                  suitcaseAddress[3], suitcaseAddress[4], suitcaseAddress[5]);
    return;
  }
  Serial.println("✓ Suitcase peer registered");
  
  // Initialize BNO085
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("❌ Failed to find BNO08x chip!");
    while (1) delay(10);
  }
  Serial.println("✓ BNO085 connected");
  
  // Initialize user state
  user.heading = 0;
  user.relative_x = 0;
  user.relative_y = 0;
  user.velocity_mag = 0;
  user.is_moving = false;
  user.is_walking = false;
  user.step_count = 0;
  user.last_step_time = 0;
  user.activity = "Unknown";
  user.stability = "Unknown";
  
  setReports();
  
  // TARE IMU ON STARTUP
  Serial.println("\n⏳ Taring IMU (zeroing heading)...");
  delay(500);  // Give sensor a moment to stabilize
  
  if (tareIMU()) {
    Serial.println("✓ IMU tared successfully!");
  } else {
    Serial.println("⚠ IMU tare failed, but continuing anyway");
  }
  
  Serial.println("\n✓ Ready to transmit data!");
  Serial.println("════════════════════════════════════════\n");
  
  last_update_time = millis();
}

void setReports() {
  bno08x.enableReport(SH2_ROTATION_VECTOR, 20000);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, 20000);
  bno08x.enableReport(SH2_STEP_COUNTER, 100000);
  bno08x.enableReport(SH2_STEP_DETECTOR, 50000);
  bno08x.enableReport(SH2_STABILITY_CLASSIFIER, 100000);
  bno08x.enableReport(SH2_PERSONAL_ACTIVITY_CLASSIFIER, 100000);
}

bool tareIMU() {
  // Tare command: Sets current heading as zero reference
  // Uses Z-axis tare (yaw) with rotation vector basis
  
  // Access the underlying sh2 HAL to call tare
  // The Adafruit library doesn't expose tare functions, but we can call sh2 directly
  int status = sh2_setTareNow(SH2_TARE_Z, SH2_TARE_BASIS_ROTATION_VECTOR);
  
  if (status != SH2_OK) {
    Serial.print("Tare command failed with status: ");
    Serial.println(status);
    return false;
  }
  
  // Give the sensor time to process the tare
  delay(100);
  
  return true;
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.println("Sensor reset - re-enabling reports");
    setReports();
  }
  
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  
  // Process sensor data
  switch (sensorValue.sensorId) {
    case SH2_ROTATION_VECTOR:
      handleRotation();
      break;
      
    case SH2_LINEAR_ACCELERATION:
      handleLinearAcceleration();
      break;
      
    case SH2_STEP_DETECTOR:
      handleStepDetector();
      break;
      
    case SH2_STEP_COUNTER:
      handleStepCounter();
      break;
      
    case SH2_STABILITY_CLASSIFIER:
      handleStabilityClassifier();
      break;
      
    case SH2_PERSONAL_ACTIVITY_CLASSIFIER:
      handleActivityClassifier();
      break;
  }
  
  // Send periodic status every 100ms
  static unsigned long last_status = 0;
  if (millis() - last_status > 100) {
    sendStatus();
    last_status = millis();
  }
}

void handleRotation() {
  quat_i = sensorValue.un.rotationVector.i;
  quat_j = sensorValue.un.rotationVector.j;
  quat_k = sensorValue.un.rotationVector.k;
  quat_real = sensorValue.un.rotationVector.real;
  quat_valid = true;
  
  user.heading = getYawFromQuat();
  while (user.heading < 0) user.heading += 360.0;
  while (user.heading >= 360) user.heading -= 360.0;
}

void handleLinearAcceleration() {
  if (!quat_valid) return;
  
  float ax = sensorValue.un.linearAcceleration.x;
  float ay = sensorValue.un.linearAcceleration.y;
  float az = sensorValue.un.linearAcceleration.z;
  
  unsigned long current_time = millis();
  float dt = (current_time - last_update_time) / 1000.0;
  
  if (dt > 0.2 || dt <= 0) {
    last_update_time = current_time;
    return;
  }
  
  float world_ax, world_ay, world_az;
  rotateVector(ax, ay, az, &world_ax, &world_ay, &world_az);
  
  if (abs(world_ax) < 0.1) world_ax = 0;
  if (abs(world_ay) < 0.1) world_ay = 0;
  
  float decay = 0.94;
  velocity_x = velocity_x * decay + world_ax * dt;
  velocity_y = velocity_y * decay + world_ay * dt;
  
  user.velocity_mag = sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
  user.is_moving = (user.velocity_mag > MOVEMENT_THRESHOLD);
  
  bool speed_ok = (user.velocity_mag > MOVEMENT_THRESHOLD && 
                   user.velocity_mag < WALKING_SPEED_MAX);
  bool recent_steps = (millis() - user.last_step_time) < STEP_TIMEOUT;
  user.is_walking = speed_ok && recent_steps;
  
  user.relative_x += velocity_x * dt;
  user.relative_y += velocity_y * dt;
  
  if (!user.is_moving && !recent_steps) {
    velocity_x *= 0.85;
    velocity_y *= 0.85;
    user.relative_x *= 0.95;
    user.relative_y *= 0.95;
  }
  
  last_update_time = current_time;
  
  // Send movement data
  if (user.is_moving || user.is_walking) {
    sendMovementData();
  }
}

void handleStepDetector() {
  user.last_step_time = millis();
  sendStepData();
}

void handleStepCounter() {
  user.step_count = sensorValue.un.stepCounter.steps;
}

void handleStabilityClassifier() {
  uint8_t classification = sensorValue.un.stabilityClassifier.classification;
  
  switch (classification) {
    case 0: user.stability = "Unknown"; break;
    case 1: user.stability = "OnTable"; break;
    case 2: user.stability = "Stationary"; break;
    case 3: user.stability = "Stable"; break;
    case 4: user.stability = "Motion"; break;
    default: user.stability = "Unknown"; break;
  }
  
  sendStabilityData();
  
  if (classification == 2) {
    velocity_x *= 0.5;
    velocity_y *= 0.5;
  }
}

void handleActivityClassifier() {
  sh2_PersonalActivityClassifier_t* activity = &sensorValue.un.personalActivityClassifier;
  
  uint8_t max_confidence = 0;
  uint8_t most_likely = 0;
  
  for (int i = 0; i < 9; i++) {
    if (activity->confidence[i] > max_confidence) {
      max_confidence = activity->confidence[i];
      most_likely = i;
    }
  }
  
  if (max_confidence > 30) {
    switch (most_likely) {
      case 0: user.activity = "Unknown"; break;
      case 1: user.activity = "InVehicle"; break;
      case 2: user.activity = "OnBicycle"; break;
      case 3: user.activity = "OnFoot"; break;
      case 4: user.activity = "Still"; break;
      case 5: user.activity = "Tilting"; break;
      case 6: user.activity = "Walking"; break;
      case 7: user.activity = "Running"; break;
      case 8: user.activity = "OnStairs"; break;
      default: user.activity = "Unknown"; break;
    }
    
    sendActivityData(max_confidence);
    
    if (most_likely == 6) {
      user.is_walking = true;
    } else if (most_likely == 4) {
      user.is_walking = false;
    }
  }
}

// ESP-NOW send functions
void sendMovementData() {
  float distance = sqrt(user.relative_x * user.relative_x + 
                       user.relative_y * user.relative_y);
  float direction = atan2(user.relative_y, user.relative_x) * 180.0 / PI;
  if (direction < 0) direction += 360.0;
  
  outgoingData.msg_type = 0;  // MOVE
  outgoingData.data1 = distance;
  outgoingData.data2 = direction;
  outgoingData.data3 = user.velocity_mag;
  outgoingData.data4 = user.heading;
  outgoingData.timestamp = millis();
  
  esp_now_send(suitcaseAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
}

void sendStepData() {
  outgoingData.msg_type = 1;  // STEP
  outgoingData.step_count = user.step_count;
  outgoingData.data1 = user.heading;
  outgoingData.timestamp = millis();
  
  esp_now_send(suitcaseAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
}

void sendStatus() {
  float distance = sqrt(user.relative_x * user.relative_x + 
                       user.relative_y * user.relative_y);
  float direction = atan2(user.relative_y, user.relative_x) * 180.0 / PI;
  if (direction < 0) direction += 360.0;
  
  outgoingData.msg_type = 2;  // STATUS
  outgoingData.data1 = distance;
  outgoingData.data2 = direction;
  outgoingData.data3 = user.velocity_mag;
  outgoingData.data4 = user.heading;
  outgoingData.step_count = user.step_count;
  
  String cardinal = getCardinalDirection(direction);
  cardinal.toCharArray(outgoingData.text, 20);
  
  outgoingData.timestamp = millis();
  
  esp_now_send(suitcaseAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
}

void sendActivityData(uint8_t confidence) {
  outgoingData.msg_type = 3;  // ACTIVITY
  outgoingData.data1 = confidence;
  user.activity.toCharArray(outgoingData.text, 20);
  outgoingData.timestamp = millis();
  
  esp_now_send(suitcaseAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
}

void sendStabilityData() {
  outgoingData.msg_type = 4;  // STABILITY
  user.stability.toCharArray(outgoingData.text, 20);
  outgoingData.timestamp = millis();
  
  esp_now_send(suitcaseAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
}

// Helper functions
float getYawFromQuat() {
  float yaw = atan2(2.0 * (quat_real * quat_k + quat_i * quat_j), 
                    1.0 - 2.0 * (quat_j * quat_j + quat_k * quat_k));
  return yaw * 180.0 / PI;
}

float calculateRoll(float w, float x, float y, float z) {
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  double roll_rad = atan2(sinr_cosp, cosr_cosp);
  
  return roll_rad * (180.0 / PI);
}

float calculatePitch(float w, float x, float y, float z) {
  double sinp = 2.0 * (w * y - z * x);
  double pitch_rad;

  if (abs(sinp) >= 1) {
    pitch_rad = copysign(M_PI / 2, sinp);
  } else {
    pitch_rad = asin(sinp);
  }
  
  return pitch_rad * (180.0 / M_PI);
}

float getHeading_XUp_ZForward(float w, float x, float y, float z) {
  float new_w = w;
  float new_x = z;
  float new_y = y;
  float new_z = -x;

  float yaw = atan2(2.0 * (new_w * new_z + new_x * new_y),
                    1.0 - 2.0 * (new_y * new_y + new_z * new_z));

  float heading = yaw * 180.0 / PI;
  if (heading < 0) heading += 360.0;
  return heading;
}

void rotateVector(float x, float y, float z, float* out_x, float* out_y, float* out_z) {
  float q0 = quat_real;
  float q1 = quat_i;
  float q2 = quat_j;
  float q3 = quat_k;
  
  *out_x = (1 - 2*q2*q2 - 2*q3*q3) * x + (2*q1*q2 - 2*q0*q3) * y + (2*q1*q3 + 2*q0*q2) * z;
  *out_y = (2*q1*q2 + 2*q0*q3) * x + (1 - 2*q1*q1 - 2*q3*q3) * y + (2*q2*q3 - 2*q0*q1) * z;
  *out_z = (2*q1*q3 - 2*q0*q2) * x + (2*q2*q3 + 2*q0*q1) * y + (1 - 2*q1*q1 - 2*q2*q2) * z;
}

String getCardinalDirection(float angle) {
  if (angle >= 337.5 || angle < 22.5) return "N";
  else if (angle >= 22.5 && angle < 67.5) return "NE";
  else if (angle >= 67.5 && angle < 112.5) return "E";
  else if (angle >= 112.5 && angle < 157.5) return "SE";
  else if (angle >= 157.5 && angle < 202.5) return "S";
  else if (angle >= 202.5 && angle < 247.5) return "SW";
  else if (angle >= 247.5 && angle < 292.5) return "W";
  else return "NW";
}