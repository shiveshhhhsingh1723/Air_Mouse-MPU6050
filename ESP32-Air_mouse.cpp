#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BleMouse.h>

Adafruit_MPU6050 mpu;
BleMouse bleMouse("ESP32 Air Mouse", "Shivesh", 100);

// Angles
float pitch = 0.0, roll = 0.0;

// Timing
unsigned long lastTime = 0;
unsigned long lastTapTime = 0;
unsigned long lastTwistTime = 0;
unsigned long lastDoubleClickTime = 0;

// ========== TUNING PARAMETERS ==========

// --- Cursor Movement Settings ---
const float CURSOR_SPEED = 0.8;
const float DEADZONE = 5.0;

// --- Click Gesture Settings ---
const float TAP_THRESHOLD = 14.0;        // Z-axis for left click
const float TWIST_THRESHOLD = 250.0;     // Z-rotation for right click
const float DOUBLE_TAP_WINDOW = 400;     // ms window for double-click
const float SHAKE_THRESHOLD = 20.0;      // Shake for middle click

// --- Scroll Settings ---
const float SCROLL_ANGLE = 25.0;
const float SCROLL_SPEED = 1.0;
const bool SMOOTH_SCROLL = true;         // Enable smooth scrolling
const float SCROLL_SENSITIVITY = 0.5;    // For smooth scroll

// --- Cursor Smoothing ---
const bool ENABLE_SMOOTHING = true;      // Enable cursor smoothing filter
const float SMOOTHING_FACTOR = 0.7;      // 0.0-1.0, higher = smoother but more lag
float smoothedMoveX = 0.0;
float smoothedMoveY = 0.0;

// --- Advanced Gestures ---
const bool ENABLE_DOUBLE_CLICK = true;   // Enable double-tap gesture
const bool ENABLE_MIDDLE_CLICK = true;   // Enable shake gesture for middle click
const bool ENABLE_DRAG_MODE = false;     // Hold gesture to drag (experimental)

// --- Timing Settings ---
const unsigned long GESTURE_COOLDOWN = 600;

// --- Complementary Filter ---
const float GYRO_WEIGHT = 0.98;
const float ACCEL_WEIGHT = 0.02;

// ========================================

// Smoothing variables
int lastClickCount = 0;
bool isDragging = false;
unsigned long dragStartTime = 0;

// Moving average filter for scroll
float scrollBuffer[5] = {0};
int scrollBufferIndex = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  bleMouse.begin();

  Serial.println("╔════════════════════════════════════╗");
  Serial.println("║   ESP32 Air Mouse - Enhanced      ║");
  Serial.println("╚════════════════════════════════════╝");
  Serial.println("\nFeatures Enabled:");
  Serial.print("  ✓ Cursor Smoothing: "); Serial.println(ENABLE_SMOOTHING ? "ON" : "OFF");
  Serial.print("  ✓ Smooth Scroll: "); Serial.println(SMOOTH_SCROLL ? "ON" : "OFF");
  Serial.print("  ✓ Double Click: "); Serial.println(ENABLE_DOUBLE_CLICK ? "ON" : "OFF");
  Serial.print("  ✓ Middle Click: "); Serial.println(ENABLE_MIDDLE_CLICK ? "ON" : "OFF");
  Serial.println("\nGestures:");
  Serial.println("  → Tilt: Move cursor");
  Serial.println("  → Air Tap: Left Click");
  Serial.println("  → Double Tap: Double Click");
  Serial.println("  → Wrist Twist: Right Click");
  Serial.println("  → Shake: Middle Click");
  Serial.println("  → Tilt Forward/Back: Scroll");
  Serial.println("\nWaiting for Bluetooth connection...");

  delay(1000);
  lastTime = millis();
}

void loop() {
  if (!bleMouse.isConnected()) {
    delay(10);
    return;
  }

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Accelerometer angles
  float accelPitch = atan2(accel.acceleration.y,
                           sqrt(accel.acceleration.x * accel.acceleration.x +
                                accel.acceleration.z * accel.acceleration.z)) * 180 / PI;

  float accelRoll = atan2(-accel.acceleration.x,
                          accel.acceleration.z) * 180 / PI;

  // Gyro rates
  float gyroPitchRate = gyro.gyro.x * 180 / PI;
  float gyroRollRate  = gyro.gyro.y * 180 / PI;
  float gyroZRate     = gyro.gyro.z * 180 / PI;

  // Complementary filter
  pitch = GYRO_WEIGHT * (pitch + gyroPitchRate * dt) + ACCEL_WEIGHT * accelPitch;
  roll  = GYRO_WEIGHT * (roll  + gyroRollRate  * dt) + ACCEL_WEIGHT * accelRoll;

  // ========== CURSOR MOVEMENT WITH SMOOTHING ==========
  int moveX = 0;
  int moveY = 0;

  if (abs(roll) > DEADZONE)
    moveX = roll * CURSOR_SPEED;

  if (abs(pitch) > DEADZONE)
    moveY = -pitch * CURSOR_SPEED;

  // Apply exponential smoothing filter
  if (ENABLE_SMOOTHING) {
    smoothedMoveX = SMOOTHING_FACTOR * smoothedMoveX + (1 - SMOOTHING_FACTOR) * moveX;
    smoothedMoveY = SMOOTHING_FACTOR * smoothedMoveY + (1 - SMOOTHING_FACTOR) * moveY;
    moveX = (int)smoothedMoveX;
    moveY = (int)smoothedMoveY;
  }

  bleMouse.move(moveX, moveY);

  // ========== LEFT CLICK & DOUBLE CLICK ==========
  float accelZ = fabs(accel.acceleration.z);
  
  if (accelZ > TAP_THRESHOLD && millis() - lastTapTime > GESTURE_COOLDOWN) {
    
    // Check for double-click
    if (ENABLE_DOUBLE_CLICK && 
        (millis() - lastDoubleClickTime) < DOUBLE_TAP_WINDOW) {
      
      // DOUBLE CLICK detected
      bleMouse.click(MOUSE_LEFT);
      delay(50);
      bleMouse.click(MOUSE_LEFT);
      Serial.println("🖱️ DOUBLE CLICK");
      
      lastDoubleClickTime = 0; // Reset to prevent triple
      lastTapTime = millis();
      
    } else {
      // Single LEFT CLICK
      bleMouse.click(MOUSE_LEFT);
      Serial.println("🖱️ LEFT CLICK");
      
      lastDoubleClickTime = millis();
      lastTapTime = millis();
    }
  }

  // ========== RIGHT CLICK ==========
  if (fabs(gyroZRate) > TWIST_THRESHOLD &&
      millis() - lastTwistTime > GESTURE_COOLDOWN) {

    bleMouse.click(MOUSE_RIGHT);
    Serial.println("🖱️ RIGHT CLICK");
    lastTwistTime = millis();
  }

  // ========== MIDDLE CLICK (Shake Gesture) ==========
  if (ENABLE_MIDDLE_CLICK) {
    float totalAccel = sqrt(accel.acceleration.x * accel.acceleration.x +
                           accel.acceleration.y * accel.acceleration.y +
                           accel.acceleration.z * accel.acceleration.z);
    
    float shake = fabs(totalAccel - 9.8); // Deviation from gravity
    
    if (shake > SHAKE_THRESHOLD && 
        millis() - lastTapTime > GESTURE_COOLDOWN &&
        millis() - lastTwistTime > GESTURE_COOLDOWN) {
      
      bleMouse.click(MOUSE_MIDDLE);
      Serial.println("🖱️ MIDDLE CLICK");
      lastTapTime = millis();
      lastTwistTime = millis();
    }
  }

  // ========== SMOOTH SCROLLING ==========
  float scrollValue = 0;
  
  if (pitch > SCROLL_ANGLE) {
    scrollValue = -1 * SCROLL_SPEED;  // Scroll down
  }
  else if (pitch < -SCROLL_ANGLE) {
    scrollValue = 1 * SCROLL_SPEED;   // Scroll up
  }

  if (SMOOTH_SCROLL && scrollValue != 0) {
    // Moving average filter for smooth scroll
    scrollBuffer[scrollBufferIndex] = scrollValue;
    scrollBufferIndex = (scrollBufferIndex + 1) % 5;
    
    float avgScroll = 0;
    for (int i = 0; i < 5; i++) {
      avgScroll += scrollBuffer[i];
    }
    avgScroll /= 5.0;
    
    // Apply scroll with sensitivity
    if (fabs(avgScroll) > 0.3) {
      bleMouse.move(0, 0, (int)(avgScroll * SCROLL_SENSITIVITY));
    }
    
  } else if (scrollValue != 0) {
    // Regular scroll
    bleMouse.move(0, 0, (int)scrollValue);
  }

  delay(10);
}
