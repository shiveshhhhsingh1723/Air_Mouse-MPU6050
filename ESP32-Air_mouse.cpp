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

// ========== TUNING PARAMETERS - ADJUST THESE ==========

// --- Cursor Movement Settings ---
const float CURSOR_SPEED = 0.8;          // 0.5 = slower, 1.2 = faster
const float DEADZONE = 5.0;              // 3.0 = more sensitive, 8.0 = less sensitive

// --- Click Gesture Settings ---
const float TAP_THRESHOLD = 14.0;        // 10.0 = easier tap, 18.0 = harder tap
const float TWIST_THRESHOLD = 250.0;     // 180.0 = easier twist, 350.0 = harder twist

// --- Scroll Settings ---
const float SCROLL_ANGLE = 25.0;         // 20.0 = easier scroll, 35.0 = harder scroll
const float SCROLL_SPEED = 1.0;          // Multiplier for scroll speed (1.0 = normal)

// --- Timing Settings ---
const unsigned long GESTURE_COOLDOWN = 600;  // 400 = faster repeat, 800 = slower repeat

// --- Complementary Filter ---
const float GYRO_WEIGHT = 0.98;          // 0.95 = less drift/more jitter, 0.99 = more drift/less jitter
const float ACCEL_WEIGHT = 0.02;         // Should equal (1 - GYRO_WEIGHT)

// ======================================================

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

  Serial.println("ESP32 Air Mouse Starting...");
  Serial.println("Current Settings:");
  Serial.print("  Cursor Speed: "); Serial.println(CURSOR_SPEED);
  Serial.print("  Deadzone: "); Serial.println(DEADZONE);
  Serial.print("  Tap Threshold: "); Serial.println(TAP_THRESHOLD);
  Serial.print("  Twist Threshold: "); Serial.println(TWIST_THRESHOLD);
  Serial.print("  Gesture Cooldown: "); Serial.print(GESTURE_COOLDOWN); Serial.println("ms");

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

  // Complementary filter with adjustable weights
  pitch = GYRO_WEIGHT * (pitch + gyroPitchRate * dt) + ACCEL_WEIGHT * accelPitch;
  roll  = GYRO_WEIGHT * (roll  + gyroRollRate  * dt) + ACCEL_WEIGHT * accelRoll;

  // ---- CURSOR MOVEMENT ----
  int moveX = 0;
  int moveY = 0;

  if (abs(roll) > DEADZONE)
    moveX = roll * CURSOR_SPEED;

  if (abs(pitch) > DEADZONE)
    moveY = -pitch * CURSOR_SPEED;

  bleMouse.move(moveX, moveY);

  // ---- LEFT CLICK (Air Tap) ----
  if (fabs(accel.acceleration.z) > TAP_THRESHOLD &&
      millis() - lastTapTime > GESTURE_COOLDOWN) {

    bleMouse.click(MOUSE_LEFT);
    Serial.println("LEFT CLICK");
    lastTapTime = millis();
  }

  // ---- RIGHT CLICK (Wrist Twist) ----
  if (fabs(gyroZRate) > TWIST_THRESHOLD &&
      millis() - lastTwistTime > GESTURE_COOLDOWN) {

    bleMouse.click(MOUSE_RIGHT);
    Serial.println("RIGHT CLICK");
    lastTwistTime = millis();
  }

  // ---- SCROLL ----
  if (pitch > SCROLL_ANGLE) {
    bleMouse.move(0, 0, -1 * SCROLL_SPEED);   // Scroll down
  }
  else if (pitch < -SCROLL_ANGLE) {
    bleMouse.move(0, 0, 1 * SCROLL_SPEED);    // Scroll up
  }

  delay(10);
}
