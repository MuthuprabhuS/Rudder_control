#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

// Complementary filter constants
const float alpha = 0.98;

float pitch = 0;
float roll = 0;
float yaw = 0;

unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();

  Serial.println("Calibrating gyroscope... Keep the sensor still.");
  mpu6050.calcGyroOffsets(true);
  Serial.println("Calibration complete.");

  prevTime = millis();
}

void loop() {
  mpu6050.update();

  // Calculate delta time
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0; // in seconds
  prevTime = currentTime;

  // Read raw values
  float accX = mpu6050.getAccX();
  float accY = mpu6050.getAccY();
  float accZ = mpu6050.getAccZ();
  float gyroX = mpu6050.getGyroX(); // deg/s
  float gyroY = mpu6050.getGyroY();
  float gyroZ = mpu6050.getGyroZ();
  float temp = mpu6050.getTemp();   // °C

  // Calculate pitch & roll from accelerometer
  float accPitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180 / PI;
  float accRoll = atan2(-accX, accZ) * 180 / PI;

  // Complementary filter for pitch and roll
  pitch = alpha * (pitch + gyroX * deltaTime) + (1 - alpha) * accPitch;
  roll  = alpha * (roll  + gyroY * deltaTime) + (1 - alpha) * accRoll;

  // Yaw from gyroZ integration
  yaw += gyroZ * deltaTime;

  // Wrap yaw within 0–360
  
  if (yaw < 0) yaw += 360;
  if (yaw >= 360) yaw -= 360;

  // === Print results ===
  Serial.print("Pitch: "); Serial.print(pitch, 2);
  Serial.print(" | Roll: "); Serial.print(roll, 2);
  Serial.print(" | Yaw: "); Serial.print(yaw, 2);
  Serial.print(" | Temp: "); Serial.print(temp, 2); Serial.println(" °C");

  delay(10); // Loop runs every ~10ms
}
