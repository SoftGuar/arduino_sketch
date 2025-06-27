#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

MPU6050 mpu(Wire);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

float yawFiltered = 0;
float yawMag = 0;
float alpha = 0.98;

// Magnetometer Calibration
float magBiasX = 2.14;
float magBiasY = -8.41;
float magBiasZ = -15.00;
float scaleX = 0.837;
float scaleY = 0.968;
float scaleZ = 1.295;

float magOffset = NAN;
// float gyroYawBias = 12.85;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mag.begin()) {
    Serial.println("HMC5883L connection failed!");
    while (1);
  }
  Serial.println("HMC5883L Magnetometer Ready!");

  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU6050 init failed with code: ");
    Serial.println(status);
    while (1);
  }
  Serial.println("MPU6050 ready!");

  delay(1000);
  mpu.calcOffsets();  // Calibrate gyro/accel
}

void loop() {
  mpu.update();

  sensors_event_t event;
  mag.getEvent(&event);

  float mx = (event.magnetic.x - magBiasX) * scaleX;
  float my = (event.magnetic.y - magBiasY) * scaleY;
  yawMag = atan2(my, mx) * 180 / M_PI;
  if (yawMag < 0) yawMag += 360.0;

  if (isnan(magOffset)) {
    magOffset = yawMag;
  }

  float rawYaw = mpu.getAngleZ();
  if (rawYaw < 0) rawYaw += 360.0;

  float correctedYaw = rawYaw + magOffset;
  if (correctedYaw < 0.0) correctedYaw += 360.0;
  if (correctedYaw >= 360.0) correctedYaw -= 360.0;

  yawFiltered = alpha * yawFiltered + (1 - alpha) * rawYaw;
  if (yawFiltered >= 360.0) yawFiltered -= 360.0;
  if (yawFiltered < 0.0) yawFiltered += 360.0;

  float accX_g = mpu.getAccX();
  float accY_g = mpu.getAccY();
  float accZ_g = mpu.getAccZ() - 1;

  Serial.print("Gyro Yaw: "); Serial.print(correctedYaw);
  Serial.print(" | Mag Yaw: "); Serial.print(yawMag);
  Serial.print(" | AccX: "); Serial.print(accX_g);
  Serial.print(" | AccY: "); Serial.print(accY_g);
  Serial.print(" | AccZ: "); Serial.println(accZ_g);

  delay(10);
}
