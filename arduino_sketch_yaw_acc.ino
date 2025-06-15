#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

float ypr[3];  // Yaw, Pitch, Roll
float yawFiltered = 0;
float yawMag = 0;
float alpha = 0.98; // Complementary filter factor

// Magnetometer Calibration Parameters
float magBiasX = 2.14;
float magBiasY = -8.41;
float magBiasZ = -15.00;
float scaleX = 0.837;
float scaleY = 0.968;
float scaleZ = 1.295;

float magOffset = NAN;
float gyroYawBias = 12.85;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }

    if (!mag.begin()) {
        Serial.println("HMC5883L connection failed!");
        while (1);
    }
    Serial.println("HMC5883L Magnetometer Ready!");

    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(false);
        mpu.resetFIFO();
        mpu.setDMPEnabled(true);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println("DMP ready!");
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }
}

void loop() {
    if (!dmpReady) return;

    fifoCount = mpu.getFIFOCount();
    if (fifoCount >= 1024) {
        while (fifoCount >= packetSize) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }
    }

    if (fifoCount >= packetSize) {
        while (fifoCount >= packetSize) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;
        }

        Quaternion q;
        VectorFloat gravity;
        int16_t accX, accY, accZ;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.getAcceleration(&accX, &accY, &accZ);

        sensors_event_t event;
        mag.getEvent(&event);

        float mx = (event.magnetic.x - magBiasX) * scaleX;
        float my = (event.magnetic.y - magBiasY) * scaleY;
        yawMag = atan2(my, mx) * 180 / M_PI;
        if (yawMag < 0) yawMag += 360.0;

        if (isnan(magOffset)) {
          magOffset = yawMag;
        }


        float rawYaw = ypr[0] * 180 / M_PI;
        if (rawYaw < 0) rawYaw += 360.0;
        float correctedYaw = rawYaw - gyroYawBias + magOffset;
        if (correctedYaw < 0.0) {
          correctedYaw += 360.0;
        }
        else if (correctedYaw >= 360.0) {
          correctedYaw -= 360.0;
        }

        yawFiltered = alpha * yawFiltered + (1 - alpha) * rawYaw;
        if (yawFiltered >= 360.0) yawFiltered -= 360.0;
        if (yawFiltered < 0.0) yawFiltered += 360.0;

        float accX_g = accX / 16384.0 - gravity.x;
        float accY_g = accY / 16384.0 - gravity.y;
        float accZ_g = accZ / 16384.0 - gravity.z;

        Serial.print("Gyro Yaw: "); Serial.print(correctedYaw);
        Serial.print(" | Mag Yaw: "); Serial.print(yawMag);
        Serial.print(" | AccX: "); Serial.print(accX_g);
        Serial.print(" | AccY: "); Serial.print(accY_g);
        Serial.print(" | AccZ: "); Serial.println(accZ_g);

        delay(10);
    }
}