# Arduino Sensor Fusion and Obstacle Detection

This repository contains two Arduino sketches:

1. **IMU + Magnetometer Fusion**  
2. **Ultrasonic Obstacle Detection**  

These are designed for sensor-based robotics or navigation systems that combine orientation sensing (yaw estimation) and distance-based obstacle detection.

---

## 📜 Contents

- `arduino_sketch_yaw_acc.ino`
- `obstacle_detection.ino`

---

## 1️⃣ IMU + Magnetometer Fusion

**File:** `arduino_sketch_yaw_acc.ino`  

### ✨ Description
This sketch reads orientation data from:
- MPU6050 (gyroscope + accelerometer)
- HMC5883L magnetometer

It fuses the gyroscope yaw with magnetometer heading using a complementary filter to reduce drift. The output includes:

- Gyro-corrected yaw
- Magnetometer yaw
- Acceleration on X, Y, Z axes (gravity compensated)

### ⚙️ Hardware
- Arduino Uno (or compatible)
- MPU6050 IMU module
- HMC5883L Magnetometer module
- I2C connections

### 📈 Output Example (Serial)


Gyro Yaw: 123.45 | Mag Yaw: 120.32 | AccX: 0.01 | AccY: -0.02 | AccZ: -0.98



### 🛠️ Notes
- Includes hard-iron and soft-iron calibration for the magnetometer (bias and scale factors).
- Uses a complementary filter (`alpha = 0.98`) to blend gyroscope and magnetometer yaw.

---

## 2️⃣ Ultrasonic Obstacle Detection

**File:** `obstacle_detection.ino`

### ✨ Description
Reads distance from an HC-SR04 ultrasonic sensor to detect obstacles. Implements a simple debounce strategy:

- Requires a set number of **consecutive detections** below a threshold before declaring an obstacle.
- Sends serial commands:
  - `stop`
  - `continue`
  - `obstacle:<distance>`

### ⚙️ Hardware
- Arduino Uno (or compatible)
- HC-SR04 Ultrasonic Distance Sensor
- Digital pins (customizable):
  - `trigPin = 9`
  - `echoPin = 10`

### 📈 Behavior
- Distance measured in centimeters.
- Obstacle threshold: 50 cm (adjustable).
- Requires 2 consecutive detections below threshold to confirm obstacle.
- Example Serial output:
```

stop
obstacle:45
continue

````

### 🛠️ Features
- Reduces false positives via consecutive detection count.
- Adjustable:
  - `detectionThreshold`
  - `triesBeforeObstacle`
  - `checkingInterval`

---

## 🚀 Getting Started

1️⃣ Clone this repository:

```bash
git clone https://github.com/yourusername/your-repo-name.git
````

2️⃣ Open either `.ino` file in the Arduino IDE.

3️⃣ Install dependencies (via Library Manager):

* MPU6050\_light
* Adafruit HMC5883 Unified
* Adafruit Sensor

4️⃣ Connect your hardware per sketch instructions.

5️⃣ Upload to your Arduino.

---

## 📌 Dependencies

* [MPU6050\_light](https://github.com/richardtechnologies/MPU6050_light)
* [Adafruit Sensor Library](https://github.com/adafruit/Adafruit_Sensor)
* [Adafruit HMC5883L Unified](https://github.com/adafruit/Adafruit_HMC5883_Unified)

---





