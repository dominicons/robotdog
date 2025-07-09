#include "IMU10DOF.h"
#include <Wire.h>
#include <math.h>
#include <Arduino.h>

#define ADXL345_ADDR 0x53
#define L3G4200D_ADDR 0x69  // Gyroscope address
#define TO_RAD 0.0174533f
#define ACC_SCALE 0.0039f // scale to g/LSB for ±2g
#define GYRO_SCALE 0.07f  // scale to deg/s/LSB for ±250dps

// Offset hiệu chỉnh khi robot đứng vuông góc
static constexpr float pitch_offset = 3.17f;
static constexpr float roll_offset  = 2.02f;

IMU10DOF::IMU10DOF() : pitch(0), roll(0), yaw(0) {
    initKalman(kalmanPitch);
    initKalman(kalmanRoll);
}

void IMU10DOF::initKalman(KalmanFilter &kalman) {
    kalman.angle = 0.0f;
    kalman.bias = 0.0f;
    kalman.P[0][0] = 0.0f;
    kalman.P[0][1] = 0.0f;
    kalman.P[1][0] = 0.0f;
    kalman.P[1][1] = 0.0f;
    kalman.Q_angle = 0.001f;    // Process noise variance for the accelerometer
    kalman.Q_bias = 0.003f;     // Process noise variance for the gyro bias
    kalman.R_measure = 0.03f;   // Measurement noise variance
}

bool IMU10DOF::begin() {
    Wire.begin();
    
    // Initialize ADXL345 (Accelerometer)
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(0x2D); // Power register
    Wire.write(0x08); // Measurement mode
    if (Wire.endTransmission() != 0) return false;
    
    // Initialize L3G4200D (Gyroscope)
    Wire.beginTransmission(L3G4200D_ADDR);
    Wire.write(0x20); // CTRL_REG1
    Wire.write(0x0F); // Normal mode, enable all axes
    if (Wire.endTransmission() != 0) {
        Serial.println(F("[WARNING] Gyroscope not found, using accelerometer only"));
    }
    
    return true;
}

bool IMU10DOF::readAccelerometer(float &ax, float &ay, float &az) {
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(0x32); // Data start register
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom(ADXL345_ADDR, 6);
    if (Wire.available() < 6) return false;

    int16_t rawX = Wire.read() | (Wire.read() << 8);
    int16_t rawY = Wire.read() | (Wire.read() << 8);
    int16_t rawZ = Wire.read() | (Wire.read() << 8);

    ax = rawX * ACC_SCALE;
    ay = rawY * ACC_SCALE;
    az = rawZ * ACC_SCALE;
    return true;
}

bool IMU10DOF::readGyroscope(float &gx, float &gy, float &gz) {
    Wire.beginTransmission(L3G4200D_ADDR);
    Wire.write(0x28 | 0x80); // Start with OUT_X_L register and auto-increment
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom(L3G4200D_ADDR, 6);
    if (Wire.available() < 6) return false;

    int16_t rawX = Wire.read() | (Wire.read() << 8);
    int16_t rawY = Wire.read() | (Wire.read() << 8);
    int16_t rawZ = Wire.read() | (Wire.read() << 8);

    gx = rawX * GYRO_SCALE;
    gy = rawY * GYRO_SCALE;
    gz = rawZ * GYRO_SCALE;
    return true;
}

float IMU10DOF::kalmanUpdate(KalmanFilter &kalman, float newAngle, float newRate, float dt) {
    // Predict
    kalman.angle += dt * (newRate - kalman.bias);
    kalman.P[0][0] += dt * (dt * kalman.P[1][1] - kalman.P[0][1] - kalman.P[1][0] + kalman.Q_angle);
    kalman.P[0][1] -= dt * kalman.P[1][1];
    kalman.P[1][0] -= dt * kalman.P[1][1];
    kalman.P[1][1] += kalman.Q_bias * dt;

    // Update
    float S = kalman.P[0][0] + kalman.R_measure;
    float K[2];
    K[0] = kalman.P[0][0] / S;
    K[1] = kalman.P[1][0] / S;

    float y = newAngle - kalman.angle;
    kalman.angle += K[0] * y;
    kalman.bias += K[1] * y;

    float P00_temp = kalman.P[0][0];
    float P01_temp = kalman.P[0][1];

    kalman.P[0][0] -= K[0] * P00_temp;
    kalman.P[0][1] -= K[0] * P01_temp;
    kalman.P[1][0] -= K[1] * P00_temp;
    kalman.P[1][1] -= K[1] * P01_temp;

    return kalman.angle;
}

void IMU10DOF::update(float dt) {
    float ax, ay, az;
    if (!readAccelerometer(ax, ay, az)) {
        Serial.println(F("[ERROR] Failed to read ADXL345"));
        return;
    }

    // Normalize accelerometer data
    float norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0) return; // avoid division by zero

    ax /= norm;
    ay /= norm;
    az /= norm;

    // Calculate angles from accelerometer (degrees)
    float pitchAcc = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / M_PI;
    float rollAcc = atan2(-ax, az) * 180.0f / M_PI;

    // Try to read gyroscope
    float gx, gy, gz;
    bool hasGyro = readGyroscope(gx, gy, gz);

    if (hasGyro) {
        // Use Kalman filter with gyroscope data
        pitch = kalmanUpdate(kalmanPitch, pitchAcc, gy, dt);
        roll = kalmanUpdate(kalmanRoll, rollAcc, gx, dt);
    } else {
        // Fallback to complementary filter if no gyroscope
        const float alpha = 0.98f;
        pitch = alpha * pitch + (1.0f - alpha) * pitchAcc;
        roll = alpha * roll + (1.0f - alpha) * rollAcc;
    }

    // Clamp output for sanity
    pitch = constrain(pitch, -90.0f, 90.0f);
    roll = constrain(roll, -90.0f, 90.0f);

    // Yaw requires magnetometer for absolute heading
    yaw = 0.0f;
}

// Đảo dấu để khi robot nghiêng về phía trước/trái là dương (chuẩn cho robot 4 chân)
float IMU10DOF::getPitch() { return -(pitch - pitch_offset); }
float IMU10DOF::getRoll()  { return -(roll - roll_offset); }
float IMU10DOF::getYaw()   { return yaw; }
