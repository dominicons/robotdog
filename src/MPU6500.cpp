// MPU6500.cpp
#include "MPU6500.h"

bool MPU6500::begin() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    return Wire.endTransmission() == 0;
}

void MPU6500::readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)14);

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // skip temp
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
}

void MPU6500::readAngle(float &roll, float &pitch) {
    int16_t ax, ay, az, gx, gy, gz;
    readRaw(ax, ay, az, gx, gy, gz);

    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;

    float gyroX = gx / 131.0;
    float gyroY = gy / 131.0;

    float rollAcc = atan2(accelY, accelZ) * RAD_TO_DEG;
    float pitchAcc = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;

    unsigned long now = millis();
    float dt = (now - _lastTime) / 1000.0;
    _lastTime = now;

    if (dt > 0.05 || dt <= 0) dt = 0.01;  // fallback

    _angleX = 0.98 * (_angleX + gyroX * dt) + 0.02 * rollAcc;
    _angleY = 0.98 * (_angleY + gyroY * dt) + 0.02 * pitchAcc;

    roll = _angleX;
    pitch = _angleY;
}
