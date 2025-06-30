// MPU6500.h
#ifndef MPU6500_H
#define MPU6500_H

#include <Arduino.h>
#include <Wire.h>

class MPU6500 {
public:
    bool begin();
    void readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);
    void readAngle(float &roll, float &pitch);

private:
    const uint8_t MPU_ADDR = 0x68;
    unsigned long _lastTime = 0;
    float _angleX = 0, _angleY = 0;
};

#endif
