#pragma once
#include "MPU6500.h"

class IMU {
public:
    IMU();
    void begin();
    void update();
    float getPitch();
    float getRoll();
private:
    MPU6500 mpu;
    float pitch, roll;
};
