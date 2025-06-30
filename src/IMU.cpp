#include "IMU.h"
#include <Wire.h>
#include <MPU6500.h> // Thêm thư viện phù hợp nếu có

IMU::IMU() : pitch(0), roll(0) {}

void IMU::begin() {
    // Khởi tạo MPU6500
    Wire.begin();
    mpu.begin();
}

void IMU::update() {
    // Đọc dữ liệu từ MPU6500, cập nhật pitch, roll
    mpu.readAngle(roll, pitch);
}

float IMU::getPitch() { return pitch; }
float IMU::getRoll() { return roll; }
