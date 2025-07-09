#pragma once
#include <stdint.h>

class Gait {
public:
    // Các tham số cấu hình động
    float speed = 1.0f;      // Tốc độ tổng thể
    float stepLength = 30.0f; // Biên độ bước (mm hoặc độ)
    float stepHeight = 20.0f; // Độ nâng chân (mm hoặc độ)
    float cycleTime = 800.0f; // Thời gian 1 chu kỳ (ms)
    float phaseOffset[4] = {0.0f, 0.5f, 0.25f, 0.75f}; // Pha từng chân (chu kỳ)

    Gait();
    void setSpeed(float s);
    void setStepLength(float l);
    void setStepHeight(float h);
    void setCycleTime(float t);
    void setPhaseOffset(int leg, float offset);

    // Hàm sinh chuyển động cho từng chân (theo thời gian t)
    void getLegTarget(int leg, float t, float &x, float &y, float &z);

    // Bézier cubic cho quỹ đạo chân
    float bezier(float t, float P0, float P1, float P2, float P3);

    // Động học thuận: từ góc khớp -> vị trí (x, y, z)
    static void forwardKinematics(float theta1, float theta2, float theta3, float &x, float &y, float &z);
    // Động học nghịch: từ vị trí (x, y, z) -> góc khớp
    static bool inverseKinematics(float x, float y, float z, float &theta1, float &theta2, float &theta3);
    // IK 2 bậc tự do: chỉ giải khuỷu và đầu gối
    static bool inverseKinematics2DOF(float x, float y, float z, float &theta2, float &theta3);

    // Jacobian: cho 1 chân, trả về ma trận 3x3 (giản lược, demo)
    static void jacobian(float theta1, float theta2, float theta3, float J[3][3]);
};
