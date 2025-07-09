#include "Gait.h"
#include <math.h>

// IK 2 bậc tự do: chỉ giải khuỷu và đầu gối (robot chân phẳng)
bool Gait::inverseKinematics2DOF(float x, float y, float z, float &theta2, float &theta3) {
    // Chỉ dùng x, z (mặt phẳng dọc chân)
    const float L1 = 50.0f; // mm, chiều dài đùi
    const float L2 = 50.0f; // mm, chiều dài cẳng
    
    // Tính khoảng cách từ khớp vai đến điểm đích
    float d = sqrt(x * x + z * z);
    
    // Kiểm tra workspace
    if (d > (L1 + L2) || d < fabs(L1 - L2)) {
        return false; // Ngoài tầm với
    }
    
    // Tính góc theo công thức IK 2 khớp
    float cos_theta3 = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (cos_theta3 < -1.0f || cos_theta3 > 1.0f) {
        return false; // Không thể giải
    }
    
    // Góc đầu gối (theta3)
    float theta3_rad = acos(cos_theta3);
    theta3 = theta3_rad * 180.0f / M_PI;
    
    // Góc khủy (theta2)
    float alpha = atan2(z, x);
    float beta = atan2(L2 * sin(theta3_rad), L1 + L2 * cos(theta3_rad));
    float theta2_rad = alpha - beta;
    theta2 = theta2_rad * 180.0f / M_PI;
    
    // Chuyển đổi sang hệ tọa độ servo (0-180 độ)
    theta2 = 90.0f - theta2; // Khủy: 90° là thẳng đứng
    theta3 = 180.0f - theta3; // Đầu gối: 180° là thẳng
    
    return true;
}
#include "Gait.h"
#include <math.h>

Gait::Gait() {}

void Gait::setSpeed(float s) { speed = s; }
void Gait::setStepLength(float l) { stepLength = l; }
void Gait::setStepHeight(float h) { stepHeight = h; }
void Gait::setCycleTime(float t) { cycleTime = t; }
void Gait::setPhaseOffset(int leg, float offset) { if (leg >= 0 && leg < 4) phaseOffset[leg] = offset; }

// Hàm sinh chuyển động cho từng chân (theo thời gian t, trả về x/y/z mục tiêu)
void Gait::getLegTarget(int leg, float t, float &x, float &y, float &z) {
    // Tính phase cho chân hiện tại
    float phase = fmod(t / cycleTime + phaseOffset[leg], 1.0f);
    if (phase < 0) phase += 1.0f;
    
    // Trot gait: 50% stance, 50% swing
    if (phase < 0.5f) {
        // STANCE PHASE (chân chạm đất, đẩy thân về phía trước)
        float stance_t = phase * 2.0f; // 0 -> 1
        x = stepLength * (0.5f - stance_t); // Từ +stepLength/2 -> -stepLength/2
        y = 0;
        z = 0; // Chân chạm đất
    } else {
        // SWING PHASE (chân nâng lên, di chuyển về phía trước)
        float swing_t = (phase - 0.5f) * 2.0f; // 0 -> 1
        
        // Quỹ đạo X: từ -stepLength/2 -> +stepLength/2
        x = stepLength * (swing_t - 0.5f);
        
        // Quỹ đạo Z: hình parabol mượt mà
        float z_factor = 4.0f * swing_t * (1.0f - swing_t); // Parabol 0->1->0
        z = stepHeight * z_factor;
        
        y = 0;
    }
    
    // Áp dụng speed multiplier
    x *= speed;
}

// Bézier cubic cho quỹ đạo chân
float Gait::bezier(float t, float P0, float P1, float P2, float P3) {
    float u = 1 - t;
    float tt = t * t;
    float uu = u * u;
    float uuu = uu * u;
    float ttt = tt * t;
    return (uuu * P0) + (3 * uu * t * P1) + (3 * u * tt * P2) + (ttt * P3);
}

// Động học thuận: demo cho robot 3 khớp/phẳng (giả sử các chiều dài link đều = 1)
void Gait::forwardKinematics(float theta1, float theta2, float theta3, float &x, float &y, float &z) {
    // Đơn giản hóa: chỉ tính x, z (robot 2D)
    float l1 = 1.0f, l2 = 1.0f, l3 = 1.0f;
    float t1 = theta1 * M_PI / 180.0f;
    float t2 = theta2 * M_PI / 180.0f;
    float t3 = theta3 * M_PI / 180.0f;
    x = l1 * cosf(t1) + l2 * cosf(t1 + t2) + l3 * cosf(t1 + t2 + t3);
    y = 0;
    z = l1 * sinf(t1) + l2 * sinf(t1 + t2) + l3 * sinf(t1 + t2 + t3);
}

// Động học nghịch: demo cho robot 3 khớp/phẳng (giả sử các chiều dài link đều = 1)
bool Gait::inverseKinematics(float x, float y, float z, float &theta1, float &theta2, float &theta3) {
    // Đơn giản hóa: chỉ tính x, z (robot 2D)
    float l1 = 1.0f, l2 = 1.0f, l3 = 1.0f;
    float d = sqrtf(x * x + z * z);
    if (d > l1 + l2 + l3) return false;
    // Demo: chỉ giải cho trường hợp đơn giản, thực tế cần giải hệ phương trình
    theta1 = atan2f(z, x) * 180.0f / M_PI;
    theta2 = 0;
    theta3 = 0;
    return true;
}

// Jacobian demo (giản lược, thực tế cần tính đạo hàm động học)
void Gait::jacobian(float theta1, float theta2, float theta3, float J[3][3]) {
    // Demo: ma trận đơn vị
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            J[i][j] = (i == j) ? 1.0f : 0.0f;
}
