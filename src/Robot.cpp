#include "Robot.h"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

// Hiệu chỉnh trung tính từng khớp cho từng chân
const int vaiTrungTinh[4]    = {90, 45, 90, 90}; //VAI   0 3 6 9
const int khuyuTrungTinh[4]  = {90, 95, 90, 90}; //KHUỶU 1 4 7 10
const int dauGoiTrungTinh[4] = {90, 90, 90, 90};// //ĐẦU 2 5 8 11

Robot::Robot() : pwm(Adafruit_PWMServoDriver()), pitchErrorSum(0), rollErrorSum(0), lastPitchError(0), lastRollError(0), lastTime(0) {
    int tmp[4][3] = {{0,1,2},{3,4,5},{6,7,8},{9,10,11}};
    memcpy(servoChannels, tmp, sizeof(servoChannels));
}

void Robot::begin() {
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(50);
    lastTime = millis();
    standStill();
}

void Robot::setServoAngle(int channel, int angle) {
    static int lastAngles[16] = {0}; // Giả sử tối đa 16 kênh
    angle = constrain(angle, 0, 180);
    if (abs(lastAngles[channel] - angle) > 1) { // Chỉ cập nhật nếu thay đổi đủ lớn
        int pulse = map(angle, 0, 180, servoMin, servoMax);
        pwm.setPWM(channel, 0, pulse);
        lastAngles[channel] = angle;
    }
}

float Robot::bezier(float t, float P0, float P1, float P2, float P3) {
    float u = 1 - t;
    float tt = t * t;
    float uu = u * u;
    float uuu = uu * u;
    float ttt = tt * t;
    return (uuu * P0) + (3 * uu * t * P1) + (3 * u * tt * P2) + (ttt * P3);
}

void Robot::smoothMove(int channel, int startAngle, int endAngle, int duration, int steps) {
    if (steps < 2) steps = 2;
    float P0 = startAngle;
    float P3 = endAngle;
    float P1 = P0 + 0.2 * (P3 - P0);  // Điều chỉnh để mượt hơn
    float P2 = P0 + 0.8 * (P3 - P0);  // Điều chỉnh để mượt hơn
    for (int i = 0; i < steps; i++) {
        float t = (float)i / (steps - 1);
        int angle = round(bezier(t, P0, P1, P2, P3));
        setServoAngle(channel, angle);
        delay(duration / steps);
    }
}

// Hàm smoothMove cũ để tương thích với code cũ
void Robot::smoothMove(int channel, int startAngle, int endAngle, int duration) {
    smoothMove(channel, startAngle, endAngle, duration, 50);
}

void Robot::standStill() {
    for (int leg = 0; leg < 4; leg++) {
        setServoAngle(servoChannels[leg][0], vaiTrungTinh[leg]);
        setServoAngle(servoChannels[leg][1], khuyuTrungTinh[leg]);
        setServoAngle(servoChannels[leg][2], dauGoiTrungTinh[leg]);
    }
}

void Robot::walkForward(int stepDelay) {
    // Tăng stepDelay mặc định nếu chưa truyền vào
    if (stepDelay < 200) stepDelay = 300;

    // Bước 1: Chân trái trước (0) nâng lên, giữ nguyên vai
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    smoothMove(servoChannels[0][1], khuyuTrungTinh[0], khuyuTrungTinh[0] + 20);
    smoothMove(servoChannels[0][2], dauGoiTrungTinh[0], dauGoiTrungTinh[0] - 35);
    delay(stepDelay);
    balance(imu.getPitch(), imu.getRoll());

    // Bước 2: Hạ chân trái trước, nâng chân phải sau (3), giữ nguyên vai
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    smoothMove(servoChannels[0][1], khuyuTrungTinh[0] + 20, khuyuTrungTinh[0]);
    smoothMove(servoChannels[0][2], dauGoiTrungTinh[0] - 35, dauGoiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    smoothMove(servoChannels[3][1], khuyuTrungTinh[3], khuyuTrungTinh[3] + 20);
    smoothMove(servoChannels[3][2], dauGoiTrungTinh[3], dauGoiTrungTinh[3] - 35);
    delay(stepDelay);
    balance(imu.getPitch(), imu.getRoll());

    // Bước 3: Hạ chân phải sau, nâng chân phải trước (1), giữ nguyên vai
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    smoothMove(servoChannels[3][1], khuyuTrungTinh[3] + 20, khuyuTrungTinh[3]);
    smoothMove(servoChannels[3][2], dauGoiTrungTinh[3] - 35, dauGoiTrungTinh[3]);
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    smoothMove(servoChannels[1][1], khuyuTrungTinh[1], khuyuTrungTinh[1] + 20);
    smoothMove(servoChannels[1][2], dauGoiTrungTinh[1], dauGoiTrungTinh[1] - 35);
    delay(stepDelay);
    balance(imu.getPitch(), imu.getRoll());

    // Bước 4: Hạ chân phải trước, nâng chân trái sau (2), giữ nguyên vai
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    smoothMove(servoChannels[1][1], khuyuTrungTinh[1] + 20, khuyuTrungTinh[1]);
    smoothMove(servoChannels[1][2], dauGoiTrungTinh[1] - 35, dauGoiTrungTinh[1]);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    smoothMove(servoChannels[2][1], khuyuTrungTinh[2], khuyuTrungTinh[2] + 20);
    smoothMove(servoChannels[2][2], dauGoiTrungTinh[2], dauGoiTrungTinh[2] - 35);
    delay(stepDelay);
    balance(imu.getPitch(), imu.getRoll());

    // Bước 5: Hạ chân trái sau, giữ nguyên vai
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    smoothMove(servoChannels[2][1], khuyuTrungTinh[2] + 20, khuyuTrungTinh[2]);
    smoothMove(servoChannels[2][2], dauGoiTrungTinh[2] - 35, dauGoiTrungTinh[2]);
    delay(stepDelay);
    balance(imu.getPitch(), imu.getRoll());
}

void Robot::walkForwardOverlap(int stepDelay, int liftAngle, int pushAngle) {
    // Thuật toán mới: sử dụng flow stepDiagonalPair nhưng cho phép overlap 2 cặp chân với 3 pha bán nguyệt
    // stepDelay: tổng thời gian cho mỗi cặp chân
    int pair[2][2] = { {0, 3}, {1, 2} };
    int phaseTime = stepDelay / 3;
    int stepHeight = 35; // Có thể thành tham số nếu cần

    for (int phase = 0; phase < 2; ++phase) {
        int legA = pair[phase][0];
        int legB = pair[phase][1];

        // PHA 1: SWING (nâng khủy, đầu gối hạ tạo bán nguyệt)
        setServoAngle(servoChannels[legA][0], vaiTrungTinh[legA]);
        setServoAngle(servoChannels[legB][0], vaiTrungTinh[legB]);
        for (int i = 0; i <= liftAngle; i += 2) {
            setServoAngle(servoChannels[legA][1], khuyuTrungTinh[legA] - i);
            setServoAngle(servoChannels[legB][1], khuyuTrungTinh[legB] + i);
            setServoAngle(servoChannels[legA][2], dauGoiTrungTinh[legA] + (i * stepHeight / liftAngle));
            setServoAngle(servoChannels[legB][2], dauGoiTrungTinh[legB] - (i * stepHeight / liftAngle));
            delay(phaseTime / (liftAngle/2));
        }
        // PHA 2: PUSH (mở thêm khủy, đầu gối trả về trung tính)
        for (int i = 0; i <= pushAngle; i += 2) {
            setServoAngle(servoChannels[legA][1], khuyuTrungTinh[legA] - liftAngle - i);
            setServoAngle(servoChannels[legB][1], khuyuTrungTinh[legB] + liftAngle + i);
            setServoAngle(servoChannels[legA][2], dauGoiTrungTinh[legA] + ((liftAngle - i) * stepHeight / liftAngle));
            setServoAngle(servoChannels[legB][2], dauGoiTrungTinh[legB] - ((liftAngle - i) * stepHeight / liftAngle));
            delay(phaseTime / (pushAngle/2));
        }
        // PHA 3: RETURN (hạ khủy về trung tính, đầu gối về trung tính)
        for (int i = liftAngle + pushAngle; i >= 0; i -= 2) {
            setServoAngle(servoChannels[legA][1], khuyuTrungTinh[legA] - i);
            setServoAngle(servoChannels[legB][1], khuyuTrungTinh[legB] + i);
            setServoAngle(servoChannels[legA][2], dauGoiTrungTinh[legA]);
            setServoAngle(servoChannels[legB][2], dauGoiTrungTinh[legB]);
            delay(phaseTime / ((liftAngle+pushAngle)/2));
        }
    }
}

// ===== CONTINUOUS OVERLAP GAIT =====
static float overlapPhaseOffset[4] = {0.0f, 0.5f, 0.0f, 0.5f}; // 0&2 cùng pha, 1&3 cùng pha (trot overlap)
bool overlapWalking = false;
unsigned long overlapStartTime = 0;

void Robot::startOverlapGait(float cycle_ms, float step_len, float step_h) {
    gait.setCycleTime(cycle_ms);
    gait.setStepLength(step_len);
    gait.setStepHeight(step_h);
    // Phase: 0&2 cùng pha, 1&3 cùng pha (trot overlap)
    gait.setPhaseOffset(0, 0.0f);
    gait.setPhaseOffset(2, 0.0f);
    gait.setPhaseOffset(1, 0.5f);
    gait.setPhaseOffset(3, 0.5f);
    overlapWalking = true;
    overlapStartTime = millis();
}

void Robot::stopOverlapGait() {
    overlapWalking = false;
    standStill();
}

void Robot::updateOverlapGait() {
    if (!overlapWalking) return;
    unsigned long now = millis();
    float t = float((now - overlapStartTime) % (unsigned long)gait.cycleTime);
    for (int leg = 0; leg < 4; ++leg) {
        float x, y, z;
        float phase = fmod(t / gait.cycleTime + gait.phaseOffset[leg], 1.0f);
        gait.getLegTarget(leg, t, x, y, z);
        // Body sway: nếu stance (phase < 0.5), hạ khuỷu stance, nếu swing (phase >= 0.5), nâng khuỷu swing
        if (phase < 0.5f) {
            z -= 10; // body sway stance
        } else {
            z += 10; // body sway swing
        }
        float theta2, theta3;
        if (Gait::inverseKinematics2DOF(x, y, z, theta2, theta3)) {
            int angle_khuyu = constrain(int(theta2), 0, 180);
            int angle_dauGoi = constrain(int(theta3), 0, 180);
            angle_khuyu = constrain(khuyuTrungTinh[leg] + (angle_khuyu - 90), 0, 180);
            angle_dauGoi = constrain(dauGoiTrungTinh[leg] + (angle_dauGoi - 90), 0, 180);
            setServoAngle(servoChannels[leg][1], angle_khuyu);
            setServoAngle(servoChannels[leg][2], angle_dauGoi);
        }
        setServoAngle(servoChannels[leg][0], vaiTrungTinh[leg]);
    }
}

void Robot::stepDiagonalPair(int stepDelay, int lift, int knee) {
    int step = 4; // tăng bước để giảm số lần lặp, tránh block
    int stride = knee; // dùng biên stride cho đầu gối stance
    int n = lift / step;
    int delayPerStep = max(15, stepDelay / n); // đảm bảo delay tối thiểu

    // --- Pha 1: 0-3 swing về trước, 1-2 stance đạp về sau ---
    for (int i = 0; i <= lift; i += step) {
        // 0-3 swing
        setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
        setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
        setServoAngle(servoChannels[0][1], khuyuTrungTinh[0] - i);
        setServoAngle(servoChannels[3][1], khuyuTrungTinh[3] + i);
        setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0] + (i * knee / lift));
        setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3] - (i * knee / lift));
        // 1-2 stance
        setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
        setServoAngle(servoChannels[2][0], vaiTrungTinh[2] );
        setServoAngle(servoChannels[1][1], khuyuTrungTinh[1]+ stride/3);
        setServoAngle(servoChannels[2][1], khuyuTrungTinh[2]+ stride/3);
        setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1] - stride);
        setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2] + stride);
        delay(delayPerStep);
    }
    // --- Pha 2: 0-3 hạ xuống trung tính, 1-2 thu về trung tính ---
    for (int i = lift; i >= 0; i -= step) {
        // 0-3 hạ xuống
        setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
        setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
        setServoAngle(servoChannels[0][1], khuyuTrungTinh[0] - i);
        setServoAngle(servoChannels[3][1], khuyuTrungTinh[3] + i);
        setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0] + (i * knee / lift));
        setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3] - (i * knee / lift));
        // 1-2 thu về
        setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
        setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
        setServoAngle(servoChannels[1][1], khuyuTrungTinh[1]);
        setServoAngle(servoChannels[2][1], khuyuTrungTinh[2]);
        setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1] - (i * stride / lift));
        setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2] + (i * stride / lift));
        delay(delayPerStep);
    }
    // --- Pha 3: 1-2 swing về trước, 0-3 stance đạp về sau ---
    for (int i = 0; i <= lift; i += step) {
        // 1-2 swing
        setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
        setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
        setServoAngle(servoChannels[1][1], khuyuTrungTinh[1] + i);
        setServoAngle(servoChannels[2][1], khuyuTrungTinh[2] - i);
        setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1] - (i * knee / lift));
        setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2] + (i * knee / lift));
        // 0-3 stance
        setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
        setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
        setServoAngle(servoChannels[0][1], khuyuTrungTinh[0]);
        setServoAngle(servoChannels[3][1], khuyuTrungTinh[3]);
        setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0] + (i * stride / lift));
        setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3] - (i * stride / lift));
        delay(delayPerStep);
    }
    // --- Pha 4: 1-2 hạ xuống trung tính, 0-3 thu về trung tính ---
    for (int i = lift; i >= 0; i -= step) {
        // 1-2 hạ xuống
        setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
        setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
        setServoAngle(servoChannels[1][1], khuyuTrungTinh[1] + i);
        setServoAngle(servoChannels[2][1], khuyuTrungTinh[2] - i);
        setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1] - (i * knee / lift));
        setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2] + (i * knee / lift));
        // 0-3 thu về
        setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
        setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
        setServoAngle(servoChannels[0][1], khuyuTrungTinh[0]);
        setServoAngle(servoChannels[3][1], khuyuTrungTinh[3]);
        setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0] + (i * stride / lift));
        setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3] - (i * stride / lift));
        delay(delayPerStep);
    }
}


// Các hàm khác giữ nguyên
void Robot::walkBackward(int stepDelay) {
    // ... (giữ nguyên mã cũ)
}

void Robot::walkLeft(int stepDelay) {
    // ... (giữ nguyên mã cũ)
}

void Robot::walkRight(int stepDelay) {
    // ... (giữ nguyên mã cũ)
}

void Robot::balance(float pitch, float roll) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0; // Thời gian chênh lệch (giây)
    if (dt > 0.05 || dt <= 0) dt = 0.01;  // Giới hạn dt
    lastTime = now;

    // Tính toán sai số
    float pitchError = pitch; // Sai số pitch
    float rollError = roll;   // Sai số roll

    // Tính thành phần Integral
    pitchErrorSum += pitchError * dt;
    rollErrorSum += rollError * dt;
    pitchErrorSum = constrain(pitchErrorSum, -50, 50); // Giới hạn tích lũy
    rollErrorSum = constrain(rollErrorSum, -50, 50);

    // Tính thành phần Derivative
    float pitchErrorD = (pitchError - lastPitchError) / dt;
    float rollErrorD = (rollError - lastRollError) / dt;

    // Tính điều chỉnh PID
    float adjustPitch = Kp_pitch * pitchError + Ki_pitch * pitchErrorSum + Kd_pitch * pitchErrorD;
    float adjustRoll = Kp_roll * rollError + Ki_roll * rollErrorSum + Kd_roll * rollErrorD;

    // Cập nhật sai số trước đó
    lastPitchError = pitchError;
    lastRollError = rollError;

    // Áp dụng điều chỉnh cho từng chân
    for (int leg = 0; leg < 4; leg++) {
        float adjust = 0.0;

        // Điều chỉnh theo pitch
        if (leg == 0 || leg == 1) {  // Chân trước
            adjust -= adjustPitch;
        } else {  // Chân sau
            adjust += adjustPitch;
        }

        // Điều chỉnh theo roll
        if (leg == 1 || leg == 3) {  // Chân phải
            adjust -= adjustRoll;
        } else {  // Chân trái
            adjust += adjustRoll;
        }

        setServoAngle(servoChannels[leg][1], constrain(khuyuTrungTinh[leg] + adjust, 0, 180));
        setServoAngle(servoChannels[leg][2], constrain(dauGoiTrungTinh[leg] - adjust, 0, 180));
    }
}