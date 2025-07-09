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