#include "Robot.h"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

// Hiệu chỉnh trung tính từng khớp cho từng chân
const int vaiTrungTinh[4]    = {90, 45, 90, 90};
const int khuyuTrungTinh[4]  = {90, 90, 90, 90};
const int dauGoiTrungTinh[4] = {90, 90, 90, 90};

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
    angle = constrain(angle, 0, 180);
    int pulse = map(angle, 0, 180, servoMin, servoMax);
    pwm.setPWM(channel, 0, pulse);
}

void Robot::smoothMove(int channel, int startAngle, int endAngle, int duration) {
    int steps = 50; // Tăng số bước để mượt mà hơn
    float stepDuration = duration / (float)steps;
    for (int i = 0; i <= steps; i++) {
        float t = i / (float)steps;
        float angle = startAngle + (endAngle - startAngle) * (0.5 - 0.5 * cos(t * PI)); // Sử dụng cos để chuyển động mượt
        setServoAngle(channel, (int)angle);
        delay(stepDuration);
    }
}

void Robot::standStill() {
    for (int leg = 0; leg < 4; leg++) {
        setServoAngle(servoChannels[leg][0], vaiTrungTinh[leg]);
        setServoAngle(servoChannels[leg][1], khuyuTrungTinh[leg]);
        setServoAngle(servoChannels[leg][2], dauGoiTrungTinh[leg]);
    }
}

void Robot::walkForward(int stepDelay) {
    // Bước 1: Chân trái trước (0) nâng lên
    smoothMove(servoChannels[0][0], vaiTrungTinh[0], vaiTrungTinh[0]);
    smoothMove(servoChannels[0][1], khuyuTrungTinh[0], khuyuTrungTinh[0] + 20); // Góc rộng hơn
    smoothMove(servoChannels[0][2], dauGoiTrungTinh[0], dauGoiTrungTinh[0] - 35); // Góc rộng hơn
    delay(stepDelay / 3);
    // Khi chân trái trước sắp đặt xuống, chân phải sau (3) bắt đầu nâng lên
    smoothMove(servoChannels[0][1], khuyuTrungTinh[0] + 20, khuyuTrungTinh[0]);
    smoothMove(servoChannels[0][2], dauGoiTrungTinh[0] - 35, dauGoiTrungTinh[0]);
    smoothMove(servoChannels[3][0], vaiTrungTinh[3], vaiTrungTinh[3]);
    smoothMove(servoChannels[3][1], khuyuTrungTinh[3], khuyuTrungTinh[3] + 20);
    smoothMove(servoChannels[3][2], dauGoiTrungTinh[3], dauGoiTrungTinh[3] - 35);
    delay(stepDelay / 3);
    // Khi chân phải sau sắp đặt xuống, chân phải trước (1) bắt đầu nâng lên
    smoothMove(servoChannels[3][1], khuyuTrungTinh[3] + 20, khuyuTrungTinh[3]);
    smoothMove(servoChannels[3][2], dauGoiTrungTinh[3] - 35, dauGoiTrungTinh[3]);
    smoothMove(servoChannels[1][0], vaiTrungTinh[1], vaiTrungTinh[1]);
    smoothMove(servoChannels[1][1], khuyuTrungTinh[1], khuyuTrungTinh[1] + 20);
    smoothMove(servoChannels[1][2], dauGoiTrungTinh[1], dauGoiTrungTinh[1] - 35);
    delay(stepDelay / 3);
    // Khi chân phải trước sắp đặt xuống, chân trái sau (2) bắt đầu nâng lên
    smoothMove(servoChannels[1][1], khuyuTrungTinh[1] + 20, khuyuTrungTinh[1]);
    smoothMove(servoChannels[1][2], dauGoiTrungTinh[1] - 35, dauGoiTrungTinh[1]);
    smoothMove(servoChannels[2][0], vaiTrungTinh[2], vaiTrungTinh[2]);
    smoothMove(servoChannels[2][1], khuyuTrungTinh[2], khuyuTrungTinh[2] + 20);
    smoothMove(servoChannels[2][2], dauGoiTrungTinh[2], dauGoiTrungTinh[2] - 35);
    delay(stepDelay / 3);
    // Hạ chân trái sau về trung tính
    smoothMove(servoChannels[2][1], khuyuTrungTinh[2] + 20, khuyuTrungTinh[2]);
    smoothMove(servoChannels[2][2], dauGoiTrungTinh[2] - 35, dauGoiTrungTinh[2]);
    delay(stepDelay / 3);
}

void Robot::walkBackward(int stepDelay) {
    // Bước 1: Chân trái trước + phải sau nâng
    smoothMove(servoChannels[0][1], khuyuTrungTinh[0], khuyuTrungTinh[0] - 15, 200); // Góc rộng hơn
    smoothMove(servoChannels[0][2], dauGoiTrungTinh[0], dauGoiTrungTinh[0] + 20, 200);
    smoothMove(servoChannels[3][1], khuyuTrungTinh[3], khuyuTrungTinh[3] - 15, 200);
    smoothMove(servoChannels[3][2], dauGoiTrungTinh[3], dauGoiTrungTinh[3] + 20, 200);
    delay(stepDelay);

    // Bước 2: Chân trái trước + phải sau hạ
    smoothMove(servoChannels[0][1], khuyuTrungTinh[0] - 15, khuyuTrungTinh[0], 200);
    smoothMove(servoChannels[0][2], dauGoiTrungTinh[0] + 20, dauGoiTrungTinh[0], 200);
    smoothMove(servoChannels[3][1], khuyuTrungTinh[3] - 15, khuyuTrungTinh[3], 200);
    smoothMove(servoChannels[3][2], dauGoiTrungTinh[3] + 20, dauGoiTrungTinh[3], 200);
    delay(stepDelay);

    // Bước 3: Chân phải trước + trái sau nâng
    smoothMove(servoChannels[1][1], khuyuTrungTinh[1], khuyuTrungTinh[1] - 15, 200);
    smoothMove(servoChannels[1][2], dauGoiTrungTinh[1], dauGoiTrungTinh[1] + 20, 200);
    smoothMove(servoChannels[2][1], khuyuTrungTinh[2], khuyuTrungTinh[2] - 15, 200);
    smoothMove(servoChannels[2][2], dauGoiTrungTinh[2], dauGoiTrungTinh[2] + 20, 200);
    delay(stepDelay);

    // Bước 4: Chân phải trước + trái sau hạ
    smoothMove(servoChannels[1][1], khuyuTrungTinh[1] - 15, khuyuTrungTinh[1], 200);
    smoothMove(servoChannels[1][2], dauGoiTrungTinh[1] + 20, dauGoiTrungTinh[1], 200);
    smoothMove(servoChannels[2][1], khuyuTrungTinh[2] - 15, khuyuTrungTinh[2], 200);
    smoothMove(servoChannels[2][2], dauGoiTrungTinh[2] + 20, dauGoiTrungTinh[2], 200);
    delay(stepDelay);
}

void Robot::walkLeft(int stepDelay) {
    // Bước 1: Chân trái trước + phải sau nâng
    smoothMove(servoChannels[0][0], vaiTrungTinh[0], vaiTrungTinh[0] + 30, 200); // Góc rộng hơn
    smoothMove(servoChannels[0][1], khuyuTrungTinh[0], khuyuTrungTinh[0] + 15, 200);
    smoothMove(servoChannels[0][2], dauGoiTrungTinh[0], dauGoiTrungTinh[0] - 20, 200);
    smoothMove(servoChannels[3][0], vaiTrungTinh[3], vaiTrungTinh[3] + 30, 200);
    smoothMove(servoChannels[3][1], khuyuTrungTinh[3], khuyuTrungTinh[3] + 15, 200);
    smoothMove(servoChannels[3][2], dauGoiTrungTinh[3], dauGoiTrungTinh[3] - 20, 200);
    delay(stepDelay);

    // Bước 2: Chân trái trước + phải sau hạ
    smoothMove(servoChannels[0][0], vaiTrungTinh[0] + 30, vaiTrungTinh[0], 200);
    smoothMove(servoChannels[0][1], khuyuTrungTinh[0] + 15, khuyuTrungTinh[0], 200);
    smoothMove(servoChannels[0][2], dauGoiTrungTinh[0] - 20, dauGoiTrungTinh[0], 200);
    smoothMove(servoChannels[3][0], vaiTrungTinh[3] + 30, vaiTrungTinh[3], 200);
    smoothMove(servoChannels[3][1], khuyuTrungTinh[3] + 15, khuyuTrungTinh[3], 200);
    smoothMove(servoChannels[3][2], dauGoiTrungTinh[3] - 20, dauGoiTrungTinh[3], 200);
    delay(stepDelay);

    // Bước 3: Chân phải trước + trái sau nâng
    smoothMove(servoChannels[1][0], vaiTrungTinh[1], vaiTrungTinh[1] + 30, 200);
    smoothMove(servoChannels[1][1], khuyuTrungTinh[1], khuyuTrungTinh[1] + 15, 200);
    smoothMove(servoChannels[1][2], dauGoiTrungTinh[1], dauGoiTrungTinh[1] - 20, 200);
    smoothMove(servoChannels[2][0], vaiTrungTinh[2], vaiTrungTinh[2] + 30, 200);
    smoothMove(servoChannels[2][1], khuyuTrungTinh[2], khuyuTrungTinh[2] + 15, 200);
    smoothMove(servoChannels[2][2], dauGoiTrungTinh[2], dauGoiTrungTinh[2] - 20, 200);
    delay(stepDelay);

    // Bước 4: Chân phải trước + trái sau hạ
    smoothMove(servoChannels[1][0], vaiTrungTinh[1] + 30, vaiTrungTinh[1], 200);
    smoothMove(servoChannels[1][1], khuyuTrungTinh[1] + 15, khuyuTrungTinh[1], 200);
    smoothMove(servoChannels[1][2], dauGoiTrungTinh[1] - 20, dauGoiTrungTinh[1], 200);
    smoothMove(servoChannels[2][0], vaiTrungTinh[2] + 30, vaiTrungTinh[2], 200);
    smoothMove(servoChannels[2][1], khuyuTrungTinh[2] + 15, khuyuTrungTinh[2], 200);
    smoothMove(servoChannels[2][2], dauGoiTrungTinh[2] - 20, dauGoiTrungTinh[2], 200);
    delay(stepDelay);
}

void Robot::walkRight(int stepDelay) {
    // Bước 1: Chân trái trước + phải sau nâng
    smoothMove(servoChannels[0][0], vaiTrungTinh[0], vaiTrungTinh[0] - 30, 200); // Góc rộng hơn
    smoothMove(servoChannels[0][1], khuyuTrungTinh[0], khuyuTrungTinh[0] + 15, 200);
    smoothMove(servoChannels[0][2], dauGoiTrungTinh[0], dauGoiTrungTinh[0] - 20, 200);
    smoothMove(servoChannels[3][0], vaiTrungTinh[3], vaiTrungTinh[3] - 30, 200);
    smoothMove(servoChannels[3][1], khuyuTrungTinh[3], khuyuTrungTinh[3] + 15, 200);
    smoothMove(servoChannels[3][2], dauGoiTrungTinh[3], dauGoiTrungTinh[3] - 20, 200);
    delay(stepDelay);

    // Bước 2: Chân trái trước + phải sau hạ
    smoothMove(servoChannels[0][0], vaiTrungTinh[0] - 30, vaiTrungTinh[0], 200);
    smoothMove(servoChannels[0][1], khuyuTrungTinh[0] + 15, khuyuTrungTinh[0], 200);
    smoothMove(servoChannels[0][2], dauGoiTrungTinh[0] - 20, dauGoiTrungTinh[0], 200);
    smoothMove(servoChannels[3][0], vaiTrungTinh[3] - 30, vaiTrungTinh[3], 200);
    smoothMove(servoChannels[3][1], khuyuTrungTinh[3] + 15, khuyuTrungTinh[3], 200);
    smoothMove(servoChannels[3][2], dauGoiTrungTinh[3] - 20, dauGoiTrungTinh[3], 200);
    delay(stepDelay);

    // Bước 3: Chân phải trước + trái sau nâng
    smoothMove(servoChannels[1][0], vaiTrungTinh[1], vaiTrungTinh[1] - 30, 200);
    smoothMove(servoChannels[1][1], khuyuTrungTinh[1], khuyuTrungTinh[1] + 15, 200);
    smoothMove(servoChannels[1][2], dauGoiTrungTinh[1], dauGoiTrungTinh[1] - 20, 200);
    smoothMove(servoChannels[2][0], vaiTrungTinh[2], vaiTrungTinh[2] - 30, 200);
    smoothMove(servoChannels[2][1], khuyuTrungTinh[2], khuyuTrungTinh[2] + 15, 200);
    smoothMove(servoChannels[2][2], dauGoiTrungTinh[2], dauGoiTrungTinh[2] - 20, 200);
    delay(stepDelay);

    // Bước 4: Chân phải trước + trái sau hạ
    smoothMove(servoChannels[1][0], vaiTrungTinh[1] - 30, vaiTrungTinh[1], 200);
    smoothMove(servoChannels[1][1], khuyuTrungTinh[1] + 15, khuyuTrungTinh[1], 200);
    smoothMove(servoChannels[1][2], dauGoiTrungTinh[1] - 20, dauGoiTrungTinh[1], 200);
    smoothMove(servoChannels[2][0], vaiTrungTinh[2] - 30, vaiTrungTinh[2], 200);
    smoothMove(servoChannels[2][1], khuyuTrungTinh[2] + 15, khuyuTrungTinh[2], 200);
    smoothMove(servoChannels[2][2], dauGoiTrungTinh[2] - 20, dauGoiTrungTinh[2], 200);
    delay(stepDelay);
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