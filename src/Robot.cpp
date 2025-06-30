#include "Robot.h"
#include <Wire.h>
#include <Arduino.h>

// Hiệu chỉnh trung tính từng khớp cho từng chân (bạn hãy thay đổi các giá trị này cho đến khi robot đứng thẳng thật sự)
const int vaiTrungTinh[4]    = {90, 45, 90, 90}; // Chân trước phải (leg=1) chỉnh lại vai, ví dụ 110 độ
const int khuyuTrungTinh[4]  = {90, 90, 90, 90};
const int dauGoiTrungTinh[4] = {90, 90, 90, 90};

Robot::Robot() : pwm(Adafruit_PWMServoDriver()) {
    int tmp[4][3] = {{0,1,2},{3,4,5},{6,7,8},{9,10,11}};
    memcpy(servoChannels, tmp, sizeof(servoChannels));
}

void Robot::begin() {
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(50);
    standStill(); // Đặt robot về tư thế đứng yên an toàn khi khởi động
}

void Robot::setServoAngle(int channel, int angle) {
    angle = constrain(angle, 0, 180);
    int pulse = map(angle, 0, 180, servoMin, servoMax);
    pwm.setPWM(channel, 0, pulse);
}

void Robot::standStill() {
    // Đặt từng khớp về giá trị trung tính đã hiệu chỉnh
    for (int leg = 0; leg < 4; leg++) {
        setServoAngle(servoChannels[leg][0], vaiTrungTinh[leg]);
        setServoAngle(servoChannels[leg][1], khuyuTrungTinh[leg]);
        setServoAngle(servoChannels[leg][2], dauGoiTrungTinh[leg]);
    }
}

void Robot::walkForward(int stepDelay) {
    // Bước 1: Chân trái trước + phải sau nâng
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);  // Vai trái trước trung tính
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0] +20); // Khuỷu nâng
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0] -20);
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0] -20); // Khuỷu nâng
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0] +20);

 // Đầu gối gập
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);  // Vai phải sau trung tính
    setServoAngle(servoChannels[3][1], khuyuTrungTinh[3] + 10); // Khuỷu nâng
    setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3] - 10); // Đầu gối gập
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);  // Vai phải trước trung tính
    setServoAngle(servoChannels[1][1], khuyuTrungTinh[1]);  // Khuỷu giữ thẳng
    setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1]);  // Đầu gối giữ thẳng
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);  // Vai trái sau trung tính
    setServoAngle(servoChannels[2][1], khuyuTrungTinh[2]);  // Khuỷu giữ thẳng
    setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2]);  // Đầu gối giữ thẳng
    delay(stepDelay);

    // Bước 2: Chân trái trước + phải sau hạ
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0]);
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    setServoAngle(servoChannels[3][1], khuyuTrungTinh[3]);
    setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3]);
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    delay(stepDelay);

    // Bước 3: Chân phải trước + trái sau nâng
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    setServoAngle(servoChannels[1][1], khuyuTrungTinh[1] + 10);
    setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1] - 10);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    setServoAngle(servoChannels[2][1], khuyuTrungTinh[2] + 10);
    setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2] - 10);
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0]);
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    setServoAngle(servoChannels[3][1], khuyuTrungTinh[3]);
    setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3]);
    delay(stepDelay);

    // Bước 4: Chân phải trước + trái sau hạ
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    setServoAngle(servoChannels[1][1], khuyuTrungTinh[1]);
    setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1]);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    setServoAngle(servoChannels[2][1], khuyuTrungTinh[2]);
    setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2]);
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    delay(stepDelay);
}

void Robot::walkBackward(int stepDelay) {
    // Bước 1: Chân trái trước + phải sau nâng (gập lùi)
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0] - 10); // Khuỷu gập lùi
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0] + 10); // Đầu gối nâng lùi
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    setServoAngle(servoChannels[3][1], khuyuTrungTinh[3] - 10);
    setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3] + 10);
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    setServoAngle(servoChannels[1][1], khuyuTrungTinh[1]);
    setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1]);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    setServoAngle(servoChannels[2][1], khuyuTrungTinh[2]);
    setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2]);
    delay(stepDelay);

    // Bước 2: Chân trái trước + phải sau hạ
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0]);
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    setServoAngle(servoChannels[3][1], khuyuTrungTinh[3]);
    setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3]);
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    delay(stepDelay);

    // Bước 3: Chân phải trước + trái sau nâng (gập lùi)
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    setServoAngle(servoChannels[1][1], khuyuTrungTinh[1] - 10);
    setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1] + 10);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    setServoAngle(servoChannels[2][1], khuyuTrungTinh[2] - 10);
    setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2] + 10);
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0]);
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    setServoAngle(servoChannels[3][1], khuyuTrungTinh[3]);
    setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3]);
    delay(stepDelay);

    // Bước 4: Chân phải trước + trái sau hạ
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    setServoAngle(servoChannels[1][1], khuyuTrungTinh[1]);
    setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1]);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    setServoAngle(servoChannels[2][1], khuyuTrungTinh[2]);
    setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2]);
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    delay(stepDelay);
}

void Robot::walkLeft(int stepDelay) {
    // Bước 1: Chân trái trước + phải sau nâng, vai nghiêng trái
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0] + 20); // nghiêng mạnh ra ngoài
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0] + 10);
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0] - 10);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3] + 20);
    setServoAngle(servoChannels[3][1], khuyuTrungTinh[3] + 10);
    setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3] - 10);
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1] - 20);
    setServoAngle(servoChannels[1][1], khuyuTrungTinh[1]);
    setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1]);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2] - 20);
    setServoAngle(servoChannels[2][1], khuyuTrungTinh[2]);
    setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2]);
    delay(stepDelay);

    // Bước 2: Chân trái trước + phải sau hạ, vai về trung tính
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0]);
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    setServoAngle(servoChannels[3][1], khuyuTrungTinh[3]);
    setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3]);
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    delay(stepDelay);

    // Bước 3: Chân phải trước + trái sau nâng, vai nghiêng trái
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1] + 20);
    setServoAngle(servoChannels[1][1], khuyuTrungTinh[1] + 10);
    setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1] - 10);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2] + 20);
    setServoAngle(servoChannels[2][1], khuyuTrungTinh[2] + 10);
    setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2] - 10);
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0] - 20);
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0]);
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3] - 20);
    setServoAngle(servoChannels[3][1], khuyuTrungTinh[3]);
    setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3]);
    delay(stepDelay);

    // Bước 4: Chân phải trước + trái sau hạ, vai về trung tính
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    setServoAngle(servoChannels[1][1], khuyuTrungTinh[1]);
    setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1]);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    setServoAngle(servoChannels[2][1], khuyuTrungTinh[2]);
    setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2]);
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    delay(stepDelay);
}

void Robot::walkRight(int stepDelay) {
    // Bước 1: Chân trái trước + phải sau nâng, vai nghiêng phải
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0] - 20);
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0] + 10);
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0] - 10);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3] - 20);
    setServoAngle(servoChannels[3][1], khuyuTrungTinh[3] + 10);
    setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3] - 10);
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1] + 20);
    setServoAngle(servoChannels[1][1], khuyuTrungTinh[1]);
    setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1]);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2] + 20);
    setServoAngle(servoChannels[2][1], khuyuTrungTinh[2]);
    setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2]);
    delay(stepDelay);

    // Bước 2: Chân trái trước + phải sau hạ, vai về trung tính
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0]);
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    setServoAngle(servoChannels[3][1], khuyuTrungTinh[3]);
    setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3]);
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    delay(stepDelay);

    // Bước 3: Chân phải trước + trái sau nâng, vai nghiêng phải
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1] - 20);
    setServoAngle(servoChannels[1][1], khuyuTrungTinh[1] + 10);
    setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1] - 10);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2] - 20);
    setServoAngle(servoChannels[2][1], khuyuTrungTinh[2] + 10);
    setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2] - 10);
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0] + 20);
    setServoAngle(servoChannels[0][1], khuyuTrungTinh[0]);
    setServoAngle(servoChannels[0][2], dauGoiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3] + 20);
    setServoAngle(servoChannels[3][1], khuyuTrungTinh[3]);
    setServoAngle(servoChannels[3][2], dauGoiTrungTinh[3]);
    delay(stepDelay);

    // Bước 4: Chân phải trước + trái sau hạ, vai về trung tính
    setServoAngle(servoChannels[1][0], vaiTrungTinh[1]);
    setServoAngle(servoChannels[1][1], khuyuTrungTinh[1]);
    setServoAngle(servoChannels[1][2], dauGoiTrungTinh[1]);
    setServoAngle(servoChannels[2][0], vaiTrungTinh[2]);
    setServoAngle(servoChannels[2][1], khuyuTrungTinh[2]);
    setServoAngle(servoChannels[2][2], dauGoiTrungTinh[2]);
    setServoAngle(servoChannels[0][0], vaiTrungTinh[0]);
    setServoAngle(servoChannels[3][0], vaiTrungTinh[3]);
    delay(stepDelay);
}

void Robot::balance(float pitch, float roll) {
    // TODO: implement balancing logic
}
