#include <Arduino.h>
#include "Robot.h"
#include "WebServer.h"
#include "IMU10DOF.h"

Robot robot;
WebServerControl web;
IMU10DOF imu;

void setup() {
    Serial.begin(115200);
    robot.begin();
    web.begin();
    imu.begin();
}

void loop() {
    web.handleClient();
    //imu.update();

    String cmd = web.getCommand();
    if (cmd == "forward") robot.walkForwardOverlap();
    else if (cmd == "overlap") robot.stepDiagonalPair(250, 25, 35);
    else if (cmd == "stop") robot.stopOverlapGait();
    else if (cmd == "backward") robot.walkBackward();
    else if (cmd == "left") robot.walkLeft();
    else if (cmd == "right") robot.walkRight();

    // Cập nhật continuous overlap gait nếu đang chạy
    robot.updateOverlapGait();

    // Xuất giá trị IMU ra Serial để debug
    Serial.print("Pitch: ");
    Serial.print(imu.getPitch());
    Serial.print(" | Roll: ");
    Serial.println(imu.getRoll());

    // Tự cân bằng nếu bật
    if (web.isBalanceEnabled()) {
        robot.balance(imu.getPitch(), imu.getRoll());
    }

    delay(20);
}