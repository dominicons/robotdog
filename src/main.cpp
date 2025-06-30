#include <Arduino.h>
#include "Robot.h"
#include "WebServer.h"
#include "IMU.h"

Robot robot;
WebServerControl web;
IMU imu;

void setup() {
    Serial.begin(115200);
    robot.begin();
    web.begin();
    imu.begin();
}

void loop() {
    web.handleClient();
    imu.update();

    String cmd = web.getCommand();
    if (cmd == "forward") robot.walkForward();
    else if (cmd == "backward") robot.walkBackward();
    else if (cmd == "left") robot.walkLeft();
    else if (cmd == "right") robot.walkRight();
    else if (cmd == "stop") robot.standStill();

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