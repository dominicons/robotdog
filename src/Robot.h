#pragma once

#include <Adafruit_PWMServoDriver.h>
#include "IMU.h"

#define SERVO_MOVE_DURATION 100

class Robot {
public:
    Robot();
    void begin();
    void standStill();
    void walkForward(int stepDelay = 200);
    void walkBackward(int stepDelay = 200);
    void walkLeft(int stepDelay = 200);
    void walkRight(int stepDelay = 200);
    void balance(float pitch, float roll);
    void setServoAngle(int channel, int angle);
    void smoothMove(int channel, int startAngle, int endAngle, int duration = SERVO_MOVE_DURATION);
    void smoothMove(int channel, int startAngle, int endAngle, int duration, int steps);
    void tripodGait(int stepDelay, int steps);
    void crawlGait(int stepDelay, int steps);
    // Bézier interpolation for smoothMove
    float bezier(float t, float P0, float P1, float P2, float P3);
private:
    Adafruit_PWMServoDriver pwm;
    static const int servoMin = 150;
    static const int servoMax = 600;
    int servoChannels[4][3];
    // Biến cho PID
    float pitchErrorSum, rollErrorSum;
    float lastPitchError, lastRollError;
    float Kp_pitch = 1.5, Ki_pitch = 0.1, Kd_pitch = 0.05; // Hệ số PID cho pitch
    float Kp_roll = 1.5, Ki_roll = 0.1, Kd_roll = 0.05;   // Hệ số PID cho roll
    unsigned long lastTime;
    IMU imu;
};