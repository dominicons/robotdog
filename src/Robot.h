#pragma once

#include <Adafruit_PWMServoDriver.h>
#include "IMU10DOF.h"
#include "Gait.h"

#define SERVO_MOVE_DURATION 100

class Robot {
public:
    Robot();
    void begin();
    void standStill();
    void walkForward(int stepDelay = 100);
    void walkForwardOverlap(int stepDelay = 100, int liftAngle = 25, int pushAngle = 10);
    void walkBackward(int stepDelay = 100);
    void walkLeft(int stepDelay = 100);
    void walkRight(int stepDelay = 100);
    void balance(float pitch, float roll);
    void setServoAngle(int channel, int angle);
    void smoothMove(int channel, int startAngle, int endAngle, int duration = SERVO_MOVE_DURATION);
    void smoothMove(int channel, int startAngle, int endAngle, int duration, int steps);
    void tripodGait(int stepDelay, int steps);
    void crawlGait(int stepDelay, int steps);
    void startOverlapGait(float cycle_ms = 800, float step_len = 30, float step_h = 20);
    void stopOverlapGait();
    void updateOverlapGait();
    void stepDiagonalPair(int stepDelay = 300, int lift = 25, int knee = 35);
    // Bézier interpolation for smoothMove
    float bezier(float t, float P0, float P1, float P2, float P3);
private:
    Adafruit_PWMServoDriver pwm;
    static const int servoMin = 150;
    static const int servoMax = 600;
    int servoChannels[4][3];
    Gait gait;
    // Biến cho PID
    float pitchErrorSum, rollErrorSum;
    float lastPitchError, lastRollError;
    float Kp_pitch = 1.5, Ki_pitch = 0.1, Kd_pitch = 0.05; // Hệ số PID cho pitch
    float Kp_roll = 1.5, Ki_roll = 0.1, Kd_roll = 0.05;   // Hệ số PID cho roll
    unsigned long lastTime;
    IMU10DOF imu;
};