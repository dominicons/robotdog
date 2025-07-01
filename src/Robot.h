#pragma once
#include <Adafruit_PWMServoDriver.h>

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
    void setServoAngle(int channel, int angle);
    void smoothMove(int channel, int startAngle, int endAngle, int duration = SERVO_MOVE_DURATION);
};