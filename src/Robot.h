#pragma once
#include <Adafruit_PWMServoDriver.h>

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
    void setServoAngle(int channel, int angle);
};
