#pragma once

// Prototype các hàm điều khiển robot 4 chân
void robotStandStill();
void robotWalkForward();
void robotTripodWalk(int numSteps, double stepLength, int stepDelay);

// Chuyển đổi góc radian sang xung PWM cho PCA9685
int angleToPulse(double angleRad); // prototype cho hàm servo
