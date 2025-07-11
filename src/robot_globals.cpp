
#include "leg.h"
#include "geometry.h"
#include "gait.h"
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

// Định nghĩa các biến toàn cục cho robot
leg legLF, legRF, legLH, legRH;
figure body;
gaitConfig walkGaitConfigLF, walkGaitConfigRF, walkGaitConfigLH, walkGaitConfigRH;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_CONTROLLER_ADDRESS);
