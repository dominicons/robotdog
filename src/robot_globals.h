#pragma once
#include "leg.h"
#include "geometry.h"
#include "gait.h"
#include <Adafruit_PWMServoDriver.h>

extern leg legLF, legRF, legLH, legRH;
extern figure body;
extern gaitConfig walkGaitConfigLF, walkGaitConfigRF, walkGaitConfigLH, walkGaitConfigRH;
extern Adafruit_PWMServoDriver pwm;
