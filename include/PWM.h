#pragma once
#include "config.h"

void pwmInit(void);

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void runPwm(float pos1, float pos2, float pos3, float pos4);
