#pragma once
#include "config.h"
#include <math.h>

void ForwardKinematics(float theta1, float theta2, float theta3, float *x, float *y, float *z);
bool InvertKinematics(float x, float y, float z, float *theta1, float *theta2, float *theta3);
bool checkCondition(float angle1, float angle2);