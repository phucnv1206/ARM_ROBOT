#pragma once
#include <Arduino.h>
#include "config.h"

void pwmInit(void);

inline uint16_t duty(uint16_t pulse);

void runPwm(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4);