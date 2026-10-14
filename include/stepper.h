#pragma once
#include <Arduino.h>
#include "config.h"

void stepInit(void);
void enableStep(bool en);
int16_t runStep(bool en, bool direct);
void releaseStep(void);