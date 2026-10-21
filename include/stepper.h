#pragma once
#include "config.h"

bool stepInit(void);
bool setStepperInitialAngle(float angleDeg);
void enableStep(bool en);
void releaseStep(void);

// Gọi mỗi tick: phát TỐI ĐA 1 xung STEP để đưa vị trí đếm xung tiến gần góc đích.
// Trả về -1/0/+1 (số xung đã phát, có dấu).
int8_t stepperTrack(float targetAngleDeg);

// Bù sai lệch (mất bước/backlash) dựa trên AS5600: +1/-1 xung, giữ lại vĩnh viễn.
void stepperAdjustTrim(int8_t deltaSteps);

float getStepperAngle(void);
int32_t getStepperSteps(void);
uint32_t stepperOverrunCount(void);  // số lần yêu cầu > 1 xung/tick (phải luôn = 0)
