#pragma once
#include "config.h"

bool stepInit(void);
bool setStepperInitialAngle(float angleDeg);
void enableStep(bool en);
int16_t runStep(bool en, bool direct);
void releaseStep(void);
bool moveToAngle(float angleDeg);
bool moveToAngleFeedback(float angleDeg);
bool updateStepperFeedback(float measuredAngleDeg);
void setStepperRate(float rateHz);
void stopStepper(void);
bool isStepperMoving(void);
float getStepperAngle(void);