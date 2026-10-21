#include "PWM.h"

// LEDC 16-bit @ 50 Hz: 1 count = 0.3 us (trước đây 11-bit = 9.8 us ~ 1 độ/count)
#define PWM_BITS 16
#define PWM_PERIOD_US 20000.0f

void pwmInit() {
  ledcSetup(0, 50, PWM_BITS); ledcAttachPin(FIRST_ARM , 0);
  ledcSetup(1, 50, PWM_BITS); ledcAttachPin(SECOND_ARM, 1);
  ledcSetup(2, 50, PWM_BITS); ledcAttachPin(WRIST     , 2);
  ledcSetup(3, 50, PWM_BITS); ledcAttachPin(GRIPPER   , 3);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static uint32_t dutyFromPulseUs(float pulseUs) {
  if (pulseUs < 500.0f) pulseUs = 500.0f;
  if (pulseUs > 2500.0f) pulseUs = 2500.0f;
  return (uint32_t)lroundf(pulseUs * (float)(1UL << PWM_BITS) / PWM_PERIOD_US);
}

void runPwm(float pos1, float pos2, float pos3, float pos4) {
  const float ch1 = mapFloat(pos1, 0.0f, 90.0f, 2400.0f, 1400.0f);
  const float ch2 = mapFloat(pos2, 90.0f, 180.0f, 900.0f, 1930.0f);
  const float ch3 = mapFloat(pos3, 0.0f, 180.0f, 600.0f, 2400.0f);
  const float ch4 = mapFloat(pos4, 0.0f, 180.0f, 600.0f, 2400.0f);
  ledcWrite(0, dutyFromPulseUs(ch1));
  ledcWrite(1, dutyFromPulseUs(ch2));
  ledcWrite(2, dutyFromPulseUs(ch3));
  ledcWrite(3, dutyFromPulseUs(ch4));
}
