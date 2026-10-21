#include "stepper.h"
#include <math.h>
#include <limits.h>

namespace {
int32_t positionSteps = 0;   // số xung đã phát (tuyệt đối)
int32_t trimSteps = 0;       // bù do AS5600
bool initialized = false;
bool dirLevel = false;
uint32_t overruns = 0;
}

bool stepInit(void) {
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(STEP, LOW);
  digitalWrite(DIR, LOW);
  dirLevel = false;
  initialized = false;
  return true;
}

bool setStepperInitialAngle(float angleDeg) {
  if (!isfinite(angleDeg)) return false;
  const double steps = (double)angleDeg * STEPS_PER_DEGREE;
  if (steps < (double)INT32_MIN || steps > (double)INT32_MAX) return false;
  positionSteps = (int32_t)lround(steps);
  trimSteps = 0;
  initialized = true;
  return true;
}

void enableStep(bool en) {
  digitalWrite(ENA, en ? LOW : HIGH);
}

void releaseStep(void) {
  digitalWrite(STEP, LOW);
}

int8_t stepperTrack(float targetAngleDeg) {
  if (!initialized || !isfinite(targetAngleDeg)) return 0;

  const int32_t desired = (int32_t)lroundf(targetAngleDeg * STEPS_PER_DEGREE) + trimSteps;
  const int32_t delta = desired - positionSteps;
  if (delta == 0) return 0;
  if (delta > 1 || delta < -1) ++overruns;  // quỹ đạo đòi nhanh hơn 1 xung/tick

  const bool up = delta > 0;
  if (up != dirLevel) {
    digitalWrite(DIR, up ? HIGH : LOW);
    dirLevel = up;
    delayMicroseconds(STEP_DIR_SETUP_US);
  }
  digitalWrite(STEP, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(STEP, LOW);
  positionSteps += up ? 1 : -1;
  return up ? 1 : -1;
}

void stepperAdjustTrim(int8_t deltaSteps) {
  trimSteps += deltaSteps;
}

float getStepperAngle(void) {
  return initialized ? (float)(positionSteps - trimSteps) / STEPS_PER_DEGREE : NAN;
}

int32_t getStepperSteps(void) { return positionSteps; }
uint32_t stepperOverrunCount(void) { return overruns; }
