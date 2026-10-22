#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "PWM.h"
#include "stepper.h"
#include "as5600.h"
#include "robot_comms.h"

namespace {
constexpr uint32_t controlPeriodUs = 1000000UL / LOOP_FREQ;
constexpr uint32_t feedbackPeriodUs = 1000000UL / AS5600_FEEDBACK_HZ;
static_assert(AS5600_FEEDBACK_HZ >= LOOP_FREQ,
              "AS5600 feedback must not be slower than the control loop");
RobotControlState robotState = {
  60.0f,   // servo1
  120.0f,  // servo2
  180.0f,  // step target
  NAN,     // measured
  90.0f,   // wrist
  0.0f,    // gripper
  false, false, false, false
};
uint32_t nextTickUs = 0;
uint32_t nextFeedbackUs = 0;

// Trung bình vài lần đọc lúc khởi động để giảm nhiễu.
float readInitialAngle() {
  float sum = 0.0f;
  int good = 0;
  for (int i = 0; i < 8; ++i) {
    const float a = AngleDeg();
    if (isfinite(a) && a >= 0.0f && a < 360.0f) { sum += a; ++good; }
    delay(5);
  }
  return good >= 4 ? sum / good : NAN;
}
}

void setup() {
  robotCommsBegin();
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  asInit();
  pwmInit();
  runPwm(robotState.servo1Angle, robotState.servo2Angle, robotState.wristAngle, robotState.gripperAngle);

  robotState.stepperReady = stepInit();
  enableStep(true);
  delay(500);
  robotState.measuredStepAngle = readInitialAngle();
  if (!isfinite(robotState.measuredStepAngle) ||
      robotState.measuredStepAngle < STEPPER_MIN_DEG ||
      robotState.measuredStepAngle > STEPPER_MAX_DEG ||
      !setStepperInitialAngle(robotState.measuredStepAngle)) {
    robotCommsSendError("AS5600 initial angle invalid; stepper disabled");
    robotState.stepperReady = false;
    enableStep(false);
  } else {
    robotState.stepTargetAngle = robotState.measuredStepAngle;
  }

  releaseStep();
  const uint32_t nowUs = micros();
  nextTickUs = nowUs + controlPeriodUs;
  nextFeedbackUs = nowUs + feedbackPeriodUs;
}

void loop() {
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextTickUs) >= 0) {
    nextTickUs += controlPeriodUs;
    if ((int32_t)(nowUs - nextTickUs) >= 0) {
      nextTickUs = nowUs + controlPeriodUs;  // trễ hơn 1 chu kỳ: đồng bộ lại, không bù dồn
    }
    // 1) việc cần đúng nhịp: IK/mẫu quỹ đạo, PWM, xung STEP
    robotCommsRunTick(robotState);
  }

  nowUs = micros();
  if ((int32_t)(nowUs - nextFeedbackUs) >= 0) {
    nextFeedbackUs += feedbackPeriodUs;
    if ((int32_t)(nowUs - nextFeedbackUs) >= 0) {
      nextFeedbackUs = nowUs + feedbackPeriodUs;  // bỏ mẫu trễ, không đọc bù dồn
    }
    if (robotState.stepperReady) {
      robotCommsFeedback(robotState, AngleDeg());
    }
  }

  robotCommsPoll(robotState);
}
