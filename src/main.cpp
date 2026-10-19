#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "PWM.h"
#include "stepper.h"
#include "as5600.h"
#include "robot_comms.h"

namespace {
constexpr uint32_t controlPeriodUs = 1000000UL / LOOP_FREQ;
RobotControlState robotState = {
  60.0f,
  120.0f,
  180.0f,
  NAN,
  false,
  false,
  false,
  false
};
uint32_t lastControlUs = 0;

void updateControl() {
  if (!robotState.stepperReady) {
    return;
  }

  robotCommsUpdateControl(robotState, AngleDeg());
}
}

void setup() {
  robotCommsBegin();
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  asInit();
  pwmInit();
  runPwm(robotState.servo1Angle, robotState.servo2Angle, 0.0f, 0.0f);

  robotState.stepperReady = stepInit();
  if (!robotState.stepperReady) {
    robotCommsSendError("stepper timer initialization failed");
    enableStep(false);
  } else {
    enableStep(true);
    delay(500);
    robotState.measuredStepAngle = AngleDeg();
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
  }

  releaseStep();
  lastControlUs = micros();
}

void loop() {
  robotCommsPoll(robotState);

  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlUs) >= controlPeriodUs) {
    lastControlUs = nowUs;
    updateControl();
  }
}
