#pragma once

struct RobotControlState {
  float servo1Angle;
  float servo2Angle;
  float stepTargetAngle;
  float measuredStepAngle;
  bool stepperReady;
  bool feedbackActive;
  bool trajectoryActive;
  bool trajectoryPreparing;
};

void robotCommsBegin();
void robotCommsPoll(RobotControlState &state);
void robotCommsUpdateControl(RobotControlState &state, float measuredAngleDeg);
void robotCommsSendError(const char *message);
