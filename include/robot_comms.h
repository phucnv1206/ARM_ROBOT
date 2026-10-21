#pragma once

struct RobotControlState {
  float servo1Angle;        // góc lệnh servo 1
  float servo2Angle;        // góc lệnh servo 2
  float stepTargetAngle;    // góc lệnh stepper (liên tục, chưa làm tròn xung)
  float measuredStepAngle;  // góc đo bằng AS5600 (trục ra)
  bool stepperReady;
  bool feedbackActive;      // đang có chuyển động được giám sát
  bool trajectoryActive;    // đang seek / chạy / chờ ổn định của quỹ đạo A-B
  bool trajectoryPreparing; // đang tính IK dựng cache
};

void robotCommsBegin();
void robotCommsPoll(RobotControlState &state);

// Gọi đúng 1 lần mỗi tick 5 ms, TRƯỚC khi đọc AS5600:
//  - dựng cache IK (1 điểm/tick) hoặc lấy 1 mẫu quỹ đạo,
//  - cập nhật PWM servo, phát tối đa 1 xung STEP.
void robotCommsRunTick(RobotControlState &state);

// Gọi theo nhịp AS5600_FEEDBACK_HZ với góc AS5600: giám sát và hiệu chỉnh step.
void robotCommsFeedback(RobotControlState &state, float measuredAngleDeg);

void robotCommsSendError(const char *message);
