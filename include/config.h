#pragma once
#include <Arduino.h>

// ===================== Vòng điều khiển (không dùng ngắt) =====================
// Mỗi tick (5 ms): 1 lần tính IK (khi đang dựng cache) hoặc 1 mẫu quỹ đạo,
// và tối đa 1 xung STEP.
#define LOOP_FREQ 200
#define AS5600_FEEDBACK_HZ 400

// ===================== Chân =====================
#define STEP 5
#define DIR 18
#define ENA 25

#define SCL 16
#define SDA 17
#define AS5600_ADDR 0x36
#define RAW_ANGLE_REG 0x0C

#define FIRST_ARM 23
#define SECOND_ARM 22
#define WRIST 21
#define GRIPPER 19

// ===================== Stepper (TMC2225, step/dir) =====================
#define MICROSTEP 4
#define RATIO 2 // 2 vòng motor / 1 vòng trục ra (AS5600 gắn ở trục ra)
#define MOTOR_STEP_ANGLE 0.9f
#define STEP_RATE_HZ 200        // tối đa 1 xung / tick  => STEP_RATE_HZ <= LOOP_FREQ
#define STEP_PULSE_US 4         // độ rộng xung STEP (TMC2225 cần >= 0.1 us)
#define STEP_DIR_SETUP_US 5     // chờ sau khi đổi chiều DIR
#define STEP_SPEED_MARGIN 0.8f  // dành 20% tốc độ xung cho hiệu chỉnh AS5600 khi đang chạy

static_assert(STEP_RATE_HZ <= LOOP_FREQ, "Chi phat toi da 1 xung moi tick");

constexpr float STEPS_PER_DEGREE =
    (360.0f / MOTOR_STEP_ANGLE) * MICROSTEP * RATIO / 360.0f;
constexpr float STEPPER_MAX_SPEED_DEG_PER_SEC =
    (float)STEP_RATE_HZ * STEP_SPEED_MARGIN / STEPS_PER_DEGREE;

// ===================== Giám sát bằng AS5600 =====================
#define STEPPER_SETTLE_DEADBAND_DEG 0.5f   // sai số chấp nhận khi dừng
#define STEPPER_MOVING_TRIM_ERROR_DEG 1.0f // chỉ bù sai số đáng kể khi đang chạy
#define STEPPER_MOVING_TRIM_INTERVAL_TICKS 10 // tối đa 40 xung bù/giây tại feedback 400 Hz
#define STEPPER_STALL_ERROR_DEG 4.0f       // sai số settling lớn hơn mức này mới báo lỗi
#define STEPPER_STALL_WINDOW_TICKS 40      // mỗi cửa sổ kiểm tra dài 100 ms tại feedback 400 Hz
#define STEPPER_STALL_MIN_PULSES 8         // bỏ qua cửa sổ có dịch chuyển lệnh quá nhỏ
#define STEPPER_STALL_MIN_PROGRESS_DEG 0.15f
#define STEPPER_STALL_NO_PROGRESS_WINDOWS 5 // cần 500 ms không tiến triển mới báo kẹt
#define STEPPER_TRIM_BUDGET_STEPS 60       // số xung hiệu chỉnh tối đa sau mỗi chuyển động
#define STEPPER_SETTLE_TIMEOUT_TICKS 400   // chờ ổn định tối đa 1 s tại feedback 400 Hz
#define STEPPER_LIMIT_MARGIN_DEG 2.0f
#define AS5600_LOST_TICKS 20               // mất đọc I2C 50 ms mới coi là mất cảm biến tại 400 Hz

// ===================== Giới hạn khớp / tốc độ =====================
#define SERVO_MAX_SPEED_DEG_PER_SEC 45.0f
#define MOTION_ACCEL_TIME_S 0.4f           // thời gian tăng tốc 0 -> tốc độ tối đa
#define PATH_SAMPLE_MM 2.0f                // khoảng cách giữa 2 điểm IK trên đường thẳng

#define SERVO1_MIN_DEG 0.0f
#define SERVO1_MAX_DEG 90.0f
#define SERVO2_MIN_DEG 90.0f
#define SERVO2_MAX_DEG 180.0f
#define STEPPER_MIN_DEG 60.0f
#define STEPPER_MAX_DEG 300.0f

#define WRIST_MIN_DEG 0.0f
#define WRIST_MAX_DEG 180.0f
#define GRIPPER_MIN_DEG 0.0f
#define GRIPPER_MAX_DEG 180.0f

// Thời gian chờ ổn định khi xoay cổ tay và đóng/mở kẹp tại điểm đích (ms)
#define WRIST_SETTLE_MS 500
#define GRIPPER_SETTLE_MS 500

// define length of arm, mm, deg, s
#define L1 130
#define L2 130
#define D 20
#define H 88
#define D1 0
#define D2 0

//deadzone of arm
#define DEADZONE_RADIUS 70
#define DEADZONE_HEIGHT 90
