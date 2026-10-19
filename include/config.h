#pragma once
#include <Arduino.h>

#define LOOP_FREQ 50

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

#define MICROSTEP 4
#define RATIO 2 // hiệu chỉnh theo tỉ số truyền thực tế: 2 vòng motor / 1 vòng trục ra
#define MOTOR_STEP_ANGLE 0.9f
#define STEP_RATE_HZ 400
#define STEPPER_FEEDBACK_DEADBAND_DEG 0.5f
#define JOINT_ANGLE_RESOLUTION_DEG 0.9f
#define SERVO_MAX_SPEED_DEG_PER_SEC 45.0f
#define SERVO1_MIN_DEG 0.0f
#define SERVO1_MAX_DEG 90.0f
#define SERVO2_MIN_DEG 90.0f
#define SERVO2_MAX_DEG 180.0f
#define STEPPER_MIN_DEG 60.0f
#define STEPPER_MAX_DEG 300.0f

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