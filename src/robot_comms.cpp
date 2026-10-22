#include "robot_comms.h"
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include "PWM.h"
#include "config.h"
#include "kinematics.h"
#include "motion.h"
#include "stepper.h"

namespace {
constexpr uint32_t telemetryPeriodMs = 100;
constexpr uint32_t controlPeriodMs = 1000UL / LOOP_FREQ;
constexpr float loopDtS = 1.0f / (float)LOOP_FREQ;
constexpr float jointLimitEpsilonDeg = 0.15f;  // cho phép sai số làm tròn của IK ở biên
constexpr uint8_t packetMagicFirst = 0x52;
constexpr uint8_t packetMagicSecond = 0x42;
constexpr uint8_t packetVersion = 1;
constexpr uint8_t packetState = 1;
constexpr uint8_t packetAck = 2;
constexpr uint8_t packetError = 3;
constexpr uint8_t packetSequencePoint = 4;
constexpr uint8_t commandGetState = 16;
constexpr uint8_t commandSetJoints = 17;
constexpr uint8_t commandPlayTrajectory = 18;
constexpr uint8_t commandStop = 19;
constexpr uint8_t commandSavePoint = 20;
constexpr uint8_t commandClearPoints = 21;
constexpr uint8_t commandPlaySequence = 22;
constexpr uint8_t commandGetPoints = 23;
constexpr uint8_t ackDetailPreparing = 1;

constexpr uint8_t MAX_SEQUENCE_POINTS = 8;

enum CommunicationError : uint8_t {
  errorNone = 0,
  errorTrajectoryBusy = 1,
  errorStepperNotReady = 2,
  errorPointInvalid = 3,
  errorJointPathInvalid = 4,
  errorCommandInvalid = 5,
  errorStopBeforeSet = 6,
  errorAngleLimit = 7,
  errorServoCondition = 8,
  errorStepperFeedbackNotReady = 9,
  errorCommandTooLong = 10,
  errorTrajectoryPointInvalid = 11,
  errorTrajectoryJointLimit = 12,
  errorAs5600Lost = 13,
  errorJointTargetInvalid = 14,
  errorPointNotFinite = 15,
  errorUnknownCommand = 16,
  errorJointCommandNotFinite = 17,
  errorStepperStall = 18,
  errorTrajectoryCacheFull = 19,
  errorSequenceFull = 20,
  errorSequenceEmpty = 21,
  errorSequenceBusy = 22
};

#pragma pack(push, 1)
struct CommunicationPacket {
  uint8_t magic[2];
  uint8_t version;
  uint8_t type;
  uint8_t payload[24];
  uint16_t crc;
};
#pragma pack(pop)

static_assert(sizeof(CommunicationPacket) == 30,
              "CommunicationPacket layout must remain 30 bytes");
static_assert(sizeof(float) == 4, "Protocol requires 32-bit float");

struct CartesianPoint {
  float x;
  float y;
  float z;
};

struct StoredPoint {
  float x;
  float y;
  float z;
  float wrist;
  float gripper;
};

// ---- Chuỗi điểm trong Flash ----
Preferences preferences;
StoredPoint sequencePoints[MAX_SEQUENCE_POINTS] = {};
uint8_t sequenceCount = 0;
bool sequenceActive = false;
int8_t sequenceDirection = 1;
uint8_t sequenceSegmentIndex = 0;

// ---- Mảng lưu bước chạy cuối cùng (Baked Segments) ----
constexpr uint8_t MAX_WAYPOINTS = 8;
constexpr uint8_t MAX_SEGMENTS = MAX_WAYPOINTS - 1; // 7 đoạn
constexpr uint16_t MAX_STEPS_PER_SEGMENT = 800;

struct SegmentCache {
  Joint steps[MAX_STEPS_PER_SEGMENT];
  uint16_t count = 0;
};

SegmentCache bakedSegments[MAX_SEGMENTS];
uint8_t bakedSegmentCount = 0;
bool isSequenceBaked = false;

// Trạng thái phát lại chuỗi
bool activeSeqPlayback = false;
int8_t activeSegIndex = 0;
int16_t activeStepIndex = 0;

enum class PlaySubState : uint8_t {
  MovingArm,
  SettleWrist,
  SettleGripper
};

PlaySubState playSubState = PlaySubState::MovingArm;
uint16_t settleTimerTicks = 0;
float wristLerpFrom = 90.0f;
float wristLerpTo = 90.0f;
uint16_t wristTotalTicks = 1;

// Trạng thái tính toán bake chuỗi
uint8_t bakeSegIndex = 0;
uint16_t bakePointIndex = 0;
uint16_t bakePointCount = 0;
CartesianPoint bakeFrom = {0, 0, 0};
CartesianPoint bakeTo = {0, 0, 0};

enum class Mode : uint8_t {
  Idle,
  Building,
  BakingSequence,
  Seeking,
  Playing,
  PlayingSequence,
  Settling
};

uint8_t receiveBuffer[sizeof(CommunicationPacket) * 4];
size_t receiveLength = 0;
uint32_t lastTelemetryMs = 0;

// ---- Quỹ đạo A-B ----
Trajectory path;  // cache các điểm IK (float, KHÔNG lượng tử hóa) + thời gian
CartesianPoint cachedPointA = {0, 0, 0};
CartesianPoint cachedPointB = {0, 0, 0};
CartesianPoint requestedPointA = {0, 0, 0};
CartesianPoint requestedPointB = {0, 0, 0};
bool cacheValid = false;
bool playReverse = false;
uint16_t buildIndex = 0;
uint16_t buildCount = 0;

Mode mode = Mode::Idle;
Joint seekFrom = {0, 0, 0};
Joint seekTo = {0, 0, 0};
Profile seekProfile;
float motionTimeS = 0.0f;

// ---- Kéo thanh trượt (jog) ----
AxisTracker tracker[3];
Joint manualTarget = {0, 0, 0};
bool manualActive = false;

// ---- Giám sát ----
uint8_t lostTicks = 0;
uint8_t stallWindowTicks = 0;
uint8_t noProgressWindows = 0;
uint16_t settleTicks = 0;
int16_t trimBudget = 0;
uint8_t movingTrimTicks = 0;
int32_t stallWindowStartSteps = 0;
float stallWindowStartAngle = NAN;

// ---- Tiện ích gói tin ----
uint16_t packetCrc(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

void writePacket(uint8_t type, const uint8_t *payload, size_t payloadLength) {
  CommunicationPacket packet = {};
  packet.magic[0] = packetMagicFirst;
  packet.magic[1] = packetMagicSecond;
  packet.version = packetVersion;
  packet.type = type;
  if (payload != nullptr && payloadLength > 0) {
    if (payloadLength > sizeof(packet.payload)) payloadLength = sizeof(packet.payload);
    memcpy(packet.payload, payload, payloadLength);
  }
  packet.crc = packetCrc(reinterpret_cast<const uint8_t *>(&packet),
                         sizeof(packet) - sizeof(packet.crc));
  Serial.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
}

uint8_t errorCodeForMessage(const char *message) {
  if (strcmp(message, "trajectory is already running or preparing") == 0)
    return errorTrajectoryBusy;
  if (strcmp(message, "stepper position is not ready") == 0)
    return errorStepperNotReady;
  if (strcmp(message, "point A is unreachable or violates joint limits") == 0)
    return errorPointInvalid;
  if (strcmp(message, "joint move to point A crosses a forbidden angle") == 0 ||
      strcmp(message, "joint interpolation crosses a forbidden angle") == 0)
    return errorJointPathInvalid;
  if (strcmp(message, "stop trajectory before setting joint angles") == 0)
    return errorStopBeforeSet;
  if (strcmp(message, "angle outside joint limits") == 0)
    return errorAngleLimit;
  if (strcmp(message, "servo angles violate joint condition") == 0)
    return errorServoCondition;
  if (strcmp(message, "stepper feedback is not ready") == 0)
    return errorStepperFeedbackNotReady;
  if (strcmp(message, "trajectory contains an unreachable or forbidden point") == 0)
    return errorTrajectoryPointInvalid;
  if (strcmp(message, "trajectory cache capacity exceeded") == 0)
    return errorTrajectoryCacheFull;
  if (strcmp(message, "trajectory point violates joint limits") == 0)
    return errorTrajectoryJointLimit;
  if (strcmp(message, "AS5600 feedback lost or outside step limits") == 0 ||
      strcmp(message, "AS5600 feedback lost while preparing trajectory") == 0)
    return errorAs5600Lost;
  if (strcmp(message, "joint target became invalid; motion stopped") == 0)
    return errorJointTargetInvalid;
  if (strcmp(message, "trajectory point is not finite") == 0)
    return errorPointNotFinite;
  if (strcmp(message, "unknown binary command") == 0)
    return errorUnknownCommand;
  if (strcmp(message, "joint command is not finite") == 0)
    return errorJointCommandNotFinite;
  if (strcmp(message, "stepper lost steps or is stalled") == 0)
    return errorStepperStall;
  if (strcmp(message, "sequence is full (maximum 8 points)") == 0)
    return errorSequenceFull;
  if (strcmp(message, "sequence requires at least 2 points to play") == 0)
    return errorSequenceEmpty;
  if (strcmp(message, "cannot save point while motion is active") == 0 ||
      strcmp(message, "cannot clear sequence while motion is active") == 0)
    return errorSequenceBusy;
  return errorCommandInvalid;
}

// ---- Khớp / giới hạn ----
SpeedLimits speedLimits() {
  return { SERVO_MAX_SPEED_DEG_PER_SEC, SERVO_MAX_SPEED_DEG_PER_SEC,
           STEPPER_MAX_SPEED_DEG_PER_SEC };
}

Joint commandedJoint(const RobotControlState &s) {
  return { s.servo1Angle, s.servo2Angle, s.stepTargetAngle };
}

CartesianPoint lerpPoint(const CartesianPoint &a, const CartesianPoint &b, float t) {
  return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
}

bool limitsOk(const Joint &j) {
  return isfinite(j.s1) && isfinite(j.s2) && isfinite(j.st) &&
         j.s1 >= SERVO1_MIN_DEG - jointLimitEpsilonDeg &&
         j.s1 <= SERVO1_MAX_DEG + jointLimitEpsilonDeg &&
         j.s2 >= SERVO2_MIN_DEG - jointLimitEpsilonDeg &&
         j.s2 <= SERVO2_MAX_DEG + jointLimitEpsilonDeg &&
         j.st >= STEPPER_MIN_DEG - jointLimitEpsilonDeg &&
         j.st <= STEPPER_MAX_DEG + jointLimitEpsilonDeg;
}

Joint clampJoint(Joint j) {
  j.s1 = fminf(SERVO1_MAX_DEG, fmaxf(SERVO1_MIN_DEG, j.s1));
  j.s2 = fminf(SERVO2_MAX_DEG, fmaxf(SERVO2_MIN_DEG, j.s2));
  j.st = fminf(STEPPER_MAX_DEG, fmaxf(STEPPER_MIN_DEG, j.st));
  return j;
}

bool validJoint(const Joint &j) {
  return limitsOk(j) && checkCondition(j.s1, j.s2);
}

bool solvePoint(const CartesianPoint &p, Joint &out) {
  float relServo2;
  Joint j;
  if (!InvertKinematics(p.x, p.y, p.z, &j.s1, &relServo2, &j.st)) return false;
  j.s2 = j.s1 + relServo2;
  if (!validJoint(j)) return false;
  out = clampJoint(j);
  return true;
}

// Kiểm tra đoạn thẳng trong không gian khớp (mẫu mỗi 0.5 độ).
bool validateJointSegment(const Joint &a, const Joint &b) {
  const float dist = fmaxf(fabsf(b.s1 - a.s1),
                           fmaxf(fabsf(b.s2 - a.s2), fabsf(b.st - a.st)));
  float frames = ceilf(dist / 0.5f);
  if (frames < 1.0f) frames = 1.0f;
  if (frames > 720.0f) frames = 720.0f;
  const uint16_t n = (uint16_t)frames;
  for (uint16_t i = 0; i <= n; ++i) {
    if (!validJoint(lerpJoint(a, b, (float)i / (float)n))) return false;
  }
  return true;
}

bool sameCartesianPoint(const CartesianPoint &l, const CartesianPoint &r) {
  constexpr float tol = 0.005f;
  return fabsf(l.x - r.x) <= tol && fabsf(l.y - r.y) <= tol && fabsf(l.z - r.z) <= tol;
}

// ---- Gửi dữ liệu ----
void sendAllSequencePoints() {
  if (sequenceCount == 0) {
    uint8_t payload[24] = {};
    payload[0] = 0;
    payload[1] = 0;
    writePacket(packetSequencePoint, payload, sizeof(payload));
    return;
  }
  for (uint8_t i = 0; i < sequenceCount; ++i) {
    uint8_t payload[24] = {};
    payload[0] = i;
    payload[1] = sequenceCount;
    memcpy(&payload[4], &sequencePoints[i].x, sizeof(float));
    memcpy(&payload[8], &sequencePoints[i].y, sizeof(float));
    memcpy(&payload[12], &sequencePoints[i].z, sizeof(float));
    memcpy(&payload[16], &sequencePoints[i].wrist, sizeof(float));
    memcpy(&payload[20], &sequencePoints[i].gripper, sizeof(float));
    writePacket(packetSequencePoint, payload, sizeof(payload));
  }
}

void sendPreparingAck() {
  uint8_t payload[24] = {};
  payload[0] = 1;
  payload[1] = ackDetailPreparing;
  writePacket(packetAck, payload, sizeof(payload));
}

void sendState(const RobotControlState &state) {
  float x = NAN, y = NAN, z = NAN;
  if (isfinite(state.measuredStepAngle)) {
    ForwardKinematics(state.servo1Angle, state.servo2Angle - state.servo1Angle,
                      state.measuredStepAngle, &x, &y, &z);
  }
  uint8_t payload[24] = {};
  memcpy(&payload[0], &state.servo1Angle, sizeof(float));
  memcpy(&payload[4], &state.servo2Angle, sizeof(float));
  memcpy(&payload[8], &x, sizeof(float));
  memcpy(&payload[12], &y, sizeof(float));
  memcpy(&payload[16], &z, sizeof(float));
  payload[20] = (state.trajectoryActive ? 1 : 0) |
                (state.trajectoryPreparing ? 2 : 0) |
                (activeSeqPlayback ? 4 : 0);
  uint8_t progress = 0;
  if (mode == Mode::BakingSequence && sequenceCount > 1) {
    uint32_t totalSegs = sequenceCount - 1;
    uint32_t segPercent = (uint32_t)bakeSegIndex * 100UL;
    if (bakePointCount > 0) {
      segPercent += (uint32_t)bakePointIndex * 100UL / bakePointCount;
    }
    progress = (uint8_t)(segPercent / totalSegs);
  } else if (mode == Mode::Building && buildCount > 0) {
    progress = (uint8_t)((uint32_t)buildIndex * 100UL / buildCount);
  } else if (mode == Mode::PlayingSequence && bakedSegmentCount > 0) {
    progress = (uint8_t)((uint32_t)activeSegIndex * 100UL / bakedSegmentCount);
  } else if (isSequenceBaked) {
    progress = 100;
  }
  payload[21] = progress;
  payload[22] = sequenceCount;
  payload[23] = (uint8_t)activeSegIndex;
  writePacket(packetState, payload, sizeof(payload));
}

void sendAck(bool accepted, const char *error = nullptr, uint32_t durationMs = 0) {
  uint8_t payload[24] = {};
  payload[0] = accepted ? 1 : 0;
  payload[1] = error != nullptr ? errorCodeForMessage(error) : (uint8_t)errorNone;
  memcpy(&payload[2], &durationMs, sizeof(durationMs));
  writePacket(packetAck, payload, sizeof(payload));
}

float readPacketFloat(const CommunicationPacket &packet, size_t offset) {
  float value;
  memcpy(&value, &packet.payload[offset], sizeof(value));
  return value;
}

// ---- Điều khiển chuyển động ----
void haltMotion(RobotControlState &state) {
  if (mode == Mode::Building || mode == Mode::BakingSequence) cacheValid = false;
  mode = Mode::Idle;
  manualActive = false;
  sequenceActive = false;
  activeSeqPlayback = false;
  playSubState = PlaySubState::MovingArm;
  settleTimerTicks = 0;
  state.trajectoryActive = false;
  state.trajectoryPreparing = false;
  state.feedbackActive = false;
  stallWindowTicks = 0;
  noProgressWindows = 0;
  // Không còn hàng đợi xung: dừng ngay tại vị trí lệnh hiện tại.
}

void failMotion(RobotControlState &state, const char *message) {
  haltMotion(state);
  robotCommsSendError(message);
}

void beginSeek(RobotControlState &state, const Joint &from, const Joint &to) {
  seekFrom = from;
  seekTo = to;
  seekProfile.init(minMoveTime(from, to, speedLimits()), MOTION_ACCEL_TIME_S);
  motionTimeS = 0.0f;
  mode = Mode::Seeking;
  manualActive = false;
  trimBudget = STEPPER_TRIM_BUDGET_STEPS;
  stallWindowTicks = 0;
  noProgressWindows = 0;
  stallWindowStartSteps = getStepperSteps();
  stallWindowStartAngle = state.measuredStepAngle;
  movingTrimTicks = 0;
  state.trajectoryPreparing = false;
  state.trajectoryActive = true;
  state.feedbackActive = true;
}

uint32_t plannedDurationMs() {
  return (uint32_t)((seekProfile.T + path.duration()) * 1000.0f);
}

bool launchTrajectory(const CartesianPoint &pointA, const CartesianPoint &pointB,
                      RobotControlState &state, const char **errMsg) {
  if (!state.stepperReady || !isfinite(state.measuredStepAngle)) {
    if (errMsg) *errMsg = "stepper position is not ready";
    return false;
  }
  Joint targetA;
  if (!solvePoint(pointA, targetA)) {
    if (errMsg) *errMsg = "point A is unreachable or violates joint limits";
    return false;
  }
  const Joint current = commandedJoint(state);
  if (!validJoint(current) || !validateJointSegment(current, targetA)) {
    if (errMsg) *errMsg = "joint move to point A crosses a forbidden angle";
    return false;
  }

  manualActive = false;
  requestedPointA = pointA;
  requestedPointB = pointB;

  const bool forwardHit = cacheValid && sameCartesianPoint(pointA, cachedPointA) &&
                          sameCartesianPoint(pointB, cachedPointB);
  const bool reverseHit = cacheValid && sameCartesianPoint(pointA, cachedPointB) &&
                          sameCartesianPoint(pointB, cachedPointA);
  if (forwardHit || reverseHit) {
    playReverse = reverseHit;
    const Joint start = playReverse ? path.last() : path.first();
    if (!validateJointSegment(current, start)) {
      if (errMsg) *errMsg = "joint move to point A crosses a forbidden angle";
      return false;
    }
    beginSeek(state, current, start);
    return true;
  }

  const float dx = pointB.x - pointA.x, dy = pointB.y - pointA.y, dz = pointB.z - pointA.z;
  const float len = sqrtf(dx * dx + dy * dy + dz * dz);
  float n = ceilf(len / PATH_SAMPLE_MM) + 1.0f;
  if (n < 2.0f) n = 2.0f;
  if (n > (float)Trajectory::kMaxInputPoints) {
    n = (float)Trajectory::kMaxInputPoints;
  }
  buildCount = (uint16_t)n;
  buildIndex = 0;
  playReverse = false;
  path.clear();
  cacheValid = false;
  mode = Mode::Building;
  state.trajectoryActive = false;
  state.trajectoryPreparing = true;
  state.feedbackActive = false;
  return true;
}

void startPlaybackAfterBaked(int8_t direction, RobotControlState &state) {
  if (bakedSegmentCount == 0) return;
  sequenceDirection = 1; // Chỉ chạy xuôi
  sequenceActive = true;
  activeSegIndex = 0;
  activeStepIndex = 0;

  Joint targetStart = bakedSegments[0].steps[0];
  state.wristAngle = sequencePoints[0].wrist;
  state.gripperAngle = sequencePoints[0].gripper;
  playSubState = PlaySubState::MovingArm;

  const Joint current = commandedJoint(state);
  const float diff = fmaxf(fabsf(targetStart.s1 - current.s1),
                     fmaxf(fabsf(targetStart.s2 - current.s2),
                           fabsf(targetStart.st - current.st)));

  manualActive = false;
  activeSeqPlayback = true;

  if (diff > 0.5f) {
    if (!validateJointSegment(current, targetStart)) {
      failMotion(state, "joint move to sequence start crosses a forbidden angle");
      return;
    }
    beginSeek(state, current, targetStart);
  } else {
    mode = Mode::PlayingSequence;
    motionTimeS = 0.0f;
    state.trajectoryPreparing = false;
    state.trajectoryActive = true;
    state.feedbackActive = true;
    trimBudget = STEPPER_TRIM_BUDGET_STEPS;
    stallWindowTicks = 0;
    noProgressWindows = 0;
    stallWindowStartSteps = getStepperSteps();
    stallWindowStartAngle = state.measuredStepAngle;
    movingTrimTicks = 0;
  }
  runPwm(state.servo1Angle, state.servo2Angle, state.wristAngle, state.gripperAngle);
}

bool initBakeSegment(uint8_t segIdx, RobotControlState &state) {
  if (segIdx >= sequenceCount - 1) return false;
  bakeSegIndex = segIdx;
  bakeFrom = { sequencePoints[segIdx].x, sequencePoints[segIdx].y, sequencePoints[segIdx].z };
  bakeTo = { sequencePoints[segIdx + 1].x, sequencePoints[segIdx + 1].y, sequencePoints[segIdx + 1].z };

  Joint targetA, targetB;
  if (!solvePoint(bakeFrom, targetA) || !solvePoint(bakeTo, targetB)) {
    failMotion(state, "trajectory contains an unreachable or forbidden point");
    return false;
  }
  const float dx = bakeTo.x - bakeFrom.x;
  const float dy = bakeTo.y - bakeFrom.y;
  const float dz = bakeTo.z - bakeFrom.z;
  const float len = sqrtf(dx * dx + dy * dy + dz * dz);
  float n = ceilf(len / PATH_SAMPLE_MM) + 1.0f;
  if (n < 2.0f) n = 2.0f;
  if (n > (float)Trajectory::kMaxInputPoints) n = (float)Trajectory::kMaxInputPoints;
  bakePointCount = (uint16_t)n;
  bakePointIndex = 0;
  path.clear();
  return true;
}

void bakeStep(RobotControlState &state) {
  const float t = (float)bakePointIndex / (float)(bakePointCount - 1);
  const CartesianPoint p = lerpPoint(bakeFrom, bakeTo, t);
  Joint j;
  if (!solvePoint(p, j)) {
    failMotion(state, "trajectory contains an unreachable or forbidden point");
    return;
  }
  if (!path.append(j)) {
    failMotion(state, "trajectory cache capacity exceeded");
    return;
  }
  ++bakePointIndex;
  if (bakePointIndex < bakePointCount) return;

  if (!path.finalize(speedLimits(), MOTION_ACCEL_TIME_S)) {
    failMotion(state, "trajectory contains an unreachable or forbidden point");
    return;
  }

  const float duration = path.duration();
  uint16_t totalSteps = (uint16_t)ceilf(duration / loopDtS);
  if (totalSteps < 1) totalSteps = 1;
  if (totalSteps > MAX_STEPS_PER_SEGMENT) totalSteps = MAX_STEPS_PER_SEGMENT;

  for (uint16_t s = 0; s < totalSteps; ++s) {
    const float timeS = (float)s * loopDtS;
    bakedSegments[bakeSegIndex].steps[s] = path.sample(timeS, false);
  }
  bakedSegments[bakeSegIndex].steps[totalSteps - 1] = path.last();
  bakedSegments[bakeSegIndex].count = totalSteps;

  ++bakeSegIndex;
  if (bakeSegIndex < sequenceCount - 1) {
    initBakeSegment(bakeSegIndex, state);
  } else {
    bakedSegmentCount = sequenceCount - 1;
    isSequenceBaked = true;
    state.trajectoryPreparing = false;
    startPlaybackAfterBaked(1, state);
  }
}

void saveCurrentPointToFlash(const RobotControlState &state) {
  if (sequenceCount >= MAX_SEQUENCE_POINTS) {
    sendAck(false, "sequence is full (maximum 8 points)");
    return;
  }
  if (!state.stepperReady || !isfinite(state.measuredStepAngle)) {
    sendAck(false, "stepper position is not ready");
    return;
  }
  if (mode != Mode::Idle || state.trajectoryActive || state.trajectoryPreparing || manualActive) {
    sendAck(false, "cannot save point while motion is active");
    return;
  }
  float x = NAN, y = NAN, z = NAN;
  ForwardKinematics(state.servo1Angle, state.servo2Angle - state.servo1Angle,
                    state.measuredStepAngle, &x, &y, &z);
  if (!isfinite(x) || !isfinite(y) || !isfinite(z)) {
    sendAck(false, "current position is not finite");
    return;
  }
  sequencePoints[sequenceCount] = { x, y, z, state.wristAngle, state.gripperAngle };
  sequenceCount++;
  preferences.putBytes("pts", sequencePoints, sizeof(StoredPoint) * sequenceCount);
  preferences.putUChar("count", sequenceCount);
  isSequenceBaked = false;
  bakedSegmentCount = 0;
  sendAck(true);
  sendAllSequencePoints();
}

void clearSequencePointsInFlash(const RobotControlState &state) {
  if (sequenceActive || mode != Mode::Idle || state.trajectoryActive || state.trajectoryPreparing || manualActive) {
    sendAck(false, "cannot clear sequence while motion is active");
    return;
  }
  sequenceCount = 0;
  preferences.putUChar("count", 0);
  isSequenceBaked = false;
  bakedSegmentCount = 0;
  sendAck(true);
  sendAllSequencePoints();
}

void processCommand(const CommunicationPacket &packet, RobotControlState &state) {
  if (packet.type == commandGetState) {
    sendState(state);
    return;
  }

  if (packet.type == commandGetPoints) {
    sendAllSequencePoints();
    return;
  }

  if (packet.type == commandSavePoint) {
    saveCurrentPointToFlash(state);
    return;
  }

  if (packet.type == commandClearPoints) {
    clearSequencePointsInFlash(state);
    return;
  }

  if (packet.type == commandPlaySequence) {
    if (mode != Mode::Idle || state.trajectoryActive || state.trajectoryPreparing || manualActive) {
      sendAck(false, "trajectory is already running or preparing");
      return;
    }
    if (sequenceCount < 2) {
      sendAck(false, "sequence requires at least 2 points to play");
      return;
    }
    sequenceDirection = 1; // Chỉ chạy xuôi theo chuỗi hành động

    if (isSequenceBaked && bakedSegmentCount == sequenceCount - 1) {
      startPlaybackAfterBaked(1, state);
      sendAck(true);
      return;
    }

    isSequenceBaked = false;
    bakedSegmentCount = 0;
    bakeSegIndex = 0;
    if (!initBakeSegment(0, state)) {
      sendAck(false, "point A is unreachable or violates joint limits");
      return;
    }
    state.trajectoryPreparing = true;
    mode = Mode::BakingSequence;
    sendPreparingAck();
    return;
  }

  if (packet.type == commandPlayTrajectory) {
    const CartesianPoint pointA = { readPacketFloat(packet, 0), readPacketFloat(packet, 4),
                                    readPacketFloat(packet, 8) };
    const CartesianPoint pointB = { readPacketFloat(packet, 12), readPacketFloat(packet, 16),
                                    readPacketFloat(packet, 20) };
    if (!isfinite(pointA.x) || !isfinite(pointA.y) || !isfinite(pointA.z) ||
        !isfinite(pointB.x) || !isfinite(pointB.y) || !isfinite(pointB.z)) {
      sendAck(false, "trajectory point is not finite");
      return;
    }
    if (mode != Mode::Idle || state.trajectoryActive || state.trajectoryPreparing || manualActive) {
      sendAck(false, "trajectory is already running or preparing");
      return;
    }
    sequenceActive = false;
    const char *errMsg = nullptr;
    if (!launchTrajectory(pointA, pointB, state, &errMsg)) {
      sendAck(false, errMsg ? errMsg : "point A is unreachable or violates joint limits");
      return;
    }
    if (mode == Mode::Seeking) {
      sendAck(true, nullptr, plannedDurationMs());
    } else {
      sendPreparingAck();
    }
    return;
  }

  if (packet.type == commandStop) {
    haltMotion(state);
    sendAck(true);
    return;
  }

  if (packet.type != commandSetJoints) {
    sendAck(false, "unknown binary command");
    return;
  }

  Joint request = { readPacketFloat(packet, 0), readPacketFloat(packet, 4),
                    readPacketFloat(packet, 8) };
  if (!isfinite(request.s1) || !isfinite(request.s2) || !isfinite(request.st)) {
    sendAck(false, "joint command is not finite");
    return;
  }
  if (mode != Mode::Idle || state.trajectoryActive || state.trajectoryPreparing) {
    sendAck(false, "stop trajectory before setting joint angles");
    return;
  }
  if (!limitsOk(request)) {
    sendAck(false, "angle outside joint limits");
    return;
  }
  request = clampJoint(request);
  if (!checkCondition(request.s1, request.s2)) {
    sendAck(false, "servo angles violate joint condition");
    return;
  }
  if (!state.stepperReady) {
    sendAck(false, "stepper feedback is not ready");
    return;
  }

  const float wristReq = readPacketFloat(packet, 12);
  const float gripReq = readPacketFloat(packet, 16);
  bool auxChanged = false;
  if (isfinite(wristReq)) {
    const float clampedWrist = fminf(WRIST_MAX_DEG, fmaxf(WRIST_MIN_DEG, wristReq));
    if (state.wristAngle != clampedWrist) {
      state.wristAngle = clampedWrist;
      auxChanged = true;
    }
  }
  if (isfinite(gripReq)) {
    const float clampedGrip = fminf(GRIPPER_MAX_DEG, fmaxf(GRIPPER_MIN_DEG, gripReq));
    if (state.gripperAngle != clampedGrip) {
      state.gripperAngle = clampedGrip;
      auxChanged = true;
    }
  }
  if (auxChanged) {
    runPwm(state.servo1Angle, state.servo2Angle, state.wristAngle, state.gripperAngle);
  }

  if (!manualActive) {
    tracker[0].reset(state.servo1Angle);
    tracker[1].reset(state.servo2Angle);
    tracker[2].reset(state.stepTargetAngle);
  }
  manualTarget = request;
  manualActive = true;
  trimBudget = STEPPER_TRIM_BUDGET_STEPS;
  stallWindowTicks = 0;
  noProgressWindows = 0;
  stallWindowStartSteps = getStepperSteps();
  stallWindowStartAngle = state.measuredStepAngle;
  movingTrimTicks = 0;
  state.feedbackActive = true;
  sendAck(true);
}

void consumeReceivedBytes(RobotControlState &state) {
  constexpr size_t maxBytesPerPoll = sizeof(CommunicationPacket) * 4;
  size_t bytesRead = 0;
  while (Serial.available() > 0 && bytesRead < maxBytesPerPoll) {
    if (receiveLength >= sizeof(receiveBuffer)) {
      memmove(receiveBuffer, receiveBuffer + 1, --receiveLength);
    }
    receiveBuffer[receiveLength++] = (uint8_t)Serial.read();
    ++bytesRead;

    size_t headerOffset = 0;
    while (headerOffset + 1 < receiveLength &&
           (receiveBuffer[headerOffset] != packetMagicFirst ||
            receiveBuffer[headerOffset + 1] != packetMagicSecond)) {
      ++headerOffset;
    }
    if (headerOffset > 0) {
      memmove(receiveBuffer, receiveBuffer + headerOffset, receiveLength - headerOffset);
      receiveLength -= headerOffset;
    }
    if (receiveLength < 2 || receiveBuffer[0] != packetMagicFirst ||
        receiveBuffer[1] != packetMagicSecond ||
        receiveLength < sizeof(CommunicationPacket)) {
      continue;
    }

    if (receiveBuffer[2] != packetVersion ||
        packetCrc(receiveBuffer, sizeof(CommunicationPacket) - 2) !=
          (uint16_t)(receiveBuffer[sizeof(CommunicationPacket) - 2] |
                     ((uint16_t)receiveBuffer[sizeof(CommunicationPacket) - 1] << 8))) {
      memmove(receiveBuffer, receiveBuffer + 1, --receiveLength);
      continue;
    }

    CommunicationPacket packet;
    memcpy(&packet, receiveBuffer, sizeof(packet));
    memmove(receiveBuffer, receiveBuffer + sizeof(packet), receiveLength - sizeof(packet));
    receiveLength -= sizeof(packet);
    processCommand(packet, state);
  }
}

// Mỗi tick đúng 1 lần IK. Điểm cuối xong thì hậu xử lý (tính thời gian + profile)
// rồi chuyển sang seek.
void buildStep(RobotControlState &state) {
  const float t = (float)buildIndex / (float)(buildCount - 1);
  const CartesianPoint p = lerpPoint(requestedPointA, requestedPointB, t);
  Joint j;
  if (!solvePoint(p, j)) {
    failMotion(state, "trajectory contains an unreachable or forbidden point");
    return;
  }
  if (!path.append(j)) {
    failMotion(state, "trajectory cache capacity exceeded");
    return;
  }
  ++buildIndex;
  if (buildIndex < buildCount) return;

  if (!path.finalize(speedLimits(), MOTION_ACCEL_TIME_S)) {
    failMotion(state, "trajectory contains an unreachable or forbidden point");
    return;
  }
  cachedPointA = requestedPointA;
  cachedPointB = requestedPointB;
  cacheValid = true;

  const Joint current = commandedJoint(state);
  if (!validateJointSegment(current, path.first())) {
    mode = Mode::Idle;
    state.trajectoryPreparing = false;
    robotCommsSendError("joint move to point A crosses a forbidden angle");
    return;
  }
  beginSeek(state, current, path.first());
}

void applyCommand(RobotControlState &state, const Joint &cmd) {
  const bool servoChanged = cmd.s1 != state.servo1Angle || cmd.s2 != state.servo2Angle;
  state.servo1Angle = cmd.s1;
  state.servo2Angle = cmd.s2;
  state.stepTargetAngle = cmd.st;
  if (servoChanged) runPwm(cmd.s1, cmd.s2, state.wristAngle, state.gripperAngle);
}

bool motionActive() {
  return (mode != Mode::Idle && mode != Mode::Building && mode != Mode::BakingSequence) || manualActive;
}

void finishTrajectory(RobotControlState &state) {
  mode = Mode::Idle;
  state.trajectoryActive = false;
  state.feedbackActive = false;
  activeSeqPlayback = false;
  sequenceActive = false;
  playSubState = PlaySubState::MovingArm;
}
}  // namespace

void robotCommsBegin() {
  Serial.begin(115200);
  preferences.begin("arm_seq", false);
  sequenceCount = preferences.getUChar("count", 0);
  if (sequenceCount > MAX_SEQUENCE_POINTS) {
    sequenceCount = 0;
  }
  if (sequenceCount > 0) {
    preferences.getBytes("pts", sequencePoints, sizeof(StoredPoint) * sequenceCount);
  }
  isSequenceBaked = false;
  bakedSegmentCount = 0;
  lastTelemetryMs = millis();
}

void robotCommsPoll(RobotControlState &state) {
  consumeReceivedBytes(state);
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastTelemetryMs) >= telemetryPeriodMs) {
    lastTelemetryMs = nowMs;
    sendState(state);
  }
}

void robotCommsRunTick(RobotControlState &state) {
  if (!state.stepperReady) return;

  Joint cmd = commandedJoint(state);

  switch (mode) {
    case Mode::Building:
      buildStep(state);
      break;

    case Mode::BakingSequence:
      bakeStep(state);
      break;

    case Mode::Seeking: {
      motionTimeS += loopDtS;
      const float x = seekProfile.position(motionTimeS);
      const float f = seekProfile.L > 0.0f ? x / seekProfile.L : 1.0f;
      cmd = lerpJoint(seekFrom, seekTo, f);
      if (motionTimeS >= seekProfile.T) {
        cmd = seekTo;
        motionTimeS = 0.0f;
        if (activeSeqPlayback) {
          mode = Mode::PlayingSequence;
          playSubState = PlaySubState::MovingArm;
          activeSegIndex = 0;
          activeStepIndex = 0;
          settleTimerTicks = 0;
          state.trajectoryActive = true;
          state.feedbackActive = true;
          stallWindowTicks = 0;
          noProgressWindows = 0;
          stallWindowStartSteps = getStepperSteps();
          stallWindowStartAngle = state.measuredStepAngle;
          movingTrimTicks = 0;
        } else {
          mode = Mode::Playing;
        }
      }
      break;
    }

    case Mode::PlayingSequence: {
      if (activeSegIndex >= 0 && activeSegIndex < bakedSegmentCount) {
        const SegmentCache &seg = bakedSegments[activeSegIndex];
        const uint16_t lastStepIdx = seg.count > 0 ? seg.count - 1 : 0;

        if (playSubState == PlaySubState::MovingArm) {
          if (activeStepIndex < seg.count) {
            cmd = seg.steps[activeStepIndex];
            ++activeStepIndex;
          }

          if (activeStepIndex >= seg.count) {
            const uint8_t targetPtIdx = activeSegIndex + 1;
            const float targetWrist = sequencePoints[targetPtIdx].wrist;
            const float targetGripper = sequencePoints[targetPtIdx].gripper;

            const bool wristDiffers = fabsf(targetWrist - state.wristAngle) > 0.1f;
            const bool gripperDiffers = fabsf(targetGripper - state.gripperAngle) > 0.1f;

            if (wristDiffers) {
              wristLerpFrom = state.wristAngle;
              wristLerpTo = targetWrist;
              playSubState = PlaySubState::SettleWrist;
              settleTimerTicks = (uint16_t)(WRIST_SETTLE_MS / (loopDtS * 1000.0f));
              if (settleTimerTicks < 1) settleTimerTicks = 1;
              wristTotalTicks = settleTimerTicks;
            } else if (gripperDiffers) {
              state.gripperAngle = targetGripper;
              runPwm(state.servo1Angle, state.servo2Angle, state.wristAngle, state.gripperAngle);
              playSubState = PlaySubState::SettleGripper;
              settleTimerTicks = (uint16_t)(GRIPPER_SETTLE_MS / (loopDtS * 1000.0f));
            } else {
              ++activeSegIndex;
              activeStepIndex = 0;
              if (activeSegIndex >= bakedSegmentCount) {
                settleTicks = 0;
                mode = Mode::Settling;
              } else {
                playSubState = PlaySubState::MovingArm;
              }
            }
          }
        } else if (playSubState == PlaySubState::SettleWrist) {
          cmd = seg.steps[lastStepIdx];
          if (settleTimerTicks > 0) {
            --settleTimerTicks;
            const float t = 1.0f - ((float)settleTimerTicks / (float)wristTotalTicks);
            const float s = t * t * (3.0f - 2.0f * t);
            state.wristAngle = wristLerpFrom + (wristLerpTo - wristLerpFrom) * s;
            runPwm(state.servo1Angle, state.servo2Angle, state.wristAngle, state.gripperAngle);
          }
          if (settleTimerTicks == 0) {
            state.wristAngle = wristLerpTo;
            runPwm(state.servo1Angle, state.servo2Angle, state.wristAngle, state.gripperAngle);
            const uint8_t targetPtIdx = activeSegIndex + 1;
            const float targetGripper = sequencePoints[targetPtIdx].gripper;
            const bool gripperDiffers = fabsf(targetGripper - state.gripperAngle) > 0.1f;

            if (gripperDiffers) {
              state.gripperAngle = targetGripper;
              runPwm(state.servo1Angle, state.servo2Angle, state.wristAngle, state.gripperAngle);
              playSubState = PlaySubState::SettleGripper;
              settleTimerTicks = (uint16_t)(GRIPPER_SETTLE_MS / (loopDtS * 1000.0f));
            } else {
              ++activeSegIndex;
              activeStepIndex = 0;
              if (activeSegIndex >= bakedSegmentCount) {
                settleTicks = 0;
                mode = Mode::Settling;
              } else {
                playSubState = PlaySubState::MovingArm;
              }
            }
          }
        } else if (playSubState == PlaySubState::SettleGripper) {
          cmd = seg.steps[lastStepIdx];
          if (settleTimerTicks > 0) {
            --settleTimerTicks;
          }
          if (settleTimerTicks == 0) {
            ++activeSegIndex;
            activeStepIndex = 0;
            if (activeSegIndex >= bakedSegmentCount) {
              settleTicks = 0;
              mode = Mode::Settling;
            } else {
              playSubState = PlaySubState::MovingArm;
            }
          }
        }
      } else {
        settleTicks = 0;
        mode = Mode::Settling;
      }
      break;
    }

    case Mode::Playing:
      motionTimeS += loopDtS;
      cmd = path.sample(motionTimeS, playReverse);
      if (motionTimeS >= path.duration()) {
        cmd = playReverse ? path.first() : path.last();
        settleTicks = 0;
        mode = Mode::Settling;
      }
      break;

    case Mode::Settling:
      break;

    case Mode::Idle:
      if (manualActive) {
        const SpeedLimits lim = speedLimits();
        tracker[0].update(manualTarget.s1, lim.s1, lim.s1 / MOTION_ACCEL_TIME_S, loopDtS);
        tracker[1].update(manualTarget.s2, lim.s2, lim.s2 / MOTION_ACCEL_TIME_S, loopDtS);
        tracker[2].update(manualTarget.st, lim.st, lim.st / MOTION_ACCEL_TIME_S, loopDtS);
        cmd = { tracker[0].pos, tracker[1].pos, tracker[2].pos };
        if (tracker[0].settled(manualTarget.s1) && tracker[1].settled(manualTarget.s2) &&
            tracker[2].settled(manualTarget.st)) {
          manualActive = false;
          state.feedbackActive = false;
        }
      }
      break;
  }

  applyCommand(state, cmd);
  stepperTrack(state.stepTargetAngle);  // tối đa 1 xung, đúng nhịp tick
}

void robotCommsFeedback(RobotControlState &state, float measuredAngleDeg) {
  if (!state.stepperReady) return;

  const bool valid = isfinite(measuredAngleDeg) && measuredAngleDeg >= 0.0f &&
                     measuredAngleDeg < 360.0f;
  if (!valid) {
    if (lostTicks < 255) ++lostTicks;
    if (lostTicks >= AS5600_LOST_TICKS) {
      state.measuredStepAngle = NAN;
      if (motionActive()) {
        failMotion(state, "AS5600 feedback lost or outside step limits");
      } else if (mode == Mode::Building || mode == Mode::BakingSequence) {
        failMotion(state, "AS5600 feedback lost while preparing trajectory");
      }
    }
    return;  // lỗi I2C thoáng qua: giữ giá trị cũ, không hủy chuyển động
  }
  lostTicks = 0;
  state.measuredStepAngle = measuredAngleDeg;

  const bool moving = motionActive();
  if (moving && (measuredAngleDeg < STEPPER_MIN_DEG - STEPPER_LIMIT_MARGIN_DEG ||
                 measuredAngleDeg > STEPPER_MAX_DEG + STEPPER_LIMIT_MARGIN_DEG)) {
    failMotion(state, "AS5600 feedback lost or outside step limits");
    return;
  }

  const float err = state.stepTargetAngle - measuredAngleDeg;  // > 0: trục ra đang chậm
  const float absErr = fabsf(err);

  // Chỉ báo kẹt nếu đã phát đủ xung nhưng AS5600 gần như không thấy trục dịch chuyển.
  if (moving && mode != Mode::Settling) {
    if (!isfinite(stallWindowStartAngle)) {
      stallWindowStartSteps = getStepperSteps();
      stallWindowStartAngle = measuredAngleDeg;
      stallWindowTicks = 0;
      noProgressWindows = 0;
    } else if (++stallWindowTicks >= STEPPER_STALL_WINDOW_TICKS) {
      const int32_t stepDelta = getStepperSteps() - stallWindowStartSteps;
      float measuredDelta = measuredAngleDeg - stallWindowStartAngle;
      if (measuredDelta > 180.0f) measuredDelta -= 360.0f;
      else if (measuredDelta < -180.0f) measuredDelta += 360.0f;

      if (abs(stepDelta) >= STEPPER_STALL_MIN_PULSES) {
        const float directedProgress =
            measuredDelta * (stepDelta > 0 ? 1.0f : -1.0f);
        if (directedProgress < STEPPER_STALL_MIN_PROGRESS_DEG) {
          if (++noProgressWindows >= STEPPER_STALL_NO_PROGRESS_WINDOWS) {
            failMotion(state, "stepper lost steps or is stalled");
            return;
          }
        } else {
          noProgressWindows = 0;
        }
      } else {
        noProgressWindows = 0;
      }
      stallWindowStartSteps = getStepperSteps();
      stallWindowStartAngle = measuredAngleDeg;
      stallWindowTicks = 0;
    }
  } else if (mode != Mode::Settling) {
    stallWindowStartSteps = getStepperSteps();
    stallWindowStartAngle = measuredAngleDeg;
    stallWindowTicks = 0;
    noProgressWindows = 0;
  }

  const bool settledForTrim =
      mode == Mode::Settling || (mode == Mode::Idle && !manualActive);
  const bool movingForTrim = motionActive() && mode != Mode::Settling;
  bool applyTrim = false;
  if (trimBudget > 0 && settledForTrim &&
      absErr > STEPPER_SETTLE_DEADBAND_DEG) {
    applyTrim = true;
  } else if (trimBudget > 0 && movingForTrim &&
             absErr > STEPPER_MOVING_TRIM_ERROR_DEG) {
    if (++movingTrimTicks >= STEPPER_MOVING_TRIM_INTERVAL_TICKS) {
      movingTrimTicks = 0;
      applyTrim = true;
    }
  } else {
    movingTrimTicks = 0;
  }
  if (applyTrim) {
    stepperAdjustTrim(err > 0.0f ? 1 : -1);
    --trimBudget;
  }

  if (mode == Mode::Settling) {
    ++settleTicks;
    if (absErr <= STEPPER_SETTLE_DEADBAND_DEG || settleTicks >= STEPPER_SETTLE_TIMEOUT_TICKS) {
      if (absErr > STEPPER_STALL_ERROR_DEG && settleTicks >= STEPPER_SETTLE_TIMEOUT_TICKS) {
        failMotion(state, "stepper lost steps or is stalled");
      } else {
        finishTrajectory(state);
      }
    }
  }
}

void robotCommsSendError(const char *message) {
  const uint8_t errorCode = errorCodeForMessage(message);
  uint8_t payload[24] = {};
  payload[0] = errorCode;
  writePacket(packetError, payload, sizeof(payload));
}
