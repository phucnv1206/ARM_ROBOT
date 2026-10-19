#include "robot_comms.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "PWM.h"
#include "config.h"
#include "kinematics.h"
#include "stepper.h"

namespace {
constexpr uint32_t telemetryPeriodMs = 100;
constexpr uint32_t controlPeriodMs = 1000UL / LOOP_FREQ;
constexpr uint16_t trajectoryPointCount = 1000;
constexpr uint8_t trajectoryPointsPerLoop = 4;
constexpr float maximumJointStepPerLoopDeg = JOINT_ANGLE_RESOLUTION_DEG;
constexpr uint8_t packetMagicFirst = 0x52;
constexpr uint8_t packetMagicSecond = 0x42;
constexpr uint8_t packetVersion = 1;
constexpr uint8_t packetState = 1;
constexpr uint8_t packetAck = 2;
constexpr uint8_t packetError = 3;
constexpr uint8_t commandGetState = 16;
constexpr uint8_t commandSetJoints = 17;
constexpr uint8_t commandPlayTrajectory = 18;
constexpr uint8_t commandStop = 19;
constexpr uint8_t ackDetailPreparing = 1;

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
  errorJointCommandNotFinite = 17
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

struct JointTarget {
  float servo1;
  float servo2;
  float step;
};

uint8_t receiveBuffer[sizeof(CommunicationPacket) * 4];
size_t receiveLength = 0;
uint32_t lastTelemetryMs = 0;
JointTarget trajectoryCache[trajectoryPointCount + 1];
float trajectoryDistance[trajectoryPointCount + 1];
CartesianPoint cachedPointA = {0.0f, 0.0f, 0.0f};
CartesianPoint cachedPointB = {0.0f, 0.0f, 0.0f};
CartesianPoint requestedPointA = {0.0f, 0.0f, 0.0f};
CartesianPoint requestedPointB = {0.0f, 0.0f, 0.0f};
JointTarget trajectorySeekStart = {0.0f, 0.0f, 0.0f};
uint16_t cacheBuildIndex = 0;
uint16_t trajectoryFrame = 0;
uint16_t trajectorySeekFrames = 0;
uint32_t trajectoryPlaybackFrame = 0;
uint32_t trajectoryPlaybackFrames = 0;
float trajectoryTotalDistance = 0.0f;
uint8_t trajectoryPhase = 0;
bool trajectoryCacheValid = false;
bool trajectoryCacheBuilding = false;

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

void writePacket(uint8_t type, const uint8_t *payload,
                 size_t payloadLength) {
  CommunicationPacket packet = {};
  packet.magic[0] = packetMagicFirst;
  packet.magic[1] = packetMagicSecond;
  packet.version = packetVersion;
  packet.type = type;
  if (payload != nullptr && payloadLength > 0) {
    if (payloadLength > sizeof(packet.payload)) {
      payloadLength = sizeof(packet.payload);
    }
    memcpy(packet.payload, payload, payloadLength);
  }
  packet.crc = packetCrc(
    reinterpret_cast<const uint8_t *>(&packet),
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
  if (strcmp(message,
             "trajectory contains an unreachable or forbidden point") == 0)
    return errorTrajectoryPointInvalid;
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
  return errorCommandInvalid;
}

float quantizeJointAngle(float angleDeg) {
  return roundf(angleDeg / JOINT_ANGLE_RESOLUTION_DEG) *
         JOINT_ANGLE_RESOLUTION_DEG;
}

JointTarget interpolate(const JointTarget &start,
                        const JointTarget &end, float t) {
  return {
    quantizeJointAngle(start.servo1 + (end.servo1 - start.servo1) * t),
    quantizeJointAngle(start.servo2 + (end.servo2 - start.servo2) * t),
    quantizeJointAngle(start.step + (end.step - start.step) * t)
  };
}

CartesianPoint interpolate(const CartesianPoint &start,
                           const CartesianPoint &end, float t) {
  return {
    start.x + (end.x - start.x) * t,
    start.y + (end.y - start.y) * t,
    start.z + (end.z - start.z) * t
  };
}

bool solvePoint(const CartesianPoint &point, JointTarget &target) {
  float relativeServo2;
  if (!InvertKinematics(point.x, point.y, point.z,
                        &target.servo1, &relativeServo2, &target.step)) {
    return false;
  }

  target.servo2 = target.servo1 + relativeServo2;
  return isfinite(target.servo1) && isfinite(target.servo2) &&
         isfinite(target.step) &&
         target.servo1 >= SERVO1_MIN_DEG &&
         target.servo1 <= SERVO1_MAX_DEG &&
         target.servo2 >= SERVO2_MIN_DEG &&
         target.servo2 <= SERVO2_MAX_DEG &&
         target.step >= STEPPER_MIN_DEG &&
         target.step <= STEPPER_MAX_DEG &&
         checkCondition(target.servo1, target.servo2);
}

bool validJointTarget(const JointTarget &target) {
  return target.servo1 >= SERVO1_MIN_DEG &&
         target.servo1 <= SERVO1_MAX_DEG &&
         target.servo2 >= SERVO2_MIN_DEG &&
         target.servo2 <= SERVO2_MAX_DEG &&
         target.step >= STEPPER_MIN_DEG &&
         target.step <= STEPPER_MAX_DEG &&
         checkCondition(target.servo1, target.servo2);
}

float jointDistance(const JointTarget &left, const JointTarget &right) {
  return fmaxf(fabsf(left.servo1 - right.servo1),
               fmaxf(fabsf(left.servo2 - right.servo2),
                     fabsf(left.step - right.step)));
}

bool validateJointSegment(const JointTarget &start, const JointTarget &end) {
  const uint16_t frames = (uint16_t)fmaxf(
    1.0f, ceilf(jointDistance(start, end) /
                maximumJointStepPerLoopDeg));
  for (uint16_t frame = 0; frame <= frames; ++frame) {
    const JointTarget target = interpolate(
      start, end, (float)frame / (float)frames);
    if (!validJointTarget(target)) {
      return false;
    }
  }
  return true;
}

bool sameCartesianPoint(const CartesianPoint &left,
                        const CartesianPoint &right) {
  constexpr float coordinateTolerance = 0.005f;
  return fabsf(left.x - right.x) <= coordinateTolerance &&
         fabsf(left.y - right.y) <= coordinateTolerance &&
         fabsf(left.z - right.z) <= coordinateTolerance;
}

void startTrajectoryPlayback(RobotControlState &state) {
  trajectoryCacheBuilding = false;
  state.trajectoryPreparing = false;
  trajectorySeekStart = {
    quantizeJointAngle(state.servo1Angle),
    quantizeJointAngle(state.servo2Angle),
    quantizeJointAngle(state.measuredStepAngle)
  };
  trajectorySeekFrames =
    (uint16_t)fmaxf(1.0f, ceilf(jointDistance(
      trajectorySeekStart, trajectoryCache[0]) /
      maximumJointStepPerLoopDeg));
  trajectoryFrame = 0;
  trajectoryPlaybackFrame = 0;
  trajectoryPlaybackFrames = (uint32_t)fmaxf(
    1.0f, ceilf(trajectoryTotalDistance / maximumJointStepPerLoopDeg));
  trajectoryPhase = 0;
  state.trajectoryActive = true;
  state.feedbackActive = true;
}

void sendPreparingAck() {
  uint8_t payload[24] = {};
  payload[0] = 1;
  payload[1] = ackDetailPreparing;
  writePacket(packetAck, payload, sizeof(payload));
}

void stopTrajectory(RobotControlState &state) {
  if (trajectoryCacheBuilding) {
    trajectoryCacheBuilding = false;
    trajectoryCacheValid = false;
  }
  state.trajectoryActive = false;
  state.trajectoryPreparing = false;
  state.feedbackActive = false;
  stopStepper();
}

void sendState(const RobotControlState &state) {
  float x = NAN;
  float y = NAN;
  float z = NAN;
  if (isfinite(state.measuredStepAngle)) {
    ForwardKinematics(state.servo1Angle,
                      state.servo2Angle - state.servo1Angle,
                      state.measuredStepAngle, &x, &y, &z);
  }

  uint8_t payload[24] = {};
  memcpy(&payload[0], &state.servo1Angle, sizeof(float));
  memcpy(&payload[4], &state.servo2Angle, sizeof(float));
  memcpy(&payload[8], &x, sizeof(float));
  memcpy(&payload[12], &y, sizeof(float));
  memcpy(&payload[16], &z, sizeof(float));
  payload[20] = (state.trajectoryActive ? 1 : 0) |
                (state.trajectoryPreparing ? 2 : 0);
  const uint16_t completedPoints = cacheBuildIndex > trajectoryPointCount
    ? trajectoryPointCount : cacheBuildIndex;
  payload[21] = (uint8_t)(
    (uint32_t)completedPoints * 100UL / trajectoryPointCount);
  writePacket(packetState, payload, sizeof(payload));
}

void sendAck(const RobotControlState &state, bool accepted,
             const char *error = nullptr,
             uint32_t trajectoryDuration = 0) {
  (void)state;
  uint8_t payload[24] = {};
  payload[0] = accepted ? 1 : 0;
  payload[1] = error != nullptr
    ? errorCodeForMessage(error) : errorNone;
  memcpy(&payload[2], &trajectoryDuration, sizeof(trajectoryDuration));
  writePacket(packetAck, payload, sizeof(payload));
}

float readPacketFloat(const CommunicationPacket &packet, size_t offset) {
  float value;
  memcpy(&value, &packet.payload[offset], sizeof(value));
  return value;
}

void processCommand(const CommunicationPacket &packet,
                    RobotControlState &state) {
  if (packet.type == commandGetState) {
    sendState(state);
    return;
  }

  if (packet.type == commandPlayTrajectory) {
    const CartesianPoint pointA = {
      readPacketFloat(packet, 0),
      readPacketFloat(packet, 4),
      readPacketFloat(packet, 8)
    };
    const CartesianPoint pointB = {
      readPacketFloat(packet, 12),
      readPacketFloat(packet, 16),
      readPacketFloat(packet, 20)
    };
    if (!isfinite(pointA.x) || !isfinite(pointA.y) ||
        !isfinite(pointA.z) || !isfinite(pointB.x) ||
        !isfinite(pointB.y) || !isfinite(pointB.z)) {
      sendAck(state, false, "trajectory point is not finite");
      return;
    }
    if (state.trajectoryActive || state.trajectoryPreparing) {
      sendAck(state, false, "trajectory is already running or preparing");
      return;
    }
    if (!state.stepperReady || !isfinite(state.measuredStepAngle)) {
      sendAck(state, false, "stepper position is not ready");
      return;
    }
    JointTarget targetA;
    if (!solvePoint(pointA, targetA)) {
      sendAck(state, false,
              "point A is unreachable or violates joint limits");
      return;
    }
    targetA.servo1 = quantizeJointAngle(targetA.servo1);
    targetA.servo2 = quantizeJointAngle(targetA.servo2);
    targetA.step = quantizeJointAngle(targetA.step);

    const JointTarget current = {
      quantizeJointAngle(state.servo1Angle),
      quantizeJointAngle(state.servo2Angle),
      quantizeJointAngle(state.measuredStepAngle)
    };
    if (!validJointTarget(current) || !validJointTarget(targetA) ||
        !validateJointSegment(current, targetA)) {
      sendAck(state, false, "joint move to point A crosses a forbidden angle");
      return;
    }

    requestedPointA = pointA;
    requestedPointB = pointB;
    if (trajectoryCacheValid && sameCartesianPoint(pointA, cachedPointA) &&
        sameCartesianPoint(pointB, cachedPointB)) {
      if (!validateJointSegment(current, trajectoryCache[0])) {
        sendAck(state, false, "joint move to point A crosses a forbidden angle");
        return;
      }
      startTrajectoryPlayback(state);
      const uint32_t durationMs =
        (uint32_t)(trajectorySeekFrames + trajectoryPlaybackFrames) *
        controlPeriodMs;
      sendAck(state, true, nullptr, durationMs);
      return;
    }

    cacheBuildIndex = 0;
    trajectoryTotalDistance = 0.0f;
    trajectoryCacheBuilding = true;
    trajectoryCacheValid = false;
    stopStepper();
    state.trajectoryActive = false;
    state.trajectoryPreparing = true;
    state.feedbackActive = false;
    sendPreparingAck();
    return;
  }

  if (packet.type == commandStop) {
    stopTrajectory(state);
    sendAck(state, true);
    return;
  }

  if (packet.type != commandSetJoints) {
    sendAck(state, false, "unknown binary command");
    return;
  }

  float requestedServo1 = readPacketFloat(packet, 0);
  float requestedServo2 = readPacketFloat(packet, 4);
  float requestedStep = readPacketFloat(packet, 8);
  if (!isfinite(requestedServo1) || !isfinite(requestedServo2) ||
      !isfinite(requestedStep)) {
    sendAck(state, false, "joint command is not finite");
    return;
  }
  if (state.trajectoryActive || state.trajectoryPreparing) {
    sendAck(state, false, "stop trajectory before setting joint angles");
    return;
  }

  requestedServo1 = quantizeJointAngle(requestedServo1);
  requestedServo2 = quantizeJointAngle(requestedServo2);
  requestedStep = quantizeJointAngle(requestedStep);

  if (requestedServo1 < SERVO1_MIN_DEG ||
      requestedServo1 > SERVO1_MAX_DEG ||
      requestedServo2 < SERVO2_MIN_DEG ||
      requestedServo2 > SERVO2_MAX_DEG ||
      requestedStep < STEPPER_MIN_DEG ||
      requestedStep > STEPPER_MAX_DEG) {
    sendAck(state, false, "angle outside joint limits");
    return;
  }
  if (!checkCondition(requestedServo1, requestedServo2)) {
    sendAck(state, false, "servo angles violate joint condition");
    return;
  }
  if (!state.stepperReady ||
      !moveToAngleFeedback(requestedStep)) {
    sendAck(state, false, "stepper feedback is not ready");
    return;
  }

  state.trajectoryActive = false;
  state.servo1Angle = requestedServo1;
  state.servo2Angle = requestedServo2;
  state.stepTargetAngle = requestedStep;
  state.feedbackActive = true;
  setStepperRate((float)STEP_RATE_HZ);
  runPwm(state.servo1Angle, state.servo2Angle, 0.0f, 0.0f);
  sendAck(state, true);
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
      memmove(receiveBuffer, receiveBuffer + headerOffset,
              receiveLength - headerOffset);
      receiveLength -= headerOffset;
    }
    if (receiveLength < 2 ||
        receiveBuffer[0] != packetMagicFirst ||
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
    memmove(receiveBuffer, receiveBuffer + sizeof(packet),
            receiveLength - sizeof(packet));
    receiveLength -= sizeof(packet);
    processCommand(packet, state);
  }
}

void buildTrajectoryCacheBatch(RobotControlState &state) {
  uint8_t calculatedPoints = 0;
  while (cacheBuildIndex <= trajectoryPointCount &&
         calculatedPoints < trajectoryPointsPerLoop) {
    const float t = (float)cacheBuildIndex / trajectoryPointCount;
    const CartesianPoint point = interpolate(requestedPointA, requestedPointB, t);
    JointTarget target;
    if (!solvePoint(point, target)) {
      trajectoryCacheBuilding = false;
      trajectoryCacheValid = false;
      state.trajectoryPreparing = false;
      robotCommsSendError("trajectory contains an unreachable or forbidden point");
      return;
    }

    target.servo1 = quantizeJointAngle(target.servo1);
    target.servo2 = quantizeJointAngle(target.servo2);
    target.step = quantizeJointAngle(target.step);
    if (!validJointTarget(target)) {
      trajectoryCacheBuilding = false;
      trajectoryCacheValid = false;
      state.trajectoryPreparing = false;
      robotCommsSendError("trajectory point violates joint limits");
      return;
    }

    trajectoryCache[cacheBuildIndex] = target;
    if (cacheBuildIndex == 0) {
      trajectoryDistance[cacheBuildIndex] = 0.0f;
    } else {
      trajectoryTotalDistance += jointDistance(
        trajectoryCache[cacheBuildIndex - 1], target);
      trajectoryDistance[cacheBuildIndex] = trajectoryTotalDistance;
    }
    ++cacheBuildIndex;
    ++calculatedPoints;
  }

  if (cacheBuildIndex > trajectoryPointCount) {
    cachedPointA = requestedPointA;
    cachedPointB = requestedPointB;
    trajectoryCacheValid = true;
    trajectoryCacheBuilding = false;
    const JointTarget current = {
      quantizeJointAngle(state.servo1Angle),
      quantizeJointAngle(state.servo2Angle),
      quantizeJointAngle(state.measuredStepAngle)
    };
    if (!validJointTarget(current) ||
        !validateJointSegment(current, trajectoryCache[0])) {
      state.trajectoryPreparing = false;
      robotCommsSendError("joint move to point A crosses a forbidden angle");
      return;
    }
    startTrajectoryPlayback(state);
  }
}

JointTarget cachedTargetAtDistance(float distance) {
  if (distance <= 0.0f) {
    return trajectoryCache[0];
  }
  if (distance >= trajectoryTotalDistance) {
    return trajectoryCache[trajectoryPointCount];
  }

  uint16_t lower = 0;
  uint16_t upper = trajectoryPointCount;
  while ((uint16_t)(lower + 1) < upper) {
    const uint16_t middle = lower + (upper - lower) / 2;
    if (trajectoryDistance[middle] <= distance) {
      lower = middle;
    } else {
      upper = middle;
    }
  }

  const float startDistance = trajectoryDistance[lower];
  const float segmentDistance = trajectoryDistance[upper] - startDistance;
  const float segmentProgress = segmentDistance > 0.0f
    ? (distance - startDistance) / segmentDistance : 1.0f;
  return interpolate(
    trajectoryCache[lower], trajectoryCache[upper], segmentProgress);
}
}

void robotCommsBegin() {
  Serial.begin(115200);
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

void robotCommsUpdateControl(RobotControlState &state, float measuredAngleDeg) {
  if (!isfinite(measuredAngleDeg) || measuredAngleDeg < 0.0f ||
      measuredAngleDeg >= 360.0f) {
    if (state.feedbackActive) {
      updateStepperFeedback(measuredAngleDeg);
      state.feedbackActive = false;
      state.trajectoryActive = false;
      robotCommsSendError("AS5600 feedback lost or outside step limits");
    }
    if (state.trajectoryPreparing) {
      trajectoryCacheBuilding = false;
      trajectoryCacheValid = false;
      state.trajectoryPreparing = false;
      robotCommsSendError("AS5600 feedback lost while preparing trajectory");
    }
    state.measuredStepAngle = NAN;
    return;
  }
  state.measuredStepAngle = measuredAngleDeg;
  if (!state.stepperReady) {
    return;
  }

  if (trajectoryCacheBuilding) {
    buildTrajectoryCacheBatch(state);
  }

  if (state.trajectoryActive) {
    JointTarget target;
    bool targetReady = false;
    if (trajectoryPhase == 0) {
      if (trajectoryFrame < trajectorySeekFrames) {
        ++trajectoryFrame;
      }
      target = interpolate(
        trajectorySeekStart, trajectoryCache[0],
        (float)trajectoryFrame / trajectorySeekFrames);
      targetReady = true;
      if (trajectoryFrame >= trajectorySeekFrames) {
        trajectoryFrame = 0;
        trajectoryPhase = 1;
      }
    } else if (trajectoryPhase == 1) {
      if (trajectoryPlaybackFrame < trajectoryPlaybackFrames) {
        ++trajectoryPlaybackFrame;
      }
      const float pathDistance = trajectoryTotalDistance *
        (float)trajectoryPlaybackFrame / trajectoryPlaybackFrames;
      target = cachedTargetAtDistance(pathDistance);
      targetReady = true;
      if (trajectoryPlaybackFrame >= trajectoryPlaybackFrames) {
        trajectoryPhase = 2;
      }
    }

    if (targetReady) {
      if (!validJointTarget(target) ||
          !moveToAngleFeedback(target.step)) {
        stopTrajectory(state);
        robotCommsSendError("joint target became invalid; motion stopped");
        return;
      }

      state.servo1Angle = target.servo1;
      state.servo2Angle = target.servo2;
      state.stepTargetAngle = target.step;
      setStepperRate((float)STEP_RATE_HZ);
      runPwm(state.servo1Angle, state.servo2Angle, 0.0f, 0.0f);
    }
  }

  if (state.feedbackActive &&
      !updateStepperFeedback(state.measuredStepAngle)) {
    state.feedbackActive = false;
    state.trajectoryActive = false;
    robotCommsSendError("AS5600 feedback lost or outside step limits");
  }
  if (state.trajectoryActive && trajectoryPhase == 2 &&
      fabsf(state.stepTargetAngle - state.measuredStepAngle) <=
        STEPPER_FEEDBACK_DEADBAND_DEG) {
    state.trajectoryActive = false;
    state.feedbackActive = false;
    setStepperRate(0.0f);
  }
}

void robotCommsSendError(const char *message) {
  const uint8_t errorCode = errorCodeForMessage(message);
  uint8_t payload[24] = {};
  payload[0] = errorCode;
  writePacket(packetError, payload, sizeof(payload));
}
