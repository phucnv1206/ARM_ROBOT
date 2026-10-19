#include <cmath>
#include "config.h"

namespace {
constexpr float degreesToRadians = 0.017453292519943295f;
constexpr float radiansToDegrees = 57.29577951308232f;

inline float custom_clamp(float val, float minVal, float maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}
}

void ForwardKinematics(float theta1, float theta2, float theta3, float *x, float *y, float *z) {
    const float theta1Rad = theta1 * degreesToRadians;
    const float theta2Rad = theta2 * degreesToRadians;
    const float theta3Rad = theta3 * degreesToRadians;

    float alpha2Rad = (180.0f * degreesToRadians) - (theta1Rad + theta2Rad);

    float r = L1 * cos(theta1Rad) + L2 * cos(alpha2Rad) - D + D2;

    *z = H + L1 * sin(theta1Rad) - L2 * sin(alpha2Rad) - D1;
    *x = r * cos(theta3Rad);
    *y = r * sin(theta3Rad);
}

bool InvertKinematics(float x, float y, float z, float *theta1, float *theta2, float *theta3) {
    float rp = sqrt(x * x + y * y) + D - D2;
    float zp = z - H + D1;

    float costheta2 = (L1 * L1 + L2 * L2 - (rp * rp + zp * zp)) / (2.0f * L1 * L2);
    if (!isfinite(costheta2) || costheta2 < -1.0001f || costheta2 > 1.0001f) {
        return false;
    }
    costheta2 = custom_clamp(costheta2, -1.0f, 1.0f);

    *theta3 = atan2(y, x) * radiansToDegrees;
    if (*theta3 < 0.0f) {
        *theta3 += 360.0f;
    }

    const float theta2Rad = atan2(sqrt(1.0f - costheta2 * costheta2), costheta2);

    const float theta1Rad = atan2(zp, rp) + atan2(L2 * sin(theta2Rad), L1 - L2 * cos(theta2Rad));

    *theta2 = theta2Rad * radiansToDegrees;
    *theta1 = theta1Rad * radiansToDegrees;

    return true;
}

bool checkCondition(float angle1, float angle2){
  if(angle1 >= SERVO1_MIN_DEG && angle1 < 30.0f){
    return angle2 >= SERVO2_MIN_DEG && angle2 <= angle1 + 150.0f;
  }
  if(angle1 >= 30.0f && angle1 <= 60.0f){
    return angle2 >= SERVO2_MIN_DEG && angle2 <= SERVO2_MAX_DEG;
  }
  if(angle1 > 60.0f && angle1 <= SERVO1_MAX_DEG){
    return angle2 >= angle1 + 30.0f && angle2 <= SERVO2_MAX_DEG;
  }
  return false;
}