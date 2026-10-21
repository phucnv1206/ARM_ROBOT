#include "motion.h"

float minMoveTime(const Joint &a, const Joint &b, const SpeedLimits &lim) {
  return fmaxf(fabsf(b.s1 - a.s1) / lim.s1,
               fmaxf(fabsf(b.s2 - a.s2) / lim.s2,
                     fabsf(b.st - a.st) / lim.st));
}

void Profile::init(float length, float accelTimeS) {
  if (!(length > 1e-6f)) {
    L = 0; vp = 1; t1 = 0; tc = 0; T = 0; a = 1;
    return;
  }
  L = length;
  accelTimeS = fmaxf(accelTimeS, 1e-3f);
  a = 1.0f / accelTimeS;
  vp = fminf(1.0f, sqrtf(L / accelTimeS));  // < 1 khi đường quá ngắn (tam giác)
  t1 = vp * accelTimeS;
  const float ramp = 0.5f * vp * t1;        // quãng đường mỗi đoạn tăng/giảm tốc
  tc = fmaxf(0.0f, (L - 2.0f * ramp) / vp);
  T = 2.0f * t1 + tc;
}

float Profile::position(float t) const {
  if (t <= 0.0f) return 0.0f;
  if (t >= T) return L;
  if (t < t1) return 0.5f * a * t * t;
  if (t < t1 + tc) return 0.5f * vp * t1 + vp * (t - t1);
  const float r = T - t;
  return L - 0.5f * a * r * r;
}

void Trajectory::clear() {
  count_ = 0;
  profile_.init(0.0f, 1.0f);
}

bool Trajectory::append(const Joint &j) {
  if (count_ == 0) {
    pts_[count_++] = j;
    return true;
  }

  const Joint &previous = pts_[count_ - 1];
  const float largestDelta = fmaxf(
      fabsf(j.s1 - previous.s1),
      fmaxf(fabsf(j.s2 - previous.s2), fabsf(j.st - previous.st)));
  if (!isfinite(largestDelta)) return false;

  const float requiredSegments =
      fmaxf(1.0f, ceilf(largestDelta / kMaxJointDeltaDeg));
  if (requiredSegments > (float)(kMaxPoints - count_)) return false;

  const uint16_t segments = (uint16_t)requiredSegments;
  for (uint16_t segment = 1; segment <= segments; ++segment) {
    const float fraction = (float)segment / (float)segments;
    pts_[count_++] = lerpJoint(previous, j, fraction);
  }
  return true;
}

bool Trajectory::finalize(const SpeedLimits &lim, float accelTimeS) {
  if (count_ < 2) return false;
  cumT_[0] = 0.0f;
  for (uint16_t k = 1; k < count_; ++k) {
    cumT_[k] = cumT_[k - 1] + minMoveTime(pts_[k - 1], pts_[k], lim);
  }
  profile_.init(cumT_[count_ - 1], accelTimeS);
  return true;
}

Joint Trajectory::sample(float t, bool reverse) const {
  float x = profile_.position(t);
  if (reverse) x = profile_.L - x;

  uint16_t lo = 0, hi = count_ - 1;
  while (hi - lo > 1) {
    const uint16_t mid = lo + (hi - lo) / 2;
    if (cumT_[mid] <= x) lo = mid; else hi = mid;
  }
  const float dt = cumT_[hi] - cumT_[lo];
  float f = dt > 1e-9f ? (x - cumT_[lo]) / dt : 1.0f;
  f = fminf(1.0f, fmaxf(0.0f, f));
  return lerpJoint(pts_[lo], pts_[hi], f);
}

void AxisTracker::update(float target, float vmax, float amax, float dt) {
  const float err = target - pos;
  const float absErr = fabsf(err);
  const float maxDv = amax * dt;
  if (absErr < 1e-4f && fabsf(vel) <= maxDv) {
    pos = target;
    vel = 0.0f;
    return;
  }
  float wanted = fminf(vmax, sqrtf(2.0f * amax * absErr));  // vận tốc còn kịp phanh
  if (err < 0.0f) wanted = -wanted;
  float dv = wanted - vel;
  if (dv > maxDv) dv = maxDv;
  else if (dv < -maxDv) dv = -maxDv;
  vel += dv;
  float next = pos + vel * dt;
  if ((target - pos) * (target - next) < 0.0f) {  // không vượt quá đích
    next = target;
    vel = 0.0f;
  }
  pos = next;
}
