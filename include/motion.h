#pragma once
// Lập kế hoạch chuyển động thuần toán học (không phụ thuộc Arduino) để test được trên PC.
#include <stdint.h>
#include <math.h>

struct Joint {
  float s1;  // servo 1 (độ)
  float s2;  // servo 2 (độ, tuyệt đối)
  float st;  // stepper (độ, trục ra)
};

struct SpeedLimits {  // độ/giây
  float s1;
  float s2;
  float st;
};

inline Joint lerpJoint(const Joint &a, const Joint &b, float f) {
  return { a.s1 + (b.s1 - a.s1) * f,
           a.s2 + (b.s2 - a.s2) * f,
           a.st + (b.st - a.st) * f };
}

// Thời gian tối thiểu để đi từ a đến b khi mọi khớp chạy đồng bộ và khớp
// "chậm nhất" chạy hết tốc độ.
float minMoveTime(const Joint &a, const Joint &b, const SpeedLimits &lim);

// Hệ số tốc độ s(t) in [0,1] dạng hình thang: tăng tốc đều -> chạy đều -> giảm tốc đều.
// Vị trí x(t) tính theo "thời gian lý tưởng" (đơn vị giây ở tốc độ tối đa), 0..L.
struct Profile {
  float L = 0, vp = 1, t1 = 0, tc = 0, T = 0, a = 1;
  void init(float length, float accelTimeS);
  float position(float t) const;
};

// Quỹ đạo gồm các điểm khớp đã tính IK, được hậu xử lý thành đường đi theo thời gian.
class Trajectory {
 public:
  static constexpr uint16_t kMaxInputPoints = 513;
  static constexpr uint16_t kMaxPoints = 800;
  static constexpr float kMaxJointDeltaDeg = 0.45f;

  void clear();
  bool append(const Joint &j);
  // Tính thời gian tích lũy theo giới hạn tốc độ từng khớp + profile tăng/giảm tốc.
  bool finalize(const SpeedLimits &lim, float accelTimeS);

  uint16_t size() const { return count_; }
  const Joint &first() const { return pts_[0]; }
  const Joint &last() const { return pts_[count_ - 1]; }
  float duration() const { return profile_.T; }
  // Mẫu tại thời điểm t (giây) kể từ lúc bắt đầu; reverse = chạy ngược B -> A.
  Joint sample(float t, bool reverse) const;

 private:
  Joint pts_[kMaxPoints];
  float cumT_[kMaxPoints];
  uint16_t count_ = 0;
  Profile profile_;
};

// Bám mục tiêu thay đổi liên tục (kéo thanh trượt) với giới hạn tốc độ + gia tốc.
struct AxisTracker {
  float pos = 0, vel = 0;
  void reset(float p) { pos = p; vel = 0; }
  void update(float target, float vmax, float amax, float dt);
  bool settled(float target) const { return pos == target && vel == 0.0f; }
};
