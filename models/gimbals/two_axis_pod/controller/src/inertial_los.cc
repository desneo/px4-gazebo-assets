#include "two_axis/inertial_los.hh"

#include <algorithm>
#include <cmath>

namespace two_axis
{
namespace
{
constexpr double kRad2Deg = 180.0 / M_PI;
// 测量连续性保护，不是飞行包线或LOS报文限幅。
constexpr double kMaxSampleGap = 0.5;
constexpr double kMaxPositionSpeed = 1000.0;
constexpr double kMinJumpDistance = 1.0;

bool Finite(const Vec3 &v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

Vec3 Difference(const Vec3 &a, const Vec3 &b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

double Length(const Vec3 &v)
{
  return std::hypot(std::hypot(v.x, v.y), v.z);
}
}  // namespace

TargetLosSample TargetLosRates(const Vec3 &r, const Vec3 &v)
{
  TargetLosSample result;
  if (!Finite(r) || !Finite(v))
  {
    result.reason = "nonfinite_sample";
    result.omega_reason = result.reason;
    return result;
  }
  result.relative_position_ned_m = r;
  result.relative_velocity_ned_mps = v;
  const double horizontal = std::hypot(r.x, r.y);
  const double range = std::hypot(horizontal, r.z);
  if (!std::isfinite(range) || range < 1e-6)
  {
    result.reason = "zero_range";
    result.omega_reason = result.reason;
    return result;
  }
  // 先分别归一化r和v，再叉乘，避免r×v或|r|²在大坐标下溢出。
  // 扩展精度中间值也避免极小有效距离时v/range过早溢出。
  const long double unit[3] = {
    static_cast<long double>(r.x) / range,
    static_cast<long double>(r.y) / range,
    static_cast<long double>(r.z) / range};
  const long double velocityOverRange[3] = {
    static_cast<long double>(v.x) / range,
    static_cast<long double>(v.y) / range,
    static_cast<long double>(v.z) / range};
  const Vec3 omega{
    static_cast<double>((unit[1] * velocityOverRange[2] -
                         unit[2] * velocityOverRange[1]) * kRad2Deg),
    static_cast<double>((unit[2] * velocityOverRange[0] -
                         unit[0] * velocityOverRange[2]) * kRad2Deg),
    static_cast<double>((unit[0] * velocityOverRange[1] -
                         unit[1] * velocityOverRange[0]) * kRad2Deg)};
  if (Finite(omega))
  {
    result.omega_valid = true;
    result.omega_reason = "ok";
    result.omega_ned_dps = omega;
  }
  else
    result.omega_reason = "nonfinite_rate";

  if (horizontal < 1e-6 || horizontal / range < 1e-6)
  {
    result.reason = "vertical_singularity";
    return result;
  }
  // 归一化表达避免远距离坐标平方导致溢出；不差分包裹后的方位角。
  const double n = r.x / horizontal, e = r.y / horizontal;
  const double horizontalRate = n * v.x + e * v.y;
  const double yaw = (n * v.y - e * v.x) / horizontal;
  const double pitch = ((r.z / range) * horizontalRate -
                        (horizontal / range) * v.z) / range;
  if (!std::isfinite(yaw * kRad2Deg) || !std::isfinite(pitch * kRad2Deg))
  {
    result.reason = "nonfinite_rate";
    return result;
  }
  result.valid = true;
  result.reason = "ok";
  result.yaw_rate_dps = yaw * kRad2Deg;
  result.pitch_rate_dps = pitch * kRad2Deg;
  return result;
}

Vec3 NominalCgWorld(const Vec3 &position, const Mat3 &rotation,
                   const Vec3 &offset)
{
  const auto rotated = MatVec(rotation, offset);
  return {position.x + rotated.x, position.y + rotated.y,
          position.z + rotated.z};
}

TargetLosSample TargetLosEstimator::Reset(const std::string &reason)
{
  initialized_ = false;
  previous_identity_.clear();
  TargetLosSample result;
  result.reason = reason;
  result.omega_reason = reason;
  return result;
}

TargetLosSample TargetLosEstimator::Update(
  double stamp, const std::string &identity, const Vec3 &observer,
  const Vec3 &target)
{
  if (!std::isfinite(stamp) || !Finite(observer) || !Finite(target))
    return Reset("nonfinite_sample");
  if (identity.empty())
    return Reset("target_unbound");

  const Vec3 relative = Difference(target, observer);
  if (!Finite(relative))
    return Reset("nonfinite_sample");
  TargetLosSample result;
  result.sim_stamp = stamp;
  result.relative_position_ned_m = relative;
  const double dt = stamp - previous_stamp_;
  if (!initialized_)
    result.reason = "first_sample";
  else if (identity != previous_identity_)
    result.reason = "target_changed";
  else if (dt < 0)
    result.reason = "time_reset";
  else if (dt <= 1e-9)
  {
    // 重复时刻不覆盖差分基准；暂停/恢复由调用者显式Reset。
    result.reason = "time_paused";
    result.omega_reason = result.reason;
    return result;
  }
  else if (dt > kMaxSampleGap)
    result.reason = "time_gap";
  else if (Length(Difference(observer, previous_observer_)) >
             std::max(kMinJumpDistance, kMaxPositionSpeed * dt) ||
           Length(Difference(target, previous_target_)) >
             std::max(kMinJumpDistance, kMaxPositionSpeed * dt))
    result.reason = "position_jump";
  else
  {
    const auto previousRelative = Difference(previous_target_, previous_observer_);
    const auto delta = Difference(relative, previousRelative);
    result = TargetLosRates(relative, {delta.x / dt, delta.y / dt, delta.z / dt});
    result.sim_stamp = stamp;
  }
  initialized_ = true;
  previous_stamp_ = stamp;
  previous_identity_ = identity;
  previous_observer_ = observer;
  previous_target_ = target;
  // 没有连续速度样本时，两套测量均无效并报告同一连续性原因。
  if (!result.relative_velocity_ned_mps)
    result.omega_reason = result.reason;
  return result;
}
}  // namespace two_axis
