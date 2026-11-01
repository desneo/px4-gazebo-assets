#pragma once

#include <optional>
#include <string>

#include "two_axis/kinematics.hh"

namespace two_axis
{
// 目标相对固定名义重心的NED方位角/仰角变化率，不是角速度向量分量。
struct TargetLosSample
{
  bool valid{false};
  std::string reason{"first_sample"};
  std::optional<double> sim_stamp;
  std::optional<Vec3> relative_position_ned_m;
  std::optional<Vec3> relative_velocity_ned_mps;
  std::optional<double> yaw_rate_dps;
  std::optional<double> pitch_rate_dps;
  // 惯性系最小LOS转动向量 r×v/|r|²，分量按N/E/D表达；独立于方位角奇异。
  bool omega_valid{false};
  std::string omega_reason{"first_sample"};
  std::optional<Vec3> omega_ned_dps;
};

// psi=atan2(E,N)，theta=atan2(-D,hypot(N,E))；右转/抬升为正。
// 同时计算NED三维LOS角速度向量，弧度转换为度后不作机械限幅。
// 方位/仰角与向量分别报告有效性；竖直视线只使前者退化。
TargetLosSample TargetLosRates(const Vec3 &relative_position_ned_m,
                              const Vec3 &relative_velocity_ned_mps);

// 将模型坐标中的固定名义重心偏移随姿态旋转，而非使用运动关节瞬时重心。
Vec3 NominalCgWorld(const Vec3 &model_position_world,
                   const Mat3 &model_rotation_world,
                   const Vec3 &nominal_cg_model);

// 双方位置必须来自同一Gazebo物理状态，差分使用仿真时间。
class TargetLosEstimator
{
public:
  TargetLosSample Update(double sim_stamp, const std::string &target_identity,
                         const Vec3 &observer_ned_m, const Vec3 &target_ned_m);
  TargetLosSample Reset(const std::string &reason);

private:
  bool initialized_{false};
  double previous_stamp_{0};
  std::string previous_identity_;
  Vec3 previous_observer_;
  Vec3 previous_target_;
};
}  // namespace two_axis
