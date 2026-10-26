#include "two_axis/inertial_los.hh"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void Check(bool condition, const char *message)
{
  if (!condition)
    throw std::runtime_error(message);
}

void Near(double actual, double expected, double tolerance, const char *message)
{
  Check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
}  // namespace

int main()
{
  using namespace two_axis;
  constexpr double degrees = 180.0 / M_PI;

  const auto radial = TargetLosRates({100, 0, -20}, {10, 0, -2});
  Check(radial.valid, "径向运动LOS有效");
  Near(*radial.yaw_rate_dps, 0, 1e-12, "径向运动方位不变");
  Near(*radial.pitch_rate_dps, 0, 1e-12, "径向运动仰角不变");
  Check(radial.omega_valid && radial.omega_ned_dps &&
        radial.omega_reason == "ok", "径向运动的零LOS向量有效");
  Near(radial.omega_ned_dps->x, 0, 1e-12, "径向运动omega_N为零");
  Near(radial.omega_ned_dps->y, 0, 1e-12, "径向运动omega_E为零");
  Near(radial.omega_ned_dps->z, 0, 1e-12, "径向运动omega_D为零");
  const auto east = TargetLosRates({100, 0, 0}, {0, 10, 0});
  Near(*east.yaw_rate_dps, .1 * degrees, 1e-12, "北侧目标向东为右转正");
  Near(east.omega_ned_dps->z, .1 * degrees, 1e-12,
       "N视线向E转动时omega_D为正");
  const auto northComponent = TargetLosRates({0, 100, 0}, {0, 0, 10});
  Near(northComponent.omega_ned_dps->x, .1 * degrees, 1e-12,
       "E视线向D转动时omega_N为正");
  const auto eastComponent = TargetLosRates({0, 0, 100}, {10, 0, 0});
  Near(eastComponent.omega_ned_dps->y, .1 * degrees, 1e-12,
       "D视线向N转动时omega_E为正");
  const auto west = TargetLosRates({100, 0, 0}, {0, -10, 0});
  Near(*west.yaw_rate_dps, -.1 * degrees, 1e-12, "北侧目标向西为左转负");
  Near(west.omega_ned_dps->z, -.1 * degrees, 1e-12,
       "反向转动的omega_D为负");
  const auto up = TargetLosRates({100, 0, 0}, {0, 0, -10});
  Near(*up.pitch_rate_dps, .1 * degrees, 1e-12, "NED向上运动仰角为正");
  Near(up.omega_ned_dps->y, .1 * degrees, 1e-12,
       "N视线抬升时omega_E为正");
  const auto down = TargetLosRates({100, 0, 0}, {0, 0, 10});
  Near(*down.pitch_rate_dps, -.1 * degrees, 1e-12, "NED向下运动仰角为负");

  const auto highRate = TargetLosRates({10, 0, 0}, {0, 100, -100});
  Check(highRate.valid && *highRate.yaw_rate_dps > 20 &&
        *highRate.pitch_rate_dps > 20, "内部惯性测量不受机械20度限幅");
  Check(highRate.omega_valid && highRate.omega_ned_dps->y > 20 &&
        highRate.omega_ned_dps->z > 20, "三维LOS向量真实值超过20度不截断");
  Check(!TargetLosRates({0, 0, 0}, {0, 0, 0}).valid,
        "零距离不能伪造有效LOS");
  const auto vertical = TargetLosRates({0, 0, -10}, {1, 0, 0});
  Check(!vertical.valid && vertical.reason == "vertical_singularity" &&
        !vertical.yaw_rate_dps && !vertical.pitch_rate_dps,
        "垂直视线的方位奇异必须无效且两个速率为空");
  Check(vertical.omega_valid && vertical.omega_reason == "ok" &&
        vertical.omega_ned_dps, "垂直视线的三维向量仍可有效测量");
  Near(vertical.omega_ned_dps->y, -.1 * degrees, 1e-12,
       "向上竖直视线向N转动的omega_E为负");
  const auto stationaryVertical = TargetLosRates({0, 0, 10}, {0, 0, 0});
  Check(!stationaryVertical.valid && stationaryVertical.omega_valid,
        "静止竖直LOS向量为有效零值而方位角无效");
  Near(stationaryVertical.omega_ned_dps->x, 0, 0, "竖直静止向量N零");
  Near(stationaryVertical.omega_ned_dps->y, 0, 0, "竖直静止向量E零");
  Near(stationaryVertical.omega_ned_dps->z, 0, 0, "竖直静止向量D零");

  // 一般姿态用未经归一化的叉乘公式独立核对，向量应垂直于LOS。
  const Vec3 generalR{3, 4, -12}, generalV{2, -1, 5};
  const auto general = TargetLosRates(generalR, generalV);
  const Vec3 expected{8.0 / 169 * degrees, -39.0 / 169 * degrees,
                      -11.0 / 169 * degrees};
  Check(general.valid && general.omega_valid, "一般姿态两种LOS表示均有效");
  Near(general.omega_ned_dps->x, expected.x, 1e-12, "一般姿态叉乘公式N分量");
  Near(general.omega_ned_dps->y, expected.y, 1e-12, "一般姿态叉乘公式E分量");
  Near(general.omega_ned_dps->z, expected.z, 1e-12, "一般姿态叉乘公式D分量");
  Near(general.omega_ned_dps->x * generalR.x +
       general.omega_ned_dps->y * generalR.y +
       general.omega_ned_dps->z * generalR.z,
       0, 1e-12, "最小LOS角速度向量与LOS垂直");
  const auto rotate = MatMul(Rz(.7), MatMul(Ry(-.4), Rx(1.1)));
  const auto rotated = TargetLosRates(MatVec(rotate, generalR), MatVec(rotate, generalV));
  const auto rotatedExpected = MatVec(rotate, expected);
  Near(rotated.omega_ned_dps->x, rotatedExpected.x, 1e-12, "转动坐标基底时向量N分量随之转动");
  Near(rotated.omega_ned_dps->y, rotatedExpected.y, 1e-12, "转动坐标基底时向量E分量随之转动");
  Near(rotated.omega_ned_dps->z, rotatedExpected.z, 1e-12, "转动坐标基底时向量D分量随之转动");
  const auto large = TargetLosRates({1e200, 0, 0}, {0, 1e200, 0});
  Check(large.omega_valid, "大坐标归一化后仍有有效向量");
  Near(large.omega_ned_dps->z, degrees, 1e-12, "避免叉乘和距离平方溢出");

  const auto badVector = TargetLosRates({1, 0, 0},
    {0, std::numeric_limits<double>::max(), 0});
  Check(!badVector.omega_valid && !badVector.omega_ned_dps &&
        badVector.omega_reason == "nonfinite_rate", "向量单位转换溢出时无效且为空");
  const auto badInput = TargetLosRates({100, 0, 0},
    {std::numeric_limits<double>::quiet_NaN(), 0, 0});
  Check(!badInput.omega_valid && !badInput.omega_ned_dps &&
        badInput.omega_reason == "nonfinite_sample", "非有限输入不能伪造有效向量");
  const auto zeroRange = TargetLosRates({0, 0, 0}, {0, 0, 0});
  Check(!zeroRange.omega_valid && !zeroRange.omega_ned_dps &&
        zeroRange.omega_reason == "zero_range", "零距离向量无效且为空");
  Check(!TargetLosRates({1e-9, 0, 1000}, {0, 1, 0}).valid,
        "近垂直视线具有明确退化保护");
  Check(!TargetLosRates({100, 0, 0},
        {std::numeric_limits<double>::quiet_NaN(), 0, 0}).valid,
        "拒绝非有限速度");
  Check(!TargetLosRates({1, 0, 0},
        {0, std::numeric_limits<double>::max(), 0}).valid,
        "拒绝角度单位转换溢出");

  TargetLosEstimator targetMotion;
  Check(!targetMotion.Update(1, "target:1", {0, 0, 0}, {100, 0, 0}).valid,
        "首样本不能得到速度");
  TargetLosEstimator firstVector;
  const auto first = firstVector.Update(1, "target:1", {0, 0, 0}, {100, 0, 0});
  Check(!first.omega_valid && !first.omega_ned_dps &&
        first.omega_reason == "first_sample", "首样本不能输出向量");
  const auto moving = targetMotion.Update(1.01, "target:1", {0, 0, 0}, {100, .1, 0});
  Check(moving.valid && moving.sim_stamp == 1.01,
        "目标位置差分产生同一仿真时刻的LOS测量");
  Near(moving.relative_velocity_ned_mps->y, 10, 1e-10,
       "差分分母采用仿真时间");
  Near(*moving.yaw_rate_dps, degrees * 1000 / 10000.01, 1e-10,
       "目标运动的方位角导数正确");
  TargetLosEstimator observerMotion;
  observerMotion.Update(1, "target:1", {0, 0, 0}, {100, 0, 0});
  const auto observer = observerMotion.Update(1.01, "target:1", {0, .1, 0}, {100, 0, 0});
  Near(*observer.yaw_rate_dps, -*moving.yaw_rate_dps, 1e-10,
       "观察点运动使用目标减观察点相对速度");
  TargetLosEstimator commonMotion;
  commonMotion.Update(1, "target:1", {10, 20, -30}, {110, 20, -30});
  const auto common = commonMotion.Update(1.01, "target:1", {10.1, 20.2, -30.1},
                                          {110.1, 20.2, -30.1});
  Near(*common.yaw_rate_dps, 0, 1e-9, "双方相同平移不产生LOS转动");
  Near(*common.pitch_rate_dps, 0, 1e-9, "双方相同爬升不产生LOS转动");

  TargetLosEstimator wrapped;
  constexpr double step = .001;
  const double a = M_PI - step / 2, b = M_PI + step / 2;
  wrapped.Update(0, "target:1", {0, 0, 0}, {100 * std::cos(a), 100 * std::sin(a), 0});
  const auto crossing = wrapped.Update(.01, "target:1", {0, 0, 0},
    {100 * std::cos(b), 100 * std::sin(b), 0});
  Near(*crossing.yaw_rate_dps, std::sin(step) / .01 * degrees, 1e-8,
       "跨正负180度无角度差分跳变");

  // 固定名义重心在世界不动，机体绕其旋转，模型原点必须对应移动。
  TargetLosEstimator rotating;
  const Vec3 offset{.3, -.1, .2}, fixedCg{1, 2, 3};
  for (int index = 0; index < 50; ++index)
  {
    const double t = index * .01;
    const auto rotation = MatMul(Rz(t), MatMul(Ry(.3 * t), Rx(-.7 * t)));
    const auto arm = MatVec(rotation, offset);
    const auto cg = NominalCgWorld(
      {fixedCg.x - arm.x, fixedCg.y - arm.y, fixedCg.z - arm.z}, rotation, offset);
    const auto result = rotating.Update(t, "target:1", cg, {100, 50, -20});
    if (index)
    {
      Check(result.valid, "固定名义重心的刚体旋转应有有效测量");
      Near(*result.yaw_rate_dps, 0, 1e-9, "机体原地转动不产生目标惯性方位速度");
      Near(*result.pitch_rate_dps, 0, 1e-9, "机体原地转动不产生目标惯性仰角速度");
    }
  }

  TargetLosEstimator invalid;
  invalid.Update(1, "target:1", {0, 0, 0}, {100, 0, 0});
  const auto changed = invalid.Update(1.01, "target:2", {0, 0, 0}, {100, 10, 0});
  Check(!changed.valid && changed.reason == "target_changed", "换目标须重新建立差分基准");
  Check(!changed.omega_valid && !changed.omega_ned_dps &&
        changed.omega_reason == "target_changed", "换目标同时清除旧向量");
  const auto paused = invalid.Update(1.01, "target:2", {0, 0, 0}, {100, 10, 0});
  Check(!paused.valid && paused.reason == "time_paused", "同一时刻不计算速度");
  Check(!paused.omega_valid && !paused.omega_ned_dps &&
        paused.omega_reason == "time_paused", "重复时刻不能输出向量");
  const auto reset = invalid.Update(.5, "target:2", {0, 0, 0}, {100, 10, 0});
  Check(!reset.valid && reset.reason == "time_reset", "时间回退不输出旧测量");
  Check(!reset.omega_valid && !reset.omega_ned_dps &&
        reset.omega_reason == "time_reset", "回退不能输出旧向量");
  Check(invalid.Update(.51, "target:2", {0, 0, 0}, {100, 10, 0}).valid,
        "回退后连续新样本恢复");
  const auto gap = invalid.Update(2, "target:2", {0, 0, 0}, {100, 10, 0});
  Check(!gap.valid && gap.reason == "time_gap", "长时间缺采样不连线计算速度");
  Check(!gap.omega_valid && !gap.omega_ned_dps &&
        gap.omega_reason == "time_gap", "时间间隔过大时向量无效");
  const auto jump = invalid.Update(2.01, "target:2", {0, 0, 0}, {10000, 10, 0});
  Check(!jump.valid && jump.reason == "position_jump", "实体瞬移不能伪造有效角速度");
  Check(!jump.omega_valid && !jump.omega_ned_dps &&
        jump.omega_reason == "position_jump", "瞬移不能伪造有效向量");
  Check(invalid.Update(2.02, "target:2", {0, 0, 0}, {10000, 10, 0}).valid,
        "瞬移后下一连续样本恢复");
  Check(!invalid.Reset("target_missing").yaw_rate_dps, "删除目标后速率为空");
  const auto missing = invalid.Reset("target_missing");
  Check(!missing.omega_valid && !missing.omega_ned_dps &&
        missing.omega_reason == "target_missing", "删除目标后向量为空");
  Check(!invalid.Update(2.03, "target:2", {0, 0, 0}, {10000, 10, 0}).valid,
        "删除后重新获取需要首样本");

  std::cout << "目标惯性LOS数学与连续性保护测试通过\n";
}
