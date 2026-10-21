#include "two_axis/kinematics.hh"
#include <cmath>
#include <algorithm>
namespace two_axis
{
Mat3 Rotation(double y, double p)
{
  return MatMul(Rz(y), Ry(-p));
}
Vec3 Los(double y, double p)
{
  return MatVec(Rotation(y, p), {0, 0, 1});
}
bool Singular(double p)
{
  return std::abs(std::sin(p)) < 0.05;
}
std::array<double, 2> ImageRates(double p, double right, double down)
{
  // 俯仰正轴为随动 -Y；图像右(+Y)、下(-X)：J=diag(-sin(p),+1)。
  // 阻尼最小二乘避免朝天时 yaw 失去视线控制能力后速度发散。
  const double s = -std::sin(p), damping = 0.05;
  return {s*right / (s * s + damping * damping), down};
}
Vec3 MountAngularVelocity(const Mat3 &previous, const Mat3 &current, double dt)
{
  if (!std::isfinite(dt) || dt <= 0 || dt > .1)
    return {};
  const auto delta = MatMul(Transpose(previous), current);
  const double angle = std::acos(std::clamp(
    (delta.m[0][0] + delta.m[1][1] + delta.m[2][2] - 1) / 2, -1., 1.));
  // 大幅姿态跳变不是连续载机角速度，不将其转成关节冲击指令。
  if (angle > .5)
    return {};
  const double factor = angle < 1e-6 ? .5 / dt : angle / (2 * std::sin(angle) * dt);
  return {factor * (delta.m[2][1] - delta.m[1][2]),
          factor * (delta.m[0][2] - delta.m[2][0]),
          factor * (delta.m[1][0] - delta.m[0][1])};
}
std::array<double, 2> StabilizationRates(double yaw, double pitch, const Vec3 &omega)
{
  const auto los = Los(yaw, pitch);
  const Vec3 desired{omega.z * los.y - omega.y * los.z,
                     omega.x * los.z - omega.z * los.x,
                     omega.y * los.x - omega.x * los.y};
  const auto rotation = Rotation(yaw, pitch);
  const auto right = MatVec(rotation, {0, 1, 0});
  const auto down = MatVec(rotation, {-1, 0, 0});
  const auto dot = [](const Vec3 &a, const Vec3 &b) { return a.x*b.x + a.y*b.y + a.z*b.z; };
  const double degrees = 180 / M_PI;
  return ImageRates(pitch, dot(desired, right) * degrees, dot(desired, down) * degrees);
}
}
