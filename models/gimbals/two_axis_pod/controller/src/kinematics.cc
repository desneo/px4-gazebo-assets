#include "two_axis/kinematics.hh"
#include <cmath>
#include <algorithm>
namespace two_axis
{
Mat3 Rotation(double y, double p)
{
  return MatMul(Rx(-y), Ry(-p));
}
Mat3 SensorRotation()
{
  // sensor +X 为视线 +Z，+Y 为图像左 +Y，+Z 为图像上 -X。
  return Ry(-M_PI / 2);
}
bool ValidSensorRotation(const Mat3 &sensor)
{
  const auto expected = SensorRotation();
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      if (!std::isfinite(sensor.m[row][column]) ||
          std::abs(sensor.m[row][column] - expected.m[row][column]) > 1e-8)
        return false;
  return true;
}
Mat3 OpticalRotation(double y, double p, const Mat3 &sensor)
{
  return MatMul(Rotation(y, p), sensor);
}
Vec3 Los(double y, double p)
{
  return MatVec(OpticalRotation(y, p, SensorRotation()), {1, 0, 0});
}
bool Singular(double p)
{
  return std::abs(std::cos(p)) < 0.05;
}
std::array<double, 2> ImageRates(double p, double right, double down)
{
  // 外轴 -X、内轴随动 -Y；图像右(-Y)、下(+X)：J=diag(-cos(p),-1)。
  // 视线接近外轴时以阻尼最小二乘抑制 yaw 速度发散。
  const double s = -std::cos(p), damping = 0.05;
  return {s*right / (s * s + damping * damping), -down};
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
  const auto optical = OpticalRotation(yaw, pitch, SensorRotation());
  const auto right = MatVec(optical, {0, -1, 0});
  const auto down = MatVec(optical, {0, 0, -1});
  const auto dot = [](const Vec3 &a, const Vec3 &b) { return a.x*b.x + a.y*b.y + a.z*b.z; };
  const double degrees = 180 / M_PI;
  return ImageRates(pitch, dot(desired, right) * degrees, dot(desired, down) * degrees);
}
}
