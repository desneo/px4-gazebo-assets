#include "two_axis/kinematics.hh"
#include <cmath>
#include <algorithm>
namespace two_axis
{
Mat3 Rotation(double y, double p)
{
  return MatMul(Rx(y), Ry(-p));
}
bool ValidJointAxis(std::size_t axis, const Vec3 &direction)
{
  if (axis > 1 || !std::isfinite(direction.x) ||
      !std::isfinite(direction.y) || !std::isfinite(direction.z))
    return false;
  const Vec3 expected = axis == 0 ? Vec3{1, 0, 0} : Vec3{0, -1, 0};
  return std::abs(direction.x - expected.x) <= 1e-8 &&
         std::abs(direction.y - expected.y) <= 1e-8 &&
         std::abs(direction.z - expected.z) <= 1e-8;
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
  // 外轴 +X、内轴随动 -Y；图像右(-Y)、下(+X)：J=diag(cos(p),-1)。
  // 视线接近外轴时以阻尼最小二乘抑制 yaw 速度发散。
  const double s = std::cos(p), damping = 0.05;
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
Vec3 WorldAngularVelocityInMount(const Mat3 &mount_world, const Vec3 &world_omega)
{
  return MatVec(Transpose(mount_world), world_omega);
}
void MountRateFallback::Observe(const Mat3 &rotation)
{
  this->previous_ = rotation;
  this->filtered_ = {};
  this->initialized_ = true;
}
Vec3 MountRateFallback::Update(const Mat3 &rotation, double dt)
{
  if (!this->initialized_)
  {
    this->Observe(rotation);
    return {};
  }
  const auto delta = MatMul(Transpose(this->previous_), rotation);
  const double cosine =
    (delta.m[0][0] + delta.m[1][1] + delta.m[2][2] - 1) / 2;
  const double angle = std::acos(std::clamp(cosine, -1., 1.));
  if (!std::isfinite(dt) || dt <= 0 || dt > .1 ||
      !std::isfinite(angle) || angle > .5)
  {
    this->Observe(rotation);
    return {};
  }
  const auto raw = MountAngularVelocity(this->previous_, rotation, dt);
  this->previous_ = rotation;
  if (!std::isfinite(raw.x) || !std::isfinite(raw.y) || !std::isfinite(raw.z))
  {
    this->filtered_ = {};
    return {};
  }
  constexpr double cutoffHz = 35.0;
  const double alpha = 1 - std::exp(-2 * M_PI * cutoffHz * dt);
  this->filtered_.x += alpha * (raw.x - this->filtered_.x);
  this->filtered_.y += alpha * (raw.y - this->filtered_.y);
  this->filtered_.z += alpha * (raw.z - this->filtered_.z);
  return this->filtered_;
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
