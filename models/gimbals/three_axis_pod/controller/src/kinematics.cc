#include "gimbal_controller/kinematics.hh"

#include <algorithm>
#include <cmath>

namespace gimbal {

Mat3 Identity()
{
  return Mat3{};
}

Mat3 MatMul(const Mat3 &_a, const Mat3 &_b)
{
  Mat3 r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {
      double s = 0.0;
      for (int k = 0; k < 3; ++k)
        s += _a.m[i][k] * _b.m[k][j];
      r.m[i][j] = s;
    }
  return r;
}

Vec3 MatVec(const Mat3 &_a, const Vec3 &_v)
{
  Vec3 r;
  r.x = _a.m[0][0] * _v.x + _a.m[0][1] * _v.y + _a.m[0][2] * _v.z;
  r.y = _a.m[1][0] * _v.x + _a.m[1][1] * _v.y + _a.m[1][2] * _v.z;
  r.z = _a.m[2][0] * _v.x + _a.m[2][1] * _v.y + _a.m[2][2] * _v.z;
  return r;
}

Mat3 Rz(double _t)
{
  const double c = std::cos(_t), s = std::sin(_t);
  Mat3 r;
  r.m[0][0] = c; r.m[0][1] = -s; r.m[0][2] = 0.0;
  r.m[1][0] = s; r.m[1][1] = c;  r.m[1][2] = 0.0;
  r.m[2][0] = 0.0; r.m[2][1] = 0.0; r.m[2][2] = 1.0;
  return r;
}

Mat3 Ry(double _t)
{
  const double c = std::cos(_t), s = std::sin(_t);
  Mat3 r;
  r.m[0][0] = c;  r.m[0][1] = 0.0; r.m[0][2] = s;
  r.m[1][0] = 0.0; r.m[1][1] = 1.0; r.m[1][2] = 0.0;
  r.m[2][0] = -s; r.m[2][1] = 0.0; r.m[2][2] = c;
  return r;
}

Mat3 Rx(double _t)
{
  const double c = std::cos(_t), s = std::sin(_t);
  Mat3 r;
  r.m[0][0] = 1.0; r.m[0][1] = 0.0; r.m[0][2] = 0.0;
  r.m[1][0] = 0.0; r.m[1][1] = c;   r.m[1][2] = -s;
  r.m[2][0] = 0.0; r.m[2][1] = s;   r.m[2][2] = c;
  return r;
}

const Mat3 &CamSensorRot()
{
  static const Mat3 kR = [] {
    Mat3 r;
    r.m[0][0] = 0.0; r.m[0][1] = 0.0; r.m[0][2] = 1.0;
    r.m[1][0] = 0.0; r.m[1][1] = 1.0; r.m[1][2] = 0.0;
    r.m[2][0] = 1.0; r.m[2][1] = 0.0; r.m[2][2] = 0.0;
    return r;
  }();
  return kR;
}

Mat3 RMountCamera(double _yaw, double _roll, double _pitch)
{
  return MatMul(Rz(-_yaw), MatMul(Rx(-_roll), Ry(_pitch)));
}

Mat3 RMountSensor(double _yaw, double _roll, double _pitch)
{
  return MatMul(RMountCamera(_yaw, _roll, _pitch), CamSensorRot());
}

Vec3 LosMount(double _yaw, double _roll, double _pitch)
{
  return MatVec(RMountCamera(_yaw, _roll, _pitch), Vec3{0.0, 0.0, 1.0});
}

std::pair<double, double> AzElFromDir(const Vec3 &_d)
{
  const double az = std::atan2(_d.y, _d.x);
  const double el = std::asin(std::max(-1.0, std::min(1.0, _d.z)));
  return {az, el};
}

std::pair<double, double> IkUp(double _az, double _el)
{
  return {M_PI - _az, _el - M_PI / 2.0};
}

Vec3 PixelToSensorRay(double _px, double _py, int _width, int _height,
                      double _hfov)
{
  const double f = (_width / 2.0) / std::tan(_hfov / 2.0);
  const double ax = std::atan((_px - _width / 2.0) / f);
  const double ay = std::atan((_py - _height / 2.0) / f);
  Vec3 d{1.0, std::tan(ax), -std::tan(ay)};
  const double n = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
  d.x /= n; d.y /= n; d.z /= n;
  return d;
}

std::pair<double, double> SensorRayToAzEl(const Vec3 &_dSensor, double _yaw,
                                          double _roll, double _pitch)
{
  const Vec3 d = MatVec(RMountSensor(_yaw, _roll, _pitch), _dSensor);
  return AzElFromDir(d);
}

double WrapPi(double _a)
{
  const double wrapped =
      (_a + M_PI) - 2.0 * M_PI * std::floor((_a + M_PI) / (2.0 * M_PI));
  return wrapped - M_PI;
}

std::array<double, 4> QuatFromR(const Mat3 &_r)
{
  const Mat3 &m = _r;
  const double tr = m.m[0][0] + m.m[1][1] + m.m[2][2];
  double w, x, y, z;
  if (tr > 0.0)
  {
    const double s = std::sqrt(tr + 1.0) * 2.0;
    w = 0.25 * s;
    x = (m.m[2][1] - m.m[1][2]) / s;
    y = (m.m[0][2] - m.m[2][0]) / s;
    z = (m.m[1][0] - m.m[0][1]) / s;
  }
  else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2])
  {
    const double s = std::sqrt(1.0 + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.0;
    w = (m.m[2][1] - m.m[1][2]) / s;
    x = 0.25 * s;
    y = (m.m[0][1] + m.m[1][0]) / s;
    z = (m.m[0][2] + m.m[2][0]) / s;
  }
  else if (m.m[1][1] > m.m[2][2])
  {
    const double s = std::sqrt(1.0 + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.0;
    w = (m.m[0][2] - m.m[2][0]) / s;
    x = (m.m[0][1] + m.m[1][0]) / s;
    y = 0.25 * s;
    z = (m.m[1][2] + m.m[2][1]) / s;
  }
  else
  {
    const double s = std::sqrt(1.0 + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.0;
    w = (m.m[1][0] - m.m[0][1]) / s;
    x = (m.m[0][2] + m.m[2][0]) / s;
    y = (m.m[1][2] + m.m[2][1]) / s;
    z = 0.25 * s;
  }
  return {w, x, y, z};
}

std::array<double, 3> RToRpyDeg(const Mat3 &_r)
{
  const Mat3 &R = _r;
  double roll, pitch, yaw;
  pitch = std::asin(std::max(-1.0, std::min(1.0, -R.m[2][0])));
  if (std::abs(R.m[2][0]) < 0.999999)
  {
    roll = std::atan2(R.m[2][1], R.m[2][2]);
    yaw = std::atan2(R.m[1][0], R.m[0][0]);
  }
  else
  {
    roll = 0.0;
    yaw = std::atan2(-R.m[0][1], R.m[1][1]);
  }
  constexpr double kRad2Deg = 180.0 / M_PI;
  return {roll * kRad2Deg, pitch * kRad2Deg, yaw * kRad2Deg};
}

double AngleBetween(const Vec3 &_a, const Vec3 &_b)
{
  double dot = _a.x * _b.x + _a.y * _b.y + _a.z * _b.z;
  dot = std::max(-1.0, std::min(1.0, dot));
  return std::acos(dot);
}

}  // namespace gimbal
