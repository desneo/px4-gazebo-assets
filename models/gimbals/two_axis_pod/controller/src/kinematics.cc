#include "two_axis/kinematics.hh"
#include <cmath>
namespace two_axis
{
Mat3 Rotation(double y, double p)
{
  return MatMul(Rz(y), Ry(p));
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
  // 对视线导数投影到相机图像右(+Y)、下(-X)：J=diag(sin(p),-1)。
  // 阻尼最小二乘避免朝天时 yaw 失去视线控制能力后速度发散。
  const double s = std::sin(p), damping = 0.05;
  return {s*right / (s * s + damping * damping), -down};
}
}
