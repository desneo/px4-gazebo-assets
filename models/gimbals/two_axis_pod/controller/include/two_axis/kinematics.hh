#pragma once
#include "gimbal_controller/kinematics.hh"
namespace two_axis
{
using namespace gimbal;
Mat3 Rotation(double yaw, double pitch);
Vec3 Los(double yaw, double pitch);
// 图像右/下方向角误差 -> 机械角速度；单位保持一致。
std::array<double, 2> ImageRates(double pitch, double right, double down);
bool Singular(double pitch);
}
