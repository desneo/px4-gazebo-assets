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
// 两次安装姿态 -> 安装系角速度（rad/s），用于动态载机视线前馈。
Vec3 MountAngularVelocity(const Mat3 &previous, const Mat3 &current, double dt);
// 抵消安装系转动产生的视线角速度；返回机械角速度（deg/s）。
std::array<double, 2> StabilizationRates(double yaw, double pitch, const Vec3 &omega);
}
