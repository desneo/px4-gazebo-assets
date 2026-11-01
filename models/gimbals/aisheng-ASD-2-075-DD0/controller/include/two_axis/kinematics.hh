#pragma once
#include "gimbal_controller/kinematics.hh"
namespace two_axis
{
using namespace gimbal;
// 外轴 +X、随动内轴 -Y：Rx(yaw) Ry(-pitch)，yaw 左负右正。
Mat3 Rotation(double yaw, double pitch);
// 实际 SDF joint 局部轴向必须与运动学一致，拒绝旧 yaw=-X 模型。
bool ValidJointAxis(std::size_t axis, const Vec3 &direction);
// sensor -> camera_link 的固定光学旋转：RPY=(0,-pi/2,0)。
Mat3 SensorRotation();
// 图像伺服只支持 CAD 固定光学基底，拒绝不匹配或无效的 SDF 外参。
bool ValidSensorRotation(const Mat3 &sensor);
// 机械旋转与传感器局部旋转各应用一次，返回 sensor -> mount。
Mat3 OpticalRotation(double yaw, double pitch, const Mat3 &sensor);
Vec3 Los(double yaw, double pitch);
// 图像右/下方向角误差 -> 机械角速度；单位保持一致。
std::array<double, 2> ImageRates(double pitch, double right, double down);
bool Singular(double pitch);
// 两次安装姿态 -> 安装系角速度（rad/s），用于动态载机视线前馈。
Vec3 MountAngularVelocity(const Mat3 &previous, const Mat3 &current, double dt);
// Gazebo世界系角速度 -> 当前安装局部系角速度。
Vec3 WorldAngularVelocityInMount(const Mat3 &mount_world, const Vec3 &world_omega);
// 仅在Gazebo角速度不可用时使用的姿态差分后备；35Hz一阶低通并拒绝单步大跳变。
class MountRateFallback
{
public:
  void Observe(const Mat3 &rotation);
  Vec3 Update(const Mat3 &rotation, double dt);

private:
  Mat3 previous_;
  Vec3 filtered_;
  bool initialized_{false};
};
// 抵消安装系转动产生的视线角速度；返回机械角速度（deg/s）。
std::array<double, 2> StabilizationRates(double yaw, double pitch, const Vec3 &omega);
}
