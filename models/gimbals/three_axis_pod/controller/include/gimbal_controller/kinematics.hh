// 三轴朝天吊舱运动学（与 models/gimbals/three_axis_pod/model.sdf 保持一致）。
//
// 关节链： R_M->C = Rz(-yaw) @ Rx(-roll) @ Ry(pitch)
//   yaw 关节轴 (0,0,-1)，roll 轴 (-1,0,0)，pitch 轴 (0,1,0)
// 相机传感器朝向（rpy = 180,-90,0 度）：
//   传感器 x（视线）= camera_link +Z；传感器 y = -Y；传感器 z（图像上）= +X
//
// IK（朝天分支）： pitch = el - 90°， yaw = 180° - az
//
// 供 Gazebo 云台插件使用。
#ifndef GIMBAL_CONTROLLER_KINEMATICS_HH_
#define GIMBAL_CONTROLLER_KINEMATICS_HH_

#include <array>
#include <utility>

namespace gimbal {

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Mat3
{
  double m[3][3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

Mat3 Identity();
Mat3 MatMul(const Mat3 &_a, const Mat3 &_b);
Vec3 MatVec(const Mat3 &_a, const Vec3 &_v);

Mat3 Rz(double _t);
Mat3 Ry(double _t);
Mat3 Rx(double _t);

/// 相机水平视场角 60°
constexpr double kHFOV = 1.0471975511965976;

/// 传感器 -> camera_link 旋转 R_CAM_SENSOR
const Mat3 &CamSensorRot();

Mat3 RMountCamera(double _yaw, double _roll, double _pitch);
Mat3 RMountSensor(double _yaw, double _roll, double _pitch);
Vec3 LosMount(double _yaw, double _roll, double _pitch);

/// 方向 -> (方位角, 俯仰角)，弧度。方位角自 +X 逆时针。
std::pair<double, double> AzElFromDir(const Vec3 &_d);

/// 朝天安装分支 IK：返回 (yaw, pitch)（弧度）。
std::pair<double, double> IkUp(double _az, double _el);

/// 像素坐标 -> 传感器系单位方向向量（与 Gazebo 相机渲染一致）。
Vec3 PixelToSensorRay(double _px, double _py, int _width, int _height,
                      double _hfov = kHFOV);

/// 传感器系方向 -> mount 系方位/俯仰角。
std::pair<double, double> SensorRayToAzEl(const Vec3 &_dSensor, double _yaw,
                                          double _roll, double _pitch);

double WrapPi(double _a);

/// 旋转矩阵 -> 四元数 (w, x, y, z)。
std::array<double, 4> QuatFromR(const Mat3 &_r);

/// 旋转矩阵 -> (roll, pitch, yaw) 角度制（ZYX 内旋）。
std::array<double, 3> RToRpyDeg(const Mat3 &_r);

/// 相机相对安装系姿态的欧拉角表示误差计算辅助（目标中心角误差）。
double AngleBetween(const Vec3 &_a, const Vec3 &_b);
Mat3 Transpose(const Mat3 &_r);
/// 参考前/右/下系中的相机FRD姿态，参考前由世界水平forwardAz指定。
Mat3 PublicRotation(double _yaw, double _roll, double _pitch,
                    const Mat3 &_mount, const Mat3 &_sensor, double _forwardAz);
/// 返回公共 yaw,pitch,roll (rad)。极点采用固定Euler分支，需同时检查PublicSingular。
std::array<double, 3> PublicAngles(const Mat3 &_rotation);
bool PublicSingular(const Mat3 &_rotation);
/// 公共Euler角速度 -> 机械yaw,pitch,roll角速度，含三轴耦合；输入输出保持相同单位。
bool PublicRatesToJoint(double _yaw, double _roll, double _pitch,
                        const Mat3 &_mount, const Mat3 &_sensor, double _forwardAz,
                        const std::array<double, 3> &_rates,
                        std::array<double, 3> &_jointRates);
/// 机械关节角速度 -> 公共Euler角速度，输入输出保持相同单位；Euler奇异时返回false。
bool JointRatesToPublic(double _yaw, double _roll, double _pitch,
                        const Mat3 &_mount, const Mat3 &_sensor, double _forwardAz,
                        const std::array<double, 3> &_rates,
                        std::array<double, 3> &_publicRates);

}  // namespace gimbal

#endif  // GIMBAL_CONTROLLER_KINEMATICS_HH_
