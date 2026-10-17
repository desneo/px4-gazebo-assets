#include "two_axis/control.hh"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace two_axis;
void Check(bool x, const char *m)
{
  if (!x)
    throw std::runtime_error(m);
}
int main()
{
  const double d = M_PI / 180;
  auto a = Los(0, 0);
  Check(std::abs(a.z - 1) < 1e-9, "零位朝天");
  a = Los(0, 90 * d);
  Check(std::abs(a.x - 1) < 1e-9, "pitch+90朝前");
  a = Los(90 * d, 0);
  Check(std::abs(a.z - 1) < 1e-9, "朝天时yaw不改变视线");
  a = Los(90 * d, 90 * d);
  Check(std::abs(a.y - 1) < 1e-9, "yaw+90时前向转到+Y");
// 有限差分验证图像控制 Jacobian，包括大角度和退化位形。
  for (double yaw :
       {
         -.7, .0, .6
         })
    for (double pitch :
  {
    -.6, .0, 1., M_PI / 2
    })
  {
    auto r = Rotation(yaw, pitch);
    auto right = MatVec(r, {0, 1, 0}), down = MatVec(r, {-1, 0, 0});
    auto l = Los(yaw, pitch), ly = Los(yaw + 1e-6, pitch), lp = Los(yaw, pitch + 1e-6);
    auto dot = [](Vec3 a, Vec3 b)
    {
      return a.x * b.x + a.y * b.y + a.z * b.z;
    };
    Vec3 dy{(ly.x - l.x) / 1e-6, (ly.y - l.y) / 1e-6, (ly.z - l.z) / 1e-6};
    Vec3 dp{(lp.x - l.x) / 1e-6, (lp.y - l.y) / 1e-6, (lp.z - l.z) / 1e-6};
    Check(std::abs(dot(dy, right) - std::sin(pitch)) < 1e-5, "yaw Jacobian");
    Check(std::abs(dot(dp, down) + 1) < 1e-5, "pitch Jacobian");
  }
  auto rates = ImageRates(0, 100, 20);
  Check(std::isfinite(rates[0]) && std::abs(rates[0]) < 1e-8, "退化时yaw速度有限且为零");
  auto angular = MountAngularVelocity(Identity(), Rz(.003), .01);
  Check(std::abs(angular.z - .3) < 1e-8 && std::abs(angular.x) < 1e-8,
        "动态安装姿态差分角速度");
  Check(MountAngularVelocity(Identity(), Rz(1), .01).z == 0,
        "姿态瞬移不生成补偿冲击");
  Check(MountAngularVelocity(Identity(), Rz(.003), 0).z == 0,
        "无效时间间隔不生成角速度");
  // 独立积分载机旋转与机械轴补偿，检查世界视线的一阶漂移。
  for (double yaw : {-.7, .2, .8})
    for (double pitch : {-.6, .5, 1.2})
  {
    Vec3 omega{.2, -.3, .4};
    auto feed = StabilizationRates(yaw, pitch, omega);
    const double dt = 1e-5;
    auto nextMount = MatMul(Rz(omega.z * dt), MatMul(Ry(omega.y * dt), Rx(omega.x * dt)));
    auto nextLos = MatVec(nextMount, Los(yaw + feed[0]*d*dt, pitch + feed[1]*d*dt));
    auto before = Los(yaw, pitch);
    const double drift = std::sqrt(std::pow(nextLos.x-before.x, 2) +
      std::pow(nextLos.y-before.y, 2) + std::pow(nextLos.z-before.z, 2)) / dt;
    Check(drift < .01, "动态补偿保持世界视线，阻尼误差有界");
  }
  // 距硬限位0.7°时，80°/s持续0.012s会越界。
  // 保护必须在加速度平滑之后生效，并且保留向内退出的能力。
  GimbalControl predictive;
  ControlConfig fast;
  fast.max_acc_dps2 = 100000;
  predictive.Configure(fast);
  predictive.Manual(-80, 0, 0);
  predictive.Step(0, false, cv::Mat(), 0, 0, {}, 0);
  JointSample nearLower;
  nearLower.yaw = -89.3 * d;
  auto safe = predictive.Step(.012, false, cv::Mat(), 0, 0, nearLower, .012);
  Check(nearLower.yaw / d + safe[0]*.012 >= -89.8, "负端点前瞻保护");
  Check(predictive.limit_flags()[0] && safe[0] > -80, "限制向外速度");
  predictive.Manual(80, 0, .012);
  safe = predictive.Step(.024, false, cv::Mat(), 0, 0, nearLower, .024);
  Check(safe[0] > 0, "端点前瞻允许反向");
  GimbalControl c;
  JointSample j;
  cv::Mat empty;
  c.Manual(50, 20, 10);
  auto cmd = c.Step(0, false, empty, 0, 0, j, 10);
  Check(cmd[0] > 0 && cmd[1] > 0, "手动机械速度");
  cmd = c.Step(.1, false, empty, 0, 0, j, 10.6);
  Check(cmd[0] == 0 && cmd[1] == 0 && c.mode() == Mode::IDLE, "手动超时立即停止");
  c.Manual(50, 20, 11);
  c.Step(.11, false, empty, 0, 0, j, 11);
  c.Manual(50, 20, 11.02);
  cmd = c.Step(.13, false, empty, 0, 0, j, 11.02);
  Check(cmd[0] > 3, "手动续发持续加速");
  c.Manual(0, 0, 11.03);
  cmd = c.Step(.14, false, empty, 0, 0, j, 12);
  Check(cmd[0] == 0 && cmd[1] == 0 && c.mode() == Mode::IDLE &&
        c.client_message().empty(), "零速手动指令直接停车且不触发超时提示");
  c.Stop();
  j = {90 * d, 90 * d, 0, 0};
  c.Manual(50, 50, 20);
  cmd = c.Step(.2, false, empty, 0, 0, j, 20);
  Check(cmd[0] == 0 && cmd[1] == 0 && c.limit_flags()[0] && c.limit_flags()[1], "正限位保护");
  c.Manual(-50, -50, 20);
  cmd = c.Step(.3, false, empty, 0, 0, j, 20);
  Check(cmd[0] < 0 && cmd[1] < 0, "限位可反向退出");
  j = {-90 * d, -45 * d, 0, 0};
  c.Manual(-50, -50, 20);
  cmd = c.Step(.4, false, empty, 0, 0, j, 20);
  Check(cmd[0] == 0 && cmd[1] == 0, "负限位保护");
  j = {.3, -.2, 0, 0};
  Check(c.ResetAxis("yaw", j), "yaw复位");
  cmd = c.Step(.5, false, empty, 0, 0, j, 20);
  Check(cmd[0] < 0 && cmd[1] == 0, "单轴复位保持另轴");
  Check(!c.ResetAxis("roll", j), "拒绝roll");
  c.GoHome();
  cmd = c.Step(.6, false, empty, 0, 0, j, 20);
  Check(cmd[0] < 0 && cmd[1] > 0, "回机械零位");
  c.Stop();
  Check(c.cmd()[0] == 0, "stop立即停止");
  cv::Mat im(240, 320, CV_8UC3);
  cv::RNG rng(42);
  rng.fill(im, cv::RNG::UNIFORM, 0, 255);
  Check(c.StartTracking(im, {.3, .3, .25, .25}, 320, 240).first, "CSRT选框");
  c.Step(1, true, im, 320, 240, {}, 30);
  Check(c.track_state() == TrackState::TRACK && c.target_valid(), "CSRT持续跟踪");
  Check(c.client_message().find("视线退化") != std::string::npos,
        "CSRT仍识别目标时明确报告视线退化");
  cmd = c.Step(1.6, false, im, 320, 240, {}, 30.6);
  Check(cmd[0] == 0 && cmd[1] == 0 && !c.target_valid(), "断帧立即停止");
  c.Step(4.2, false, im, 320, 240, {}, 33.2);
  Check(c.track_state() == TrackState::LOST, "长期断帧进入丢失");
  c.Stop();
  Check(c.client_message().empty(), "停止后清除已失效的跟踪约束提示");
  Check(c.StartTracking(im, {.65, .35, .2, .2}, 320, 240).first,
        "机械边界图像选框有效");
  JointSample trackedLimit{89.9*d, 45*d, 0, 0};
  c.Step(4.3, true, im, 320, 240, trackedLimit, 33.3);
  Check(c.track_state() == TrackState::TRACK && c.limit_flags()[0] &&
        c.client_message().find("机械限位") != std::string::npos &&
        c.stability() == Stability::UNSTABLE,
        "图像识别正常但机械不可继续向外时不能宣称稳定锁定");
  c.Stop();
  JointSample invalid;
  invalid.yaw = std::nan("");
  c.Step(5, false, empty, 0, 0, invalid, 34);
  Check(c.track_state() == TrackState::FAULT, "无效关节进入故障");
  c.Manual(20, 20, 34);
  cmd = c.Step(5.1, false, empty, 0, 0, {}, 34.1);
  Check(cmd[0] == 0 && cmd[1] == 0 && c.track_state() == TrackState::FAULT, "故障锁定");
  c.ResetFault();
  Check(c.fault_reason().empty() && c.track_state() == TrackState::IDLE, "故障复位");
  std::cout << "two-axis control and kinematics passed\n";
}
