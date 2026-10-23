#include "two_axis/control.hh"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace two_axis;
void Check(bool x, const char *m)
{
  if (!x)
    throw std::runtime_error(m);
}
void CheckRate(const std::array<double, 2> &cmd, double limit = kMaxJointRateDps)
{
  for (double value : cmd)
    Check(std::isfinite(value) && std::abs(value) <= limit + 1e-10,
          "每轴最终机械速度指令必须为有限值且不超过有效上限");
}
int main()
{
  const double d = M_PI / 180;
  TrackingBackend backend = TrackingBackend::CSRT;
  Check(ParseTrackingBackend("sim_truth", backend) &&
        backend == TrackingBackend::SIM_TRUTH &&
        std::string(ToString(backend)) == "sim_truth",
        "真值后端配置可解析");
  Check(ParseTrackingBackend("csrt", backend) &&
        backend == TrackingBackend::CSRT &&
        !ParseTrackingBackend("tracker", backend),
        "仅接受sim_truth和csrt后端");
  const auto projectedCenter = ProjectTruthTarget(
    1, "center", {10, 0, 0}, 320, 240, M_PI / 3, .1, 100);
  const auto projectedLeftUp = ProjectTruthTarget(
    2, "left_up", {10, 1, 1}, 320, 240, M_PI / 3, .1, 100);
  Check(projectedCenter && projectedCenter->center_x == 160 &&
        projectedCenter->center_y == 120 && projectedLeftUp &&
        projectedLeftUp->center_x < 160 && projectedLeftUp->center_y < 120,
        "相机真值投影中心及左右上下符号正确");
  Check(!ProjectTruthTarget(3, "behind", {-1, 0, 0}, 320, 240,
                            M_PI / 3, .1, 100),
        "相机后方或裁剪范围外目标不可投影");
  const std::vector<TruthProjection> choices{
    {1, "first", 120, 100, 10}, {2, "nearest", 158, 121, 12},
    {3, "outside", 280, 200, 9}};
  const auto selected = SelectTruthTarget(choices, {80, 60, 160, 120}, 320, 240);
  Check(selected && choices[*selected].name == "nearest",
        "框内多架飞机按离选框中心最近者绑定");
  const auto truthBox = MakeTruthObservation(
    choices[1], {110, 80, 100, 80}, 12, 320, 240, 77);
  Check(truthBox.status == TruthStatus::VISIBLE && truthBox.frame_seq == 77 &&
        truthBox.center_x == 158 && truthBox.center_y == 121,
        "真值观测保留真实中心与关联帧");

  ControlConfig rateConfig;
  Check(rateConfig.max_rate_dps == 20 && rateConfig.max_acc_dps2 == 160,
        "默认机械轴速度20度每秒，加速度160度每秒平方");
  Check(JointRateLimit(100) == 20 && JointRateLimit(7.5) == 7.5,
        "速度配置只能收紧，旧100度每秒配置不能绕过20度硬上限");
  for (double invalidRate : {0., -1., std::nan(""), std::numeric_limits<double>::infinity()})
    Check(JointRateLimit(invalidRate) == 0, "无效速度配置关闭运动输出");
  GimbalControl boundedManual;
  rateConfig.max_rate_dps = 100;
  boundedManual.Configure(rateConfig);
  std::array<double, 2> previous{};
  for (int i = 0; i <= 60; ++i)
  {
    const double now = i*.01;
    const double sign = i <= 30 ? 1. : -1.;
    boundedManual.Manual(sign*1000, -sign*1000, now);
    const auto output = boundedManual.Step(now, false, {}, 0, 0, {}, now);
    CheckRate(output);
    for (int axis = 0; axis < 2; ++axis)
      Check(std::abs(output[axis]-previous[axis]) <= 160*.01 + 1e-9,
            "双轴正反向运动保持160度每秒平方加速度平滑");
    if (i == 30)
      Check(std::abs(output[0]-20) < 1e-9 && std::abs(output[1]+20) < 1e-9,
            "大手动请求在20度每秒饱和");
    previous = output;
  }
  Check(std::abs(previous[0]+20) < 1e-9 && std::abs(previous[1]-20) < 1e-9,
        "正反两个方向均受硬上限约束");
  ControlConfig tightened = rateConfig;
  tightened.max_rate_dps = 5;
  boundedManual.Configure(tightened);
  CheckRate(boundedManual.cmd(), 5);
  CheckRate(boundedManual.Step(.6, false, {}, 0, 0, {}, .6), 5);
  Check(std::abs(boundedManual.cmd()[0]+5) < 1e-9,
        "运行中降低上限立即约束残留cmd，零dt也不能超出新上限");
  tightened.max_rate_dps = 1000;
  boundedManual.Configure(tightened);
  for (int i = 61; i <= 80; ++i)
    CheckRate(boundedManual.Step(i*.01, false, {}, 0, 0, {}, i*.01));
  Check(std::abs(boundedManual.cmd()[0]+20) < 1e-9,
        "再次调高配置仍只能恢复到20度每秒");
  tightened.max_rate_dps = std::nan("");
  boundedManual.Configure(tightened);
  CheckRate(boundedManual.cmd(), 0);

  for (int sign : {-1, 1})
  {
    JointSample displaced{sign*45*d, sign*20*d, 0, 0};
    GimbalControl home;
    home.Configure(rateConfig);
    home.GoHome();
    for (int i = 0; i <= 30; ++i)
      CheckRate(home.Step(i*.01, false, {}, 0, 0, displaced, i*.01));
    Check(std::abs(home.cmd()[0]+sign*20) < 1e-9 &&
          std::abs(home.cmd()[1]+sign*20) < 1e-9,
          "大偏角双轴回零沿机械方向以不超过20度每秒运动");
    for (const std::string axis : {"yaw", "pitch"})
    {
      GimbalControl reset;
      reset.Configure(rateConfig);
      Check(reset.ResetAxis(axis, displaced), "单轴复位接受机械轴名");
      for (int i = 0; i <= 30; ++i)
        CheckRate(reset.Step(i*.01, false, {}, 0, 0, displaced, i*.01));
      const int active = axis == "yaw" ? 0 : 1;
      Check(std::abs(reset.cmd()[active]+sign*20) < 1e-9 && reset.cmd()[1-active] == 0,
            "单轴复位限速20度每秒且保持另一轴");
    }
  }
  auto miss = MeasureTargetMiss(960, 540, 1920, 1080);
  Check(miss.valid && miss.center_px[0] == 960 && miss.center_px[1] == 540 &&
        miss.offset_px[0] == 0 && miss.offset_px[1] == 0 &&
        miss.offset_deg[0] == 0 && miss.offset_deg[1] == 0,
        "画面中心脱靶量为零，中心坐标采用左上角原点");
  for (int sx : {-1, 1})
    for (int sy : {-1, 1})
  {
    miss = MeasureTargetMiss(960 + sx*480, 540 + sy*270, 1920, 1080);
    Check(miss.valid && miss.offset_px[0] == sx*480 &&
          miss.offset_px[1] == sy*270 &&
          std::abs(miss.offset_deg[0] - sx*std::atan(.5*std::tan(M_PI/6))/d) < 1e-9 &&
          std::abs(miss.offset_deg[1] - sy*std::atan(270.*std::tan(M_PI/6)/960)/d) < 1e-9,
          "四象限脱靶量右下为正，角度采用相机焦距而非线性视场比例");
  }
  Check(!MeasureTargetMiss(0, 0, 0, 1080).valid &&
        !MeasureTargetMiss(std::nan(""), 0, 1920, 1080).valid,
        "无效图像尺寸或坐标不能产生有效脱靶量");
  auto a = Los(0, 0);
  Check(std::abs(a.z - 1) < 1e-9 && !Singular(0), "零位光轴+Z且两轴均可控制视线");
  a = Los(0, -80 * d);
  Check(std::abs(a.x - std::sin(80*d)) < 1e-9 &&
        std::abs(a.z - std::cos(80*d)) < 1e-9, "负俯仰朝安装前方");
  a = Los(0, 30 * d);
  Check(std::abs(a.x + .5) < 1e-9 &&
        std::abs(a.z - std::sqrt(.75)) < 1e-9, "正俯仰朝安装后方");
  a = Los(60 * d, 0);
  Check(std::abs(a.y + std::sqrt(.75)) < 1e-9 &&
        std::abs(a.z - .5) < 1e-9, "yaw绕安装+X使零位光轴朝图像右侧-Y转动");
  a = Los(-60 * d, 0);
  Check(std::abs(a.y - std::sqrt(.75)) < 1e-9 && std::abs(a.z - .5) < 1e-9,
        "负yaw使视线朝图像左侧+Y转动");
  Check(ValidJointAxis(0, {1, 0, 0}) && ValidJointAxis(1, {0, -1, 0}) &&
        !ValidJointAxis(0, {-1, 0, 0}) && !ValidJointAxis(1, {0, 1, 0}) &&
        !ValidJointAxis(2, {1, 0, 0}) && !ValidJointAxis(0, {std::nan(""), 0, 0}),
        "只接受右正yaw和原pitch轴向，拒绝旧模型、未知轴与非有限轴向");
  a = Los(60 * d, -80 * d);
  Check(std::abs(a.x - std::sin(80*d)) < 1e-9 &&
        std::abs(a.y + std::sin(60*d)*std::cos(80*d)) < 1e-9 &&
        std::abs(a.z - std::cos(60*d)*std::cos(80*d)) < 1e-9,
        "组合机械角符合两条实际转轴");
  Check(!Singular(-80*d) && !Singular(30*d) &&
        Singular(-90*d) && Singular(90*d), "确认范围无奇异点，视线平行外轴时退化");
  const auto zeroOptical = OpticalRotation(0, 0, SensorRotation());
  const auto left = MatVec(zeroOptical, {0, 1, 0});
  const auto up = MatVec(zeroOptical, {0, 0, 1});
  Check(std::abs(left.y - 1) < 1e-9 && std::abs(up.x + 1) < 1e-9,
        "光学基底图像左+Y、图像上-X");
  Check(ValidSensorRotation(SensorRotation()) &&
        ValidSensorRotation(MatMul(Ry(-M_PI/2), Rx(1e-10))),
        "固定光学外参允许数值舍入误差");
  Check(!ValidSensorRotation(Identity()) && !ValidSensorRotation(CamSensorRot()) &&
        !ValidSensorRotation(MatMul(SensorRotation(), Rx(1e-5))),
        "拒绝错误基底、旧模型光学旋转和超出容差的外参");
  auto invalidSensor = SensorRotation();
  invalidSensor.m[0][0] = std::nan("");
  Check(!ValidSensorRotation(invalidSensor), "无效光学外参不能启用图像伺服");
  // 由实际 sensor 局部外参求姿态，不得再叠加默认光学旋转。
  for (const auto &sensor : {SensorRotation(), Identity(), MatMul(SensorRotation(), Rx(.2))})
  {
    const auto optical = OpticalRotation(.4, -.6, sensor);
    const auto ray = MatVec(optical, {1, 0, 0});
    const auto localRay = MatVec(sensor, {1, 0, 0});
    const auto expected = MatVec(Rx(.4), MatVec(Ry(.6), localRay));
    Check(std::abs(ray.x-expected.x) < 1e-9 && std::abs(ray.y-expected.y) < 1e-9 &&
          std::abs(ray.z-expected.z) < 1e-9, "传感器局部旋转和机械旋转各应用一次");
  }
// 有限差分验证图像控制 Jacobian，包括大角度和退化位形。
  for (double yaw :
       {
         -.7, .0, .6
         })
    for (double pitch :
  {
    -M_PI / 2, -80*d, -1., .0, 30*d, M_PI / 2
    })
  {
    auto optical = OpticalRotation(yaw, pitch, SensorRotation());
    auto right = MatVec(optical, {0, -1, 0}), down = MatVec(optical, {0, 0, -1});
    auto l = Los(yaw, pitch), ly = Los(yaw + 1e-6, pitch), lp = Los(yaw, pitch + 1e-6);
    auto dot = [](Vec3 a, Vec3 b)
    {
      return a.x * b.x + a.y * b.y + a.z * b.z;
    };
    Vec3 dy{(ly.x - l.x) / 1e-6, (ly.y - l.y) / 1e-6, (ly.z - l.z) / 1e-6};
    Vec3 dp{(lp.x - l.x) / 1e-6, (lp.y - l.y) / 1e-6, (lp.z - l.z) / 1e-6};
    Check(std::abs(dot(dy, right) - std::cos(pitch)) < 1e-5, "yaw Jacobian");
    Check(std::abs(dot(dp, down) + 1) < 1e-5, "pitch Jacobian");
    Check(std::abs(dot(dy, down)) < 1e-5 && std::abs(dot(dp, right)) < 1e-5,
          "两个机械轴在图像方向中无交叉项");
  }
  auto rates = ImageRates(-90*d, 100, 20);
  Check(std::isfinite(rates[0]) && std::abs(rates[0]) < 1e-8, "退化时yaw速度有限且为零");
  Check(rates[1] == -20, "图像下方误差对应负俯仰速度");
  rates = ImageRates(0, 20, -10);
  Check(rates[0] > 0 && rates[1] == 10, "零位图像右上误差对应正偏航正俯仰");
  for (double pitch : {-80*d, 0., 30*d})
  {
    rates = ImageRates(pitch, 20, 10);
    Check(rates[0] > 0 && rates[1] == -10, "确认范围图像右下误差对应正偏航负俯仰");
  }
  for (int sign : {-1, 1})
  {
    rates = ImageRates(-70*d, sign*20., 0);
    Check(sign*rates[0] > 0 && rates[1] == 0,
          "图像左右目标误差分别驱动负正yaw，不改变pitch");
    const auto feed = StabilizationRates(0, 0, {sign*.1, 0, 0});
    Check(sign*feed[0] < 0 && std::abs(feed[1]) < 1e-9,
          "零位载机绕+X转动时yaw反向补偿，保持世界视线");
  }
  auto angular = MountAngularVelocity(Identity(), Rz(.003), .01);
  Check(std::abs(angular.z - .3) < 1e-8 && std::abs(angular.x) < 1e-8,
        "动态安装姿态差分角速度");
  Check(MountAngularVelocity(Identity(), Rz(1), .01).z == 0,
        "姿态瞬移不生成补偿冲击");
  Check(MountAngularVelocity(Identity(), Rz(.003), 0).z == 0,
        "无效时间间隔不生成角速度");
  // 独立积分载机旋转与机械轴补偿，检查世界视线的一阶漂移。
  for (double yaw : {-.7, .2, .8})
    for (double pitch : {-80*d, -1.2, -.5, 0., 30*d})
  {
    Vec3 omega{.2, -.3, .4};
    auto feed = StabilizationRates(yaw, pitch, omega);
    const double dt = 1e-5;
    auto nextMount = MatMul(Rz(omega.z * dt), MatMul(Ry(omega.y * dt), Rx(omega.x * dt)));
    auto nextLos = MatVec(nextMount, Los(yaw + feed[0]*d*dt, pitch + feed[1]*d*dt));
    auto before = Los(yaw, pitch);
    const double drift = std::sqrt(std::pow(nextLos.x-before.x, 2) +
      std::pow(nextLos.y-before.y, 2) + std::pow(nextLos.z-before.z, 2)) / dt;
    const double dampingBound = .0025 / (std::pow(std::cos(pitch), 2) + .0025) *
      std::sqrt(omega.x*omega.x + omega.y*omega.y + omega.z*omega.z);
    Check(drift < dampingBound + 1e-5, "动态补偿保持世界视线，阻尼误差有界");
  }
  // 距硬限位0.5°时，20°/s持续两倍0.012s控制间隔会跨过软件缓冲。
  // 保护必须在加速度平滑之后生效，并且保留向内退出的能力。
  GimbalControl predictive;
  ControlConfig fast;
  Check(fast.yaw_min_deg == -60 && fast.yaw_max_deg == 60 &&
        fast.pitch_min_deg == -80 && fast.pitch_max_deg == 30,
        "默认机械限位与确认的新模型范围一致");
  fast.max_acc_dps2 = 100000;
  fast.max_rate_dps = 100;
  predictive.Configure(fast);
  predictive.Manual(-80, 0, 0);
  predictive.Step(0, false, cv::Mat(), 0, 0, {}, 0);
  JointSample nearLower;
  nearLower.yaw = -59.5 * d;
  auto safe = predictive.Step(.012, false, cv::Mat(), 0, 0, nearLower, .012);
  Check(nearLower.yaw / d + safe[0]*.024 >= -59.8, "负端点两倍控制间隔前瞻保护");
  Check(predictive.limit_flags()[0] && safe[0] > -20, "20度每秒内仍限制向外速度");
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
  j = {60 * d, 30 * d, 0, 0};
  c.Manual(50, 50, 20);
  cmd = c.Step(.2, false, empty, 0, 0, j, 20);
  Check(cmd[0] == 0 && cmd[1] == 0 && c.limit_flags()[0] && c.limit_flags()[1], "正限位保护");
  c.Manual(-50, -50, 20);
  cmd = c.Step(.3, false, empty, 0, 0, j, 20);
  Check(cmd[0] < 0 && cmd[1] < 0, "限位可反向退出");
  j = {-60 * d, -80 * d, 0, 0};
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
  GimbalControl truthControl;
  Check(truthControl.StartTruthTracking({.4, .4, .2, .2}, 320, 240).first,
        "真值模式选框初始化不依赖图像纹理");
  TruthObservation truthVisible;
  truthVisible.status = TruthStatus::VISIBLE;
  truthVisible.box = {180, 90, 64, 48};
  truthVisible.center_x = 212;
  truthVisible.center_y = 114;
  truthVisible.frame_seq = 700;
  truthControl.Step(0, true, {}, 320, 240, {}, 0, 700, &truthVisible);
  Check(truthControl.track_state() == TrackState::TRACK &&
        truthControl.target_valid() &&
        truthControl.target_miss().frame_seq == 700,
        "真值模式直接输出绑定飞机的真实像素位置");
  TruthObservation unsynchronized;
  truthControl.Step(.1, true, {}, 320, 240, {}, .1, 701, &unsynchronized);
  Check(truthControl.track_state() == TrackState::TRACK &&
        truthControl.target_miss().frame_seq == 700,
        "真值快照暂未同步时保留上一测量且不误报丢失");
  TruthObservation invisible;
  invisible.status = TruthStatus::INVISIBLE;
  invisible.frame_seq = 702;
  truthControl.Step(.2, true, {}, 320, 240, {}, .2, 702, &invisible);
  Check(truthControl.track_state() == TrackState::TEMP_LOST &&
        !truthControl.target_valid(), "绑定飞机离开视野后进入临时丢失");
  truthVisible.frame_seq = 703;
  truthControl.Step(.3, true, {}, 320, 240, {}, .3, 703, &truthVisible);
  Check(truthControl.track_state() == TrackState::TRACK &&
        truthControl.target_miss().frame_seq == 703,
        "同一绑定飞机重入视野后恢复且不换锁");
  GimbalControl jitterControl;
  Check(jitterControl.StartTruthTracking({.4, .4, .2, .2}, 320, 240).first,
        "大幅画面抖动测试可初始化真值跟踪");
  const std::array<Vec3, 5> jitterSensorPoints{{
    {30, -30*std::tan(22*d), -30*std::tan(15*d)},
    {30,  30*std::tan(22*d), -30*std::tan(15*d)},
    {30, -30*std::tan(22*d),  30*std::tan(15*d)},
    {30,  30*std::tan(22*d),  30*std::tan(15*d)},
    {30, 0, 0}}};
  constexpr int jitterFrames = 240;
  for (int i = 0; i < jitterFrames; ++i)
  {
    const auto projection = ProjectTruthTarget(
      9, "jitter_aircraft", jitterSensorPoints[i % jitterSensorPoints.size()],
      320, 240, M_PI/3, .1, 100);
    Check(projection.has_value(), "大幅抖动期间飞机始终位于相机视野");
    const auto observation = MakeTruthObservation(
      *projection, {128, 96, 64, 48}, 30, 320, 240, 800+i);
    const double now = i / 30.0;
    const auto output = jitterControl.Step(
      now, true, {}, 320, 240, {}, now, 800+i, &observation);
    CheckRate(output);
    Check(jitterControl.track_state() == TrackState::TRACK &&
          jitterControl.target_valid() &&
          jitterControl.target_miss().frame_seq == 800+i,
          "目标跨四象限快速抖动时真值跟踪必须逐帧保持有效");
  }
  Check(jitterControl.valid_frames() == jitterFrames &&
        jitterControl.invalid_frames() == 0,
        "大幅画面抖动不产生丢帧或错误跟踪计数");
  cv::Mat im(240, 320, CV_8UC3);
  cv::RNG rng(42);
  rng.fill(im, cv::RNG::UNIFORM, 0, 255);
  for (int sign : {-1, 1})
  {
    GimbalControl directionTrack;
    const double x = sign > 0 ? .7 : .1;
    Check(directionTrack.StartTracking(im, {x, .4, .2, .2}, 320, 240).first,
          "左右方向跟踪使用真实CSRT初始化");
    const auto output = directionTrack.Step(0, true, im, 320, 240, {}, 0, 400+sign);
    Check(directionTrack.target_valid() && sign*output[0] > 0,
          "真实CSRT画面右侧目标驱动正yaw、左侧目标驱动负yaw");
  }
  for (bool compensate : {false, true})
  {
    GimbalControl limitedTrack;
    ControlConfig trackingConfig;
    trackingConfig.max_rate_dps = 100;
    if (compensate)
      trackingConfig.mount_angular_velocity = {4, -5, 6};
    limitedTrack.Configure(trackingConfig);
    const JointSample trackingJoint{.3, -70*d, 0, 0};
    const auto feed = StabilizationRates(trackingJoint.yaw, trackingJoint.pitch,
                                         trackingConfig.mount_angular_velocity);
    if (compensate)
      Check(std::abs(feed[0]) > 20 && std::abs(feed[1]) > 20,
            "测试载机补偿在两轴均超过20度每秒");
    Check(limitedTrack.StartTracking(im, compensate ? std::array<double, 4>{.4, .4, .2, .2} :
          std::array<double, 4>{.75, .05, .2, .2}, 320, 240).first,
          "限速跟踪使用真实CSRT初始化");
    std::array<double, 2> prev{};
    for (int i = 0; i <= 30; ++i)
    {
      const auto output = limitedTrack.Step(i*.01, i == 0, im, 320, 240,
                                            trackingJoint, i*.01, 500);
      CheckRate(output);
      for (int axis = 0; axis < 2; ++axis)
        Check(std::abs(output[axis]-prev[axis]) <= 160*.01 + 1e-9,
              "图像伺服与载机补偿合成后仍保留加速度平滑");
      prev = output;
    }
    Check(limitedTrack.target_valid() && limitedTrack.saturated() &&
          std::abs(limitedTrack.cmd()[0]) == 20 && std::abs(limitedTrack.cmd()[1]) == 20 &&
          limitedTrack.client_message().find("角速度受限") != std::string::npos,
          "图像误差或载机补偿触顶后保持跟踪并明确报告角速度受限");
    trackingConfig.max_rate_dps = 3;
    limitedTrack.Configure(trackingConfig);
    CheckRate(limitedTrack.cmd(), 3);
    CheckRate(limitedTrack.Step(.3, false, im, 320, 240, trackingJoint, .3), 3);
  }
  Check(c.StartTracking(im, {.3, .3, .25, .25}, 320, 240).first, "CSRT选框");
  Check(!c.target_miss().valid, "仅选框初始化不宣称有效脱靶量");
  c.Step(1, true, im, 320, 240, {}, 30, 42);
  Check(c.track_state() == TrackState::TRACK && c.target_valid(), "CSRT持续跟踪");
  miss = c.target_miss();
  Check(miss.valid && miss.frame_seq == 42 &&
        miss.center_px[0] == c.target_box().Cx() &&
        miss.center_px[1] == c.target_box().Cy() &&
        miss.offset_px[0] == c.target_box().Cx() - 160 &&
        miss.offset_px[1] == c.target_box().Cy() - 120,
        "实际跟踪成功后发布对应帧的目标中心与有向脱靶量");
  c.Step(1.01, false, im, 320, 240, {}, 30.01);
  Check(c.target_miss().valid && c.target_miss().frame_seq == 42,
        "两次相机帧之间保留未超时测量及原帧序号");
  Check(c.client_message().find("视线退化") == std::string::npos,
        "零位CSRT可正常进行两轴视线跟踪");
  cmd = c.Step(1.6, false, im, 320, 240, {}, 30.6);
  Check(cmd[0] == 0 && cmd[1] == 0 && !c.target_valid(), "断帧立即停止");
  Check(!c.target_miss().valid && c.target_miss().frame_seq == -1,
        "断帧超时废弃脱靶量及关联帧");
  c.Step(4.2, false, im, 320, 240, {}, 33.2);
  Check(c.track_state() == TrackState::LOST, "长期断帧进入丢失");
  Check(!c.target_miss().valid, "丢失状态没有有效脱靶量");
  c.Stop();
  Check(c.client_message().empty(), "停止后清除已失效的跟踪约束提示");
  Check(c.StartTracking(im, {.65, .35, .2, .2}, 320, 240).first,
        "机械边界图像选框有效");
  JointSample trackedLimit{59.9*d, -45*d, 0, 0};
  c.Step(4.3, true, im, 320, 240, trackedLimit, 33.3);
  Check(c.track_state() == TrackState::TRACK && c.limit_flags()[0] &&
        c.client_message().find("机械限位") != std::string::npos &&
        c.stability() == Stability::UNSTABLE,
        "图像识别正常但机械不可继续向外时不能宣称稳定锁定");
  c.Stop();
  Check(!c.target_miss().valid, "停止跟踪清除脱靶量");
  // 额外放宽测试配置只用于覆盖数学奇异保护；组件确认范围不含该姿态。
  GimbalControl singularTrack;
  ControlConfig singularConfig;
  singularConfig.pitch_min_deg = -95;
  singularTrack.Configure(singularConfig);
  Check(singularTrack.StartTracking(im, {.3, .3, .25, .25}, 320, 240).first,
        "退化保护跟踪初始化");
  singularTrack.Step(0, true, im, 320, 240, {0, -90*d, 0, 0}, 0);
  Check(singularTrack.track_state() == TrackState::TRACK &&
        singularTrack.client_message().find("视线退化") != std::string::npos &&
        singularTrack.stability() == Stability::UNDETERMINED,
        "视线平行外轴时保持跟踪识别但明确报告退化");
  GimbalControl hidden;
  Check(hidden.StartTracking(im, {.3, .3, .25, .25}, 320, 240).first,
        "遮挡测试初始化");
  hidden.Step(0, true, im, 320, 240, {}, 0, 100);
  Check(hidden.target_miss().valid, "遮挡前测量有效");
  cv::Mat blank(im.size(), im.type(), cv::Scalar::all(0));
  hidden.Step(.04, true, blank, 320, 240, {}, .04, 101);
  Check(hidden.track_state() == TrackState::TEMP_LOST && !hidden.target_miss().valid,
        "新帧跟踪失败立即废弃脱靶量，不等待断帧看门狗");
  hidden.Step(.08, true, im, 320, 240, {}, .08, 102);
  Check(hidden.target_miss().valid && hidden.target_miss().frame_seq == 102,
        "重捕获后以新帧恢复有效脱靶量");
  hidden.Manual(20, 0, .1);
  Check(!hidden.target_miss().valid, "人工转动取消跟踪并清除脱靶量");
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
