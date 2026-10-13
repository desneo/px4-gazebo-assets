#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "gimbal_controller/control.hh"
#include "gimbal_controller/kinematics.hh"
#include "gimbal_controller/mini_json.hh"

namespace {
void Require(bool condition, const char *message)
{
  if (!condition)
    throw std::runtime_error(message);
}

cv::Mat TargetImage()
{
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat roi = image(cv::Rect(220, 100, 32, 32));
  cv::RNG rng(12345);
  rng.fill(roi, cv::RNG::UNIFORM, 0, 256);
  return image;
}

void MountDirections()
{
  const auto sensor = gimbal::CamSensorRot();
  const auto expected = gimbal::MatMul(gimbal::Ry(-M_PI / 2), gimbal::Rx(M_PI));
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      Require(std::abs(sensor.m[row][col] - expected.m[row][col]) < 1e-12,
              "sensor rotation matches SDF rpy and has no reflection");
  const auto q = gimbal::QuatFromR(sensor);
  Require(std::abs(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]-1) < 1e-12,
          "sensor quaternion is unit length");
  const auto right = gimbal::PixelToSensorRay(240, 120, 320, 240);
  Require(right.y < 0 && gimbal::MatVec(sensor, right).y > 0,
          "image right ray retains physical direction after sensor reflection correction");
  const cv::Mat empty;
  constexpr double d2r = M_PI / 180.0;
  for (const auto &rotation : {gimbal::Identity(), gimbal::Rx(M_PI),
      gimbal::Ry(M_PI / 2.0), gimbal::Rz(.7)})
  {
    gimbal::ControlConfig cfg;
    cfg.mount_world_rotation = rotation;
    if(rotation.m[2][2]<-.99) cfg.sensor_rotation=gimbal::Ry(-M_PI/2);
    cfg.forward_az_deg = 37.0;
    gimbal::GimbalControl control(cfg);
    gimbal::JointSample js;
    double now = 0.0;
    auto settle = [&] {
      for (int i = 0; i < 2500; ++i)
      {
        const auto cmd = control.Step(now, false, empty, 0, 0, js);
        js.yaw += cmd[0] * d2r * .01;
        js.pitch += cmd[1] * d2r * .01;
        js.roll += cmd[2] * d2r * .01;
        now += .01;
      }
      Require(control.mode() == gimbal::Mode::IDLE,
              "static installation setpoint completes without limit deadlock");
      return gimbal::MatVec(rotation, gimbal::LosMount(js.yaw, js.roll, js.pitch));
    };
    const bool levelAccepted=control.GoLevelForward();
    auto los = settle();
    Require(!levelAccepted || gimbal::AngleBetween(los, {std::cos(37 * d2r), std::sin(37 * d2r), 0}) < .6 * d2r,
            "reset points world forward for rotated static installations");
    control.GoHome();
    los = settle();
    if (rotation.m[2][2] < -.99)
    {
      Require(los.z > .68 && los.z < .71,
              "downward home observes reachable sky without claiming zenith");
      Require(js.pitch >= (cfg.pitch_min_deg + cfg.limit_margin_deg) * d2r,
              "downward home respects pitch safety margin");
    }
    else
      Require(gimbal::AngleBetween(los, {0, 0, 1}) < .6 * d2r,
              "skyward home points world zenith");
  }
}

void AxisReset()
{
  constexpr double d2r=M_PI/180.;
  const cv::Mat empty;
  for(bool downward : {false,true})
  {
    gimbal::ControlConfig cfg;
    cfg.mount_world_rotation=gimbal::MatMul(gimbal::Rz(.3),downward?gimbal::Rx(M_PI):gimbal::Identity());
    cfg.sensor_rotation=downward?gimbal::Ry(-M_PI/2):gimbal::CamSensorRot();
    cfg.forward_az_deg=.4/d2r;
    for(const std::string axis:{"yaw","pitch","roll"})
    {
      gimbal::GimbalControl control(cfg);
      gimbal::JointSample js;
      js.yaw=-2.5+6*M_PI;js.pitch=-1.15;js.roll=.3;
      auto rotation=[&]{return gimbal::PublicRotation(js.yaw,js.roll,js.pitch,
        cfg.mount_world_rotation,cfg.sensor_rotation,.4);};
      const auto initial=gimbal::PublicAngles(rotation());
      const int selected=axis=="yaw"?0:axis=="pitch"?1:2;
      control.Manual(2,-2,2,0);
      control.Step(0,false,empty,0,0,js);
      Require(!control.ResetAxis("invalid",js) && control.mode()==gimbal::Mode::MANUAL,
        "invalid public axis preserves control mode");
      Require(control.ResetAxis(axis,js),"reachable public axis reset accepted for both optical installations");
      auto previous=control.cmd();
      for(int i=1;i<=2500;++i)
      {
        auto command=control.Step(i*.01,false,empty,0,0,js);
        for(int j=0;j<3;++j)
        {
          Require(std::abs(command[j])<=100.00001,"public reset respects mechanical speed limit");
          Require(std::abs(command[j]-previous[j])<=1.60001,"public reset respects mechanical acceleration limit");
        }
        previous=command;
        js.yaw+=command[0]*d2r*.01;js.pitch+=command[1]*d2r*.01;js.roll+=command[2]*d2r*.01;
        const auto actual=gimbal::PublicAngles(rotation());
        for(int j=0;j<3;++j) if(j!=selected)
          Require(std::abs(gimbal::WrapPi(actual[j]-initial[j]))<1.*d2r,
            "nonselected public axes held throughout coupled reset trajectory");
      }
      Require(control.mode()==gimbal::Mode::IDLE,"public axis reset completes");
      const auto final=gimbal::PublicAngles(rotation());
      Require(std::abs(final[selected])<.6*d2r,"selected public axis reaches zero");
    }
    const double y=-2.5,p=-1.15,r=.3;
    const std::array<double,3> requested{.13,-.11,.17};
    std::array<double,3> jointRates,roundTrip;
    Require(gimbal::PublicRatesToJoint(y,r,p,cfg.mount_world_rotation,cfg.sensor_rotation,.4,requested,jointRates),
      "public Euler rate maps to all mechanical axes");
    Require(gimbal::JointRatesToPublic(y,r,p,cfg.mount_world_rotation,cfg.sensor_rotation,.4,jointRates,roundTrip),
      "mechanical rates map back to public Euler rates");
    for(int i=0;i<3;++i) Require(std::abs(roundTrip[i]-requested[i])<1e-10,"coupled rate Jacobian round trip");
    const auto before=gimbal::PublicAngles(gimbal::PublicRotation(y,r,p,cfg.mount_world_rotation,cfg.sensor_rotation,.4));
    const double dt=1e-6;
    const auto after=gimbal::PublicAngles(gimbal::PublicRotation(y+jointRates[0]*dt,r+jointRates[2]*dt,
      p+jointRates[1]*dt,cfg.mount_world_rotation,cfg.sensor_rotation,.4));
    for(int i=0;i<3;++i) Require(std::abs(gimbal::WrapPi(after[i]-before[i])/dt-requested[i])<1e-6,
      "public manual rates match independent numerical attitude derivative");
  }
  gimbal::GimbalControl singular;
  Require(!singular.ManualPublic(1,0,0,{}) && singular.mode()==gimbal::Mode::IDLE,
    "zenith yaw rate is rejected and stopped");
  Require(!singular.ResetAxis("roll",{}),"zenith roll reset rejected");
  Require(singular.ManualPublic(0,-10,0,{}),"pitch rate can leave zenith singularity");
  gimbal::JointSample level;level.yaw=M_PI;level.pitch=-M_PI/2;
  const auto image=TargetImage();
  singular.StartTracking(image,{220./320,100./240,32./320,32./240},320,240);
  Require(singular.ResetAxis("pitch",level) && !singular.has_target_box(),"public reset cancels tracking");
  gimbal::ControlConfig uncalibrated;
  uncalibrated.mount_world_rotation=gimbal::Rx(M_PI);
  gimbal::GimbalControl upsideDown(uncalibrated);
  Require(!upsideDown.GoLevelForward(level),"uncalibrated downward image roll180 cannot falsely reset to public zero");
  Require(!upsideDown.ResetAxis("roll",level),"unreachable optical roll zero rejected");
}

void Smoke()
{
  AxisReset();
  MountDirections();
  for (const char *number : {"-", "+1", "01", "1.", "1e", "1e+", "30-99", "1e999"})
  {
    gimbal::json::Value value;
    Require(!gimbal::json::Parse(std::string("{\"yaw\":") + number + "}", value),
            "invalid numeric command rejected");
  }
  gimbal::json::Value numeric;
  Require(gimbal::json::Parse("{\"yaw\":-1.25e+2}", numeric) &&
          numeric.Find("yaw")->AsNum() == -125, "valid numeric command parsed");
  const cv::Mat empty;
  gimbal::JointSample js;
  gimbal::GimbalControl control;
  auto command = control.Step(0, false, empty, 0, 0, js);
  Require(command == std::array<double, 3>{0, 0, 0}, "idle command");
  control.Manual(200, 0, 0);
  command = control.Step(.01, false, empty, 0, 0, js);
  Require(std::abs(command[0] - 1.6) < 1e-6, "manual acceleration limit");
  for (int i = 2; i <= 100; ++i)
  {
    control.Manual(200, 0, 0, i * .01);
    command = control.Step(i * .01, false, empty, 0, 0, js);
  }
  Require(std::abs(command[0] - 100) < 1e-6, "manual speed limit");
  control.Stop();
  for (int i = 101; i <= 200; ++i)
    command = control.Step(i * .01, false, empty, 0, 0, js);
  Require(command[0] == 0 && control.mode() == gimbal::Mode::IDLE,
          "stop deceleration");

  gimbal::GimbalControl limited;
  js.pitch = js.roll = M_PI / 4;
  limited.Manual(0, 30, 30);
  command = limited.Step(0, false, empty, 0, 0, js);
  Require(command[1] == 0 && command[2] == 0, "limit prevents starting outward");
  Require(limited.limit_flags()[1] && limited.limit_flags()[2], "limit reporting");
  limited.Manual(0, -30, -30);
  command = limited.Step(.01, false, empty, 0, 0, js);
  Require(command[1] < 0 && command[2] < 0, "limit permits inward motion");

  gimbal::GimbalControl home;
  home.GoHome();
  command = home.Step(0, false, empty, 0, 0, {});
  Require(command == std::array<double, 3>{0, 0, 0} &&
              home.mode() == gimbal::Mode::IDLE, "home completion");
  home.GoHome();
  gimbal::JointSample wrapped_home;
  wrapped_home.yaw = -2 * M_PI;
  command = home.Step(.01, false, empty, 0, 0, wrapped_home);
  Require(command == std::array<double, 3>{0, 0, 0} &&
              home.mode() == gimbal::Mode::IDLE, "home accepts full-turn yaw");
  const auto bad = home.StartTracking(cv::Mat(240, 320, CV_8UC3, cv::Scalar(0)),
                                      {.1, .1, .1, .1}, 320, 240);
  Require(!bad.first && home.track_state() == gimbal::TrackState::FAULT,
          "low texture rejection");
  home.ResetFault();
  Require(home.track_state() == gimbal::TrackState::IDLE, "fault reset");

  const auto image = TargetImage();
  const auto started = home.StartTracking(image,
      {220. / 320, 100. / 240, 32. / 320, 32. / 240}, 320, 240);
  Require(started.first, "textured target initialization");
  home.Step(0, true, image, 320, 240, {});
  Require(home.track_state() == gimbal::TrackState::TRACK && home.target_valid(),
          "textured target tracking");
  const auto ray = gimbal::PixelToSensorRay(160, 120, 320, 240);
  Require(std::abs(ray.x - 1) < 1e-12 && ray.y == 0 && ray.z == 0,
          "image center ray");
  std::cout << "PASS: manual speed/acceleration, stop, joint limits from rest, "
               "home, fault reset, target initialization/tracking, center ray\n";
}

void Regressions()
{
  const cv::Mat empty;
  const std::array<double, 3> zero{0, 0, 0};
  gimbal::GimbalControl manual;
  manual.Manual(30, 30, 30, 0);
  for (int i = 0; i < 50; ++i)
    manual.Step(i * .01, false, empty, 0, 0, {});
  Require(manual.cmd()[0] > 0, "manual active before timeout");
  Require(manual.Step(.5, false, empty, 0, 0, {}) == zero &&
      manual.mode() == gimbal::Mode::IDLE, "manual stops exactly at 0.5s");
  manual.Manual(30, 30, 30, .6);
  Require(manual.Step(.6, false, empty, 0, 0, {})[0] > 0,
          "manual recovers with fresh command");
  for (int i = 1; i <= 20; ++i)
  {
    const double now = .6 + .2 * i;
    manual.Manual(30, 30, 30, now);
    Require(manual.Step(now, false, empty, 0, 0, {})[0] > 0,
            "periodic manual commands renew timeout");
  }
  manual.WatchdogStop();
  Require(manual.cmd() == zero && manual.mode() == gimbal::Mode::IDLE,
          "wall-clock watchdog clears current output immediately");
  manual.Manual(30, 30, 30, 5);
  manual.Step(5, false, empty, 0, 0, {});
  for (int i = 1; i <= 20; ++i)
    manual.Step(5 + i * .01, false, empty, 0, 0, {});
  gimbal::JointSample js;
  js.pitch = js.roll = M_PI / 4;
  const auto upper = manual.Step(5.21, false, empty, 0, 0, js);
  Require(upper[1] == 0 && upper[2] == 0, "upper limit clears outward velocity");
  manual.Manual(0, -30, -30, 5.22);
  Require(manual.Step(5.22, false, empty, 0, 0, js)[1] < 0,
          "upper limit permits inward recovery");
  for (int i = 23; i <= 40; ++i)
    manual.Step(5 + i * .01, false, empty, 0, 0, {});
  js.pitch = -135 * M_PI / 180;
  js.roll = -M_PI / 4;
  const auto lower = manual.Step(5.41, false, empty, 0, 0, js);
  Require(lower[1] == 0 && lower[2] == 0, "lower limit clears outward velocity");
  manual.Manual(0, 30, 30, 5.42);
  Require(manual.Step(5.42, false, empty, 0, 0, js)[1] > 0,
          "lower limit permits inward recovery");

  const auto image = TargetImage();
  const auto start = [&](gimbal::GimbalControl &control) {
    Require(control.StartTracking(image,
        {220. / 320, 100. / 240, 32. / 320, 32. / 240}, 320, 240).first,
        "target initialization");
  };
  gimbal::GimbalControl tracking;
  start(tracking);
  tracking.Step(0, true, image, 320, 240, {});
  tracking.Step(.49, false, image, 320, 240, {});
  Require(tracking.track_state() == gimbal::TrackState::TRACK &&
          tracking.target_valid() && std::abs(tracking.cmd()[0]) > 0,
          "video remains active before freshness deadline");
  Require(tracking.Step(.5, false, image, 320, 240, {}) == zero &&
          tracking.track_state() == gimbal::TrackState::TEMP_LOST &&
          !tracking.target_valid() &&
          tracking.stability() == gimbal::Stability::UNDETERMINED,
          "video stops exactly at 0.5s and invalidates target");
  tracking.Step(1.99, false, image, 320, 240, {});
  Require(tracking.track_state() == gimbal::TrackState::TEMP_LOST,
          "video temporary loss before 2s");
  tracking.Step(2, false, image, 320, 240, {});
  Require(tracking.track_state() == gimbal::TrackState::LOST &&
          tracking.cmd() == zero, "video LOST exactly 2s after last frame");
  tracking.Step(2.01, true, image, 320, 240, {});
  Require(tracking.track_state() == gimbal::TrackState::LOST &&
          !tracking.target_valid(), "LOST requires new selection");

  start(tracking);
  Require(!tracking.target_valid() && !tracking.has_target_box() &&
          tracking.cmd() == zero, "new selection clears previous tracking state");
  tracking.Step(3, true, image, 320, 240, {});
  tracking.Step(3.5, false, image, 320, 240, {});
  tracking.Step(3.6, true, image, 320, 240, {});
  Require(tracking.track_state() == gimbal::TrackState::TRACK &&
          tracking.target_valid(), "fresh frame recovers temporary video loss");
  tracking.Step(4.09, false, image, 320, 240, {});
  Require(tracking.track_state() == gimbal::TrackState::TRACK,
          "recovery renews freshness deadline");
  Require(tracking.Step(4.11, false, image, 320, 240, {}) == zero,
          "renewed freshness deadline stops output");

  gimbal::GimbalControl init;
  start(init);
  init.Step(10, false, empty, 320, 240, {});
  Require(init.track_state() == gimbal::TrackState::INIT,
          "selection waits for first fresh frame");
  Require(init.Step(10.5, false, empty, 320, 240, {}) == zero &&
          init.track_state() == gimbal::TrackState::TEMP_LOST,
          "INIT video loss stops at 0.5s");
  init.Step(12, false, empty, 320, 240, {});
  Require(init.track_state() == gimbal::TrackState::LOST,
          "INIT without frames becomes LOST at 2s");
  gimbal::GimbalControl independent_clock;
  independent_clock.Manual(30, 0, 0, 100);
  independent_clock.Step(1, false, empty, 0, 0, {}, 100);
  independent_clock.Step(1.001, false, empty, 0, 0, {}, 100.49);
  Require(independent_clock.cmd()[0] > 0,
          "wall-clock manual command active before deadline");
  Require(independent_clock.Step(1.002, false, empty, 0, 0, {}, 100.5) == zero,
          "manual watchdog independent of simulation speed");
  start(independent_clock);
  independent_clock.Step(2, true, image, 320, 240, {}, 101);
  independent_clock.Step(2.001, false, empty, 320, 240, {}, 101.5);
  Require(independent_clock.track_state() == gimbal::TrackState::TEMP_LOST &&
          independent_clock.cmd() == zero, "video watchdog uses wall-clock age");
  independent_clock.Step(2.002, false, empty, 320, 240, {}, 103);
  Require(independent_clock.track_state() == gimbal::TrackState::LOST,
          "video LOST independent of simulation speed");

  const cv::Mat blank(240, 320, CV_8UC3, cv::Scalar(0));
  gimbal::GimbalControl invalid;
  start(invalid);
  invalid.Step(0, true, image, 320, 240, {});
  invalid.Step(.1, true, blank, 320, 240, {});
  Require(invalid.track_state() == gimbal::TrackState::TEMP_LOST &&
          !invalid.target_valid(), "failed tracking invalidates target");
  invalid.Step(.2, true, image, 320, 240, {});
  Require(invalid.track_state() == gimbal::TrackState::TRACK &&
          invalid.target_valid() && invalid.has_target_box(),
          "reacquisition marks valid and computes a current command");

  // 视频持续到达，目标连续遮挡 60 帧后在新位置重现，两次遮挡独立计时。
  gimbal::GimbalControl occluded;
  start(occluded);
  occluded.Step(0, true, image, 320, 240, {});
  for (int cycle = 0; cycle < 2; ++cycle)
  {
    const double begin = .1 + cycle * 2.2;
    for (int frame = 0; frame < 60; ++frame)
    {
      const double now = begin + frame / 30.;
      occluded.Step(now, true, blank, 320, 240, {});
      Require(occluded.track_state() == gimbal::TrackState::TEMP_LOST &&
              !occluded.target_valid(), "2s occlusion retains recovery window");
      if (frame >= 30)
        Require(occluded.cmd() == zero, "occluded target stops servo output");
    }
    cv::Mat moved(240, 320, CV_8UC3, cv::Scalar(0));
    const int x = cycle == 0 ? 160 : 200;
    image(cv::Rect(220, 100, 32, 32)).copyTo(moved(cv::Rect(x, 100, 32, 32)));
    occluded.Step(begin + 2, true, moved, 320, 240, {});
    Require(occluded.track_state() == gimbal::TrackState::TRACK &&
            occluded.target_valid() &&
            std::abs(occluded.target_box().Cx() - (x + 16)) <= 3,
            "target automatically reacquired at new position after full 2s");
  }

  gimbal::GimbalControl expired;
  start(expired);
  expired.Step(0, true, image, 320, 240, {});
  for (int frame = 0; frame < 75; ++frame)
    expired.Step(.1 + frame / 30., true, blank, 320, 240, {});
  Require(expired.track_state() == gimbal::TrackState::TEMP_LOST,
          "occlusion remains temporary before 2.5s");
  expired.Step(2.6, true, image, 320, 240, {});
  Require(expired.track_state() == gimbal::TrackState::LOST &&
          !expired.target_valid() && expired.cmd() == zero,
          "recovery at expired occlusion deadline requires new selection");
  start(invalid);
  invalid.Step(3, true, blank, 320, 240, {});
  Require(invalid.track_state() == gimbal::TrackState::INIT,
          "invalid fresh INIT frame keeps initialization state");
  invalid.Step(4.99, true, blank, 320, 240, {});
  invalid.Step(5, true, blank, 320, 240, {});
  Require(invalid.track_state() == gimbal::TrackState::LOST &&
          invalid.cmd() == zero, "INIT continuous invalid frames expire at 2s");

  for (int action = 0; action < 4; ++action)
  {
    gimbal::GimbalControl transition;
    start(transition);
    transition.Step(0, true, image, 320, 240, {});
    Require(transition.has_target_box(), "tracking exposes a current box");
    switch (action)
    {
      case 0: transition.Stop(); break;
      case 1: transition.Manual(10, 0, 0); break;
      case 2: transition.GoHome(); break;
      case 3: transition.GoLevelForward(); break;
    }
    Require(!transition.target_valid() && !transition.has_target_box() &&
            transition.stability() == gimbal::Stability::UNDETERMINED &&
            transition.client_message().empty() && transition.fault_reason().empty(),
            "mode transitions clear old target, stability and messages");
  }
  gimbal::GimbalControl fault;
  Require(!fault.StartTracking(blank, {.1, .1, .1, .1}, 320, 240).first &&
          !fault.client_message().empty(), "fault has actionable message");
  fault.ResetFault();
  Require(fault.track_state() == gimbal::TrackState::IDLE &&
          fault.client_message().empty() && fault.fault_reason().empty(),
          "fault reset clears old messages");

  std::cout << "PASS: manual watchdog, renewal/recovery, moving joint limits, "
               "video timeout boundaries, repeated 2s occlusion recovery, "
               "occlusion expiry and INIT loss\n";
}
}  // namespace

int main(int argc, char **argv)
{
  try
  {
    if (argc == 2 && std::string(argv[1]) == "--regressions")
      Regressions();
    else
      Smoke();
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
