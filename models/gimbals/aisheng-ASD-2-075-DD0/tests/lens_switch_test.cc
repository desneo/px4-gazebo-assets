#include <cassert>
#include <cmath>
#include <limits>
#include "two_axis/control.hh"
#include "two_axis/lens_switch.hh"

int main()
{
  using namespace two_axis;
  assert(!ShouldSwitchToWide(true, true, {0, 0, 500, 300}));
  assert(ShouldSwitchToWide(true, true, {0, 0, 501, 200}));
  assert(ShouldSwitchToWide(true, true, {0, 0, 200, 301}));
  assert(!ShouldSwitchToWide(false, true, {0, 0, 501, 301}));
  assert(!ShouldSwitchToWide(true, false, {0, 0, 501, 301}));
  assert(!ShouldSwitchToWide(true, true,
    {0, 0, std::numeric_limits<double>::quiet_NaN(), 301}));

  GimbalControl control;
  control.Configure(ControlConfig{});
  assert(!control.WaitForCameraFrame());
  assert(control.StartTruthTracking({.45, .45, .1, .1}, 1920, 1080).first);
  TruthProjection target{123, "target_aircraft", 960, 540, 1.0};
  const Box initial{0, 0, 80, 60};
  auto observation = MakeTruthObservation(target, initial, 6.5, 1920, 1080, 1);
  control.Step(1, true, {}, 1920, 1080, {}, 1, 1, &observation);
  assert(control.mode() == Mode::TRACK && control.target_valid());
  assert(ShouldSwitchToWide(true, true, control.target_box()));
  assert(control.WaitForCameraFrame());
  assert(control.mode() == Mode::TRACK && control.track_state() == TrackState::INIT);
  assert(!control.target_valid() && !control.has_target_box());
  const auto waiting = control.Step(1.01, false, {}, 0, 0, {}, 1.01);
  assert(waiting[0] == 0 && waiting[1] == 0);
  const double ratio = CameraFocalRatio(12.7 * M_PI / 180., 38 * M_PI / 180.);
  assert(ratio > 0 && ratio < 1);
  const auto roi = ReprojectTrackingBox({710, 390, 500, 300}, 1920, 1080, ratio);
  assert(std::abs(roi[0] + roi[2] / 2.0 - 0.5) < 1e-8);
  assert(std::abs(roi[1] + roi[3] / 2.0 - 0.5) < 1e-8);
  assert(std::abs(roi[2] - 500.0 * ratio / 1920) < 1e-8);
  observation = MakeTruthObservation(target,
    {0, 0, initial.w * ratio, initial.h * ratio}, 6.5, 1920, 1080, 2);
  control.Step(1.02, true, {}, 1920, 1080, {}, 1.02, 2, &observation);
  assert(control.mode() == Mode::TRACK && control.target_valid());
  assert(control.target_miss().frame_seq == 2);
  assert(std::abs(control.target_box().w - (80 * 6.5 * ratio + 10)) < 1e-8);
  assert(std::abs(control.target_box().h - (60 * 6.5 * ratio + 10)) < 1e-8);

  // CSRT 切换期间同样保持 TRACK 模式，宽角帧到达后可重建模板。
  cv::Mat image(240, 320, CV_8UC3);
  cv::randu(image, 0, 255);
  GimbalControl csrt;
  csrt.Configure(ControlConfig{});
  assert(csrt.StartTracking(image, {.25, .25, .5, .5}, 320, 240).first);
  assert(csrt.WaitForCameraFrame());
  assert(csrt.mode() == Mode::TRACK && !csrt.target_valid());
  assert(csrt.StartTracking(image, {.25, .25, .5, .5}, 320, 240).first);
}
