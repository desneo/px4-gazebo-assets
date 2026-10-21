#pragma once
#include <array>
#include <cstdint>
#include <string>
#include "gimbal_controller/tracker.hh"
#include "two_axis/kinematics.hh"
namespace two_axis
{
enum class Mode { IDLE, MANUAL, HOME, TRACK };
enum class TrackState { IDLE, INIT, TRACK, TEMP_LOST, LOST, FAULT };
enum class Stability { UNDETERMINED, STABLE, UNSTABLE };
const char *ToString(Mode);
const char *ToString(TrackState);
const char *ToString(Stability);
struct JointSample
{
  double yaw{}, pitch{}, yaw_vel{}, pitch_vel{};
};
struct TargetMiss
{
  bool valid{false};
  std::array<double, 2> center_px{}, offset_px{}, offset_deg{};
  std::int64_t frame_seq{-1};
};
// 画面左上角为原点，右/下为正；脱靶量相对画面中心。
TargetMiss MeasureTargetMiss(double cx, double cy, int width, int height);
struct ControlConfig
{
  double max_rate_dps{100}, max_acc_dps2{160}, kp{4};
  double yaw_min_deg{-60}, yaw_max_deg{60}, pitch_min_deg{-80}, pitch_max_deg{30};
  double manual_timeout_s{.5}, frame_stop_timeout_s{.5}, occlusion_timeout_s{2.5};
  Mat3 mount_world_rotation, sensor_rotation{SensorRotation()};
  Vec3 mount_angular_velocity;
};
class GimbalControl
{
public:
  void Configure(const ControlConfig &c)
  {
    cfg_ = c;
  }
  std::pair<bool, std::string> StartTracking(const cv::Mat &, const std::array<double, 4>&, int, int);
  void Stop();
  void WatchdogStop();
  void GoHome();
  void ResetFault();
  void Manual(double yaw, double pitch, double now);
  bool ResetAxis(const std::string &, const JointSample &);
  std::array<double, 2> Step(double, bool, const cv::Mat &, int, int, const JointSample &, double = -1, std::int64_t = -1);
  const TargetMiss &target_miss()const
  {
    return miss_;
  }
  Mode mode()const
  {
    return mode_;
  } TrackState track_state()const
  {
    return state_;
  }
  Stability stability()const
  {
    return stability_;
  }
  const std::string &fault_reason()const
  {
    return fault_;
  } const std::string &client_message()const
  {
    return message_;
  }
  bool target_valid()const
  {
    return valid_;
  } bool has_target_box()const
  {
    return has_box_;
  }
  const Box &target_box()const
  {
    return box_;
  } double center_err_deg()const
  {
    return error_;
  }
  const std::array<bool, 2>&limit_flags()const
  {
    return limits_;
  }
  bool saturated()const
  {
    return saturated_;
  } const std::array<double, 2>&cmd()const
  {
    return cmd_;
  }
  long valid_frames()const
  {
    return valid_frames_;
  } long invalid_frames()const
  {
    return invalid_frames_;
  }
private:
  ControlConfig cfg_;
  Mode mode_{Mode::IDLE};
  TrackState state_{TrackState::IDLE};
  Stability stability_{Stability::UNDETERMINED};
  Tracker tracker_;
  Box box_;
  TargetMiss miss_;
  bool has_box_{false}, valid_{false}, saturated_{false};
  std::string fault_, message_;
  std::array<double, 2> cmd_{}, manual_{}, target_{}, image_error_{};
  std::array<bool, 2> limits_{};
  double manual_time_{}, frame_time_{-1}, lost_time_{-1}, last_time_{-1}, stable_time_{-1}, error_{};
  long valid_frames_{}, invalid_frames_{};
};
}
