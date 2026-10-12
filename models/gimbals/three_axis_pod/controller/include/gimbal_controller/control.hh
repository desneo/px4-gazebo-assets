// 目标跟踪控制、状态机与稳定性判定。
//
// 控制律：图像游标误差 -> 目标方位/俯仰角 -> PI(+D) 速度指令；
// 速度/加速度限幅，关节限位与饱和上报。
#ifndef GIMBAL_CONTROLLER_CONTROL_HH_
#define GIMBAL_CONTROLLER_CONTROL_HH_

#include <array>
#include <deque>
#include <string>
#include <utility>

#include <opencv2/core.hpp>

#include "gimbal_controller/tracker.hh"

namespace gimbal {

enum class TrackState
{
  IDLE,
  INIT,
  TRACK,
  TEMP_LOST,
  LOST,
  FAULT
};

enum class Stability
{
  UNDETERMINED,
  STABLE,
  UNSTABLE
};

enum class Mode
{
  IDLE,
  TRACK,
  MANUAL,
  HOME
};

const char *ToString(TrackState _s);
const char *ToString(Stability _s);
const char *ToString(Mode _m);

struct ControlConfig
{
  double max_rate_dps{100.0};
  double max_acc_dps2{160.0};
  double kp{4.0};
  double ki{1.6};
  double kd{0.10};
  double ff{1.0};
  double forward_az_deg{0.0};
  double integral_limit{25.0};
  double pitch_min_deg{-135.0};
  double pitch_max_deg{45.0};
  double roll_min_deg{-45.0};
  double roll_max_deg{45.0};
  double limit_margin_deg{1.0};
  double sat_vel_err_dps{25.0};
  double sat_hold_s{0.2};
  double integral_freeze_deg{15.0};
  double manual_timeout_s{0.5};
  double frame_stop_timeout_s{0.5};
  double temp_lost_timeout_s{2.0};
  double stable_enter_s{1.0};
  double stable_rms_deg{1.0};
  double unstable_rms_deg{2.0};
};

/// 关节采样（弧度 / 弧度每秒）。顺序固定：yaw, pitch, roll。
struct JointSample
{
  double yaw{0.0};
  double pitch{0.0};
  double roll{0.0};
  double yaw_vel{0.0};
  double pitch_vel{0.0};
  double roll_vel{0.0};
};

class GimbalControl
{
 public:
  GimbalControl();
  explicit GimbalControl(const ControlConfig &_cfg);

  ControlConfig &cfg() { return cfg_; }
  void Configure(const ControlConfig &_cfg) { cfg_ = _cfg; }

  /// box_norm: (x, y, w, h) 归一化到 0..1。返回 (ok, message)。
  std::pair<bool, std::string> StartTracking(const cv::Mat &_bgr,
                                             const std::array<double, 4> &_box,
                                             int _width, int _height);
  void Stop();
  /// 超时保护立即清除输出，不经过加速度平滑。
  void WatchdogStop();
  void Manual(double _yawDps, double _pitchDps, double _rollDps);
  /// _now 与 Step 的 watchdog 时间使用相同时间源。
  void Manual(double _yawDps, double _pitchDps, double _rollDps, double _now);
  void GoHome();
  void GoLevelForward();
  void ResetFault();

  /// 执行一步控制；返回三轴速度指令（°/s，顺序 yaw, pitch, roll）。
  /// _now 用于仿真伺服；_watchdogNow 用于超时保护，省略时使用 _now。
  std::array<double, 3> Step(double _now, bool _newFrame, const cv::Mat &_bgr,
                             int _width, int _height, const JointSample &_js,
                             double _watchdogNow = -1.0);

  Mode mode() const { return mode_; }
  TrackState track_state() const { return track_state_; }
  Stability stability() const { return stability_; }
  const std::string &fault_reason() const { return fault_reason_; }
  const std::string &client_message() const { return client_message_; }
  bool target_valid() const { return target_valid_; }
  bool has_target_box() const { return has_box_; }
  const Box &target_box() const { return target_box_; }
  double center_err_deg() const { return center_err_deg_; }
  /// 顺序：yaw, pitch, roll
  const std::array<bool, 3> &limit_flags() const { return limit_flags_; }
  bool saturated() const { return saturated_; }
  const std::array<double, 3> &cmd() const { return cmd_; }
  long valid_frames() const { return valid_frames_; }
  long invalid_frames() const { return invalid_frames_; }

 private:
  static double Clamp(double _v, double _lo, double _hi);
  double Slew(double _target, double _prev, double _dt) const;
  void ResetFilters();
  void UpdateMeasurement(double _now, const cv::Mat &_bgr, int _width,
                         int _height, double _yaw, double _roll,
                         double _pitch, double _watchdogNow);
  bool TryReacquire(const cv::Mat &_bgr);
  void TargetCmdFromTracking();
  void TargetCmdHome(double _yaw, double _pitch, double _roll);
  void ApplyLimits(std::array<double, 3> &_command, double _rollRad,
                   double _pitchRad, double _now);
  void UpdateSaturation(const JointSample &_js, double _now);
  void UpdateStability(double _now);

  ControlConfig cfg_;
  Mode mode_{Mode::IDLE};
  TrackState track_state_{TrackState::IDLE};
  Stability stability_{Stability::UNDETERMINED};
  std::string fault_reason_;
  std::string client_message_;

  Tracker tracker_;
  Box target_box_;
  bool has_box_{false};
  bool target_valid_{false};
  double measure_az_{0.0};
  double measure_el_{0.0};
  double center_err_deg_{0.0};

  std::array<double, 3> home_target_{{0.0, 0.0, 0.0}};  // yaw, pitch, roll
  bool has_lost_since_{false};
  double lost_since_{0.0};
  bool has_init_started_{false};
  double init_started_{0.0};
  bool has_frame_time_{false};
  double frame_time_{0.0};
  bool has_manual_time_{false};
  double manual_time_{0.0};
  bool has_stable_since_{false};
  double stable_since_{0.0};

  std::deque<std::pair<double, double>> err_window_;

  std::array<double, 3> cmd_{{0.0, 0.0, 0.0}};      // yaw, pitch, roll
  std::array<double, 3> raw_cmd_{{0.0, 0.0, 0.0}};
  std::array<double, 3> manual_cmd_{{0.0, 0.0, 0.0}};
  double last_cmd_t_{0.0};
  bool has_last_cmd_t_{false};
  bool saturated_{false};
  bool has_sat_since_{false};
  double sat_since_{0.0};
  std::array<bool, 3> limit_flags_{{false, false, false}};
  std::array<double, 3> limit_until_{{0.0, 0.0, 0.0}};  // yaw, pitch, roll

  std::array<double, 2> integral_{{0.0, 0.0}};   // az, el
  std::array<double, 2> last_err_{{0.0, 0.0}};
  std::array<double, 2> last_meas_{{0.0, 0.0}};
  std::array<double, 2> meas_rate_{{0.0, 0.0}};
  bool has_last_meas_t_{false};
  double last_meas_t_{0.0};
  bool has_last_err_t_{false};
  double last_err_t_{0.0};
  bool has_frame_seq_{false};
  long last_frame_seq_{0};

  std::array<double, 3> ff_cmd_{{0.0, 0.0, 0.0}};
  bool has_ff_cmd_{false};

  long valid_frames_{0};
  long invalid_frames_{0};
};

}  // namespace gimbal

#endif  // GIMBAL_CONTROLLER_CONTROL_HH_
