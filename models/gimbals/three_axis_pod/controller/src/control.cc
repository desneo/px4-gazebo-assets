#include "gimbal_controller/control.hh"

#include <algorithm>
#include <cmath>

#include "gimbal_controller/kinematics.hh"

namespace gimbal {

const char *ToString(TrackState _s)
{
  switch (_s)
  {
    case TrackState::IDLE: return "IDLE";
    case TrackState::INIT: return "INIT";
    case TrackState::TRACK: return "TRACK";
    case TrackState::TEMP_LOST: return "TEMP_LOST";
    case TrackState::LOST: return "LOST";
    case TrackState::FAULT: return "FAULT";
  }
  return "IDLE";
}

const char *ToString(Stability _s)
{
  switch (_s)
  {
    case Stability::UNDETERMINED: return "UNDETERMINED";
    case Stability::STABLE: return "STABLE";
    case Stability::UNSTABLE: return "UNSTABLE";
  }
  return "UNDETERMINED";
}

const char *ToString(Mode _m)
{
  switch (_m)
  {
    case Mode::IDLE: return "IDLE";
    case Mode::TRACK: return "TRACK";
    case Mode::MANUAL: return "MANUAL";
    case Mode::HOME: return "HOME";
  }
  return "IDLE";
}

GimbalControl::GimbalControl() = default;
GimbalControl::GimbalControl(const ControlConfig &_cfg) : cfg_(_cfg) {}

double GimbalControl::Clamp(double _v, double _lo, double _hi)
{
  return std::max(_lo, std::min(_hi, _v));
}

double GimbalControl::Slew(double _target, double _prev, double _dt) const
{
  const double max_d = cfg_.max_acc_dps2 * _dt;
  const double v = Clamp(_target, _prev - max_d, _prev + max_d);
  return Clamp(v, -cfg_.max_rate_dps, cfg_.max_rate_dps);
}

void GimbalControl::ResetFilters()
{
  ff_cmd_ = {0.0, 0.0, 0.0};
  has_ff_cmd_ = false;
  integral_ = {0.0, 0.0};
  last_err_ = {0.0, 0.0};
  last_meas_ = {0.0, 0.0};
  meas_rate_ = {0.0, 0.0};
  has_last_meas_t_ = false;
  has_last_err_t_ = false;
  err_window_.clear();
  has_stable_since_ = false;
  stability_ = Stability::UNDETERMINED;
}

std::pair<bool, std::string> GimbalControl::StartTracking(
    const cv::Mat &_bgr, const std::array<double, 4> &_box, int _width,
    int _height)
{
  Box box{_box[0] * _width, _box[1] * _height, _box[2] * _width,
          _box[3] * _height};
  auto [ok, reason] = tracker_.Start(_bgr, box);
  client_message_.clear();
  target_valid_ = false;
  has_box_ = false;
  has_frame_time_ = false;
  cmd_ = raw_cmd_ = {0.0, 0.0, 0.0};
  ResetFilters();
  if (!ok)
  {
    track_state_ = TrackState::FAULT;
    mode_ = Mode::IDLE;
    fault_reason_ = reason;
    if (reason == "selection_too_small")
      client_message_ = "选框过小，请重新框选";
    else if (reason == "selection_low_texture")
      client_message_ = "选框内纹理不足（可能对准天空），请重新框选";
    else if (reason == "init_update_failed")
      client_message_ = "跟踪初始化失败，请重新框选";
    else
      client_message_ = "初始化失败: " + reason;
    return {false, client_message_};
  }
  mode_ = Mode::TRACK;
  track_state_ = TrackState::INIT;
  fault_reason_.clear();
  init_started_ = 0.0;  // 由 Step 首次调用时按 now 记录
  has_init_started_ = false;
  has_lost_since_ = false;
  ResetFilters();
  return {true, ""};
}

void GimbalControl::Stop()
{
  mode_ = Mode::IDLE;
  track_state_ = TrackState::IDLE;
  target_valid_ = false;
  has_box_ = false;
  center_err_deg_ = 0.0;
  fault_reason_.clear();
  client_message_.clear();
  ResetFilters();
  raw_cmd_ = {0.0, 0.0, 0.0};
}

void GimbalControl::WatchdogStop()
{
  Stop();
  cmd_ = manual_cmd_ = {0.0, 0.0, 0.0};
}

void GimbalControl::Manual(double _yawDps, double _pitchDps, double _rollDps)
{
  if (mode_ != Mode::MANUAL)
    Stop();
  has_manual_time_ = false;
  mode_ = Mode::MANUAL;
  track_state_ = TrackState::IDLE;
  target_valid_ = false;
  const double r = cfg_.max_rate_dps;
  manual_cmd_ = {Clamp(_yawDps, -r, r), Clamp(_pitchDps, -r, r),
                 Clamp(_rollDps, -r, r)};
}

void GimbalControl::Manual(double _yawDps, double _pitchDps, double _rollDps,
                           double _now)
{
  Manual(_yawDps, _pitchDps, _rollDps);
  manual_time_ = _now;
  has_manual_time_ = true;
}

void GimbalControl::GoHome()
{
  Stop();
  mode_ = Mode::HOME;
  track_state_ = TrackState::IDLE;
  target_valid_ = false;
  home_target_ = {0.0, 0.0, 0.0};
}

void GimbalControl::GoLevelForward()
{
  Stop();
  mode_ = Mode::HOME;
  track_state_ = TrackState::IDLE;
  target_valid_ = false;
  const double az_fwd = cfg_.forward_az_deg * M_PI / 180.0;
  home_target_ = {WrapPi(M_PI - az_fwd), -M_PI / 2.0, 0.0};
}

void GimbalControl::ResetFault()
{
  if (track_state_ == TrackState::FAULT)
    Stop();
}

void GimbalControl::TargetCmdFromTracking()
{
  if (track_state_ == TrackState::TRACK && has_ff_cmd_)
    raw_cmd_ = ff_cmd_;
  else
    raw_cmd_ = {0.0, 0.0, 0.0};
}

void GimbalControl::TargetCmdHome(double _yaw, double _pitch, double _roll)
{
  const double k = 60.0;  // dps per rad
  constexpr double kRad2Deg = 180.0 / M_PI;
  const double dyaw = WrapPi(home_target_[0] - _yaw);
  const double dpitch = home_target_[1] - _pitch;
  const double droll = home_target_[2] - _roll;
  raw_cmd_ = {Clamp(dyaw * kRad2Deg * k * 0.0175, -40, 40),
              Clamp(dpitch * kRad2Deg * k * 0.0175, -40, 40),
              Clamp(droll * kRad2Deg * k * 0.0175, -40, 40)};
  if (std::abs(dyaw) < 0.5 * M_PI / 180.0 &&
      std::abs(dpitch) < 0.5 * M_PI / 180.0 &&
      std::abs(droll) < 0.5 * M_PI / 180.0)
  {
    mode_ = Mode::IDLE;
    raw_cmd_ = {0.0, 0.0, 0.0};
  }
}

void GimbalControl::ApplyLimits(std::array<double, 3> &_command,
                                double _rollRad, double _pitchRad, double _now)
{
  constexpr double kRad2Deg = 180.0 / M_PI;
  const double pitch_deg = _pitchRad * kRad2Deg;
  const double roll_deg = _rollRad * kRad2Deg;

  auto handle = [&](int _idx, double _posDeg, double _lo, double _hi,
                    double _cmd) -> double {
    bool flag = false;
    if (_posDeg >= _hi - cfg_.limit_margin_deg && _cmd > 0)
    {
      _cmd = 0.0;
      flag = true;
    }
    if (_posDeg <= _lo + cfg_.limit_margin_deg && _cmd < 0)
    {
      _cmd = 0.0;
      flag = true;
    }
    if (_posDeg <= _lo + 0.3 || _posDeg >= _hi - 0.3)
      flag = true;
    if (flag)
      limit_until_[_idx] = _now + 0.5;
    limit_flags_[_idx] = (_now < limit_until_[_idx]);
    return _cmd;
  };

  _command[1] = handle(1, pitch_deg, cfg_.pitch_min_deg, cfg_.pitch_max_deg,
                       _command[1]);
  _command[2] = handle(2, roll_deg, cfg_.roll_min_deg, cfg_.roll_max_deg,
                       _command[2]);
  limit_flags_[0] = false;
}

void GimbalControl::UpdateSaturation(const JointSample &_js, double _now)
{
  bool sat = false;
  const double vel[3] = {_js.yaw_vel, _js.pitch_vel, _js.roll_vel};
  for (int i = 0; i < 3; ++i)
  {
    if (std::abs(raw_cmd_[i]) > cfg_.max_rate_dps + 1e-6)
      sat = true;
    const double v = vel[i] * 180.0 / M_PI;
    if (std::abs(cmd_[i] - v) > cfg_.sat_vel_err_dps && std::abs(cmd_[i]) > 5.0)
      sat = true;
  }
  if (sat)
  {
    if (!has_sat_since_)
    {
      has_sat_since_ = true;
      sat_since_ = _now;
    }
    if (_now - sat_since_ >= cfg_.sat_hold_s)
      saturated_ = true;
  }
  else
  {
    has_sat_since_ = false;
    saturated_ = false;
  }
}

void GimbalControl::UpdateStability(double _now)
{
  const double cutoff = _now - 1.0;
  while (!err_window_.empty() && err_window_.front().first < cutoff)
    err_window_.pop_front();
  if (!target_valid_ || track_state_ != TrackState::TRACK ||
      err_window_.size() < 3)
  {
    stability_ = Stability::UNDETERMINED;
    has_stable_since_ = false;
    return;
  }
  double sum = 0.0;
  for (const auto &e : err_window_)
    sum += e.second * e.second;
  const double rms = std::sqrt(sum / err_window_.size());
  bool limited = false;
  for (bool f : limit_flags_)
    limited = limited || f;
  if (rms > cfg_.unstable_rms_deg || limited || saturated_)
  {
    stability_ = Stability::UNSTABLE;
    has_stable_since_ = false;
  }
  else if (rms <= cfg_.stable_rms_deg)
  {
    if (!has_stable_since_)
    {
      has_stable_since_ = true;
      stable_since_ = _now;
    }
    const bool window_covers = (_now - err_window_.front().first) >= 0.9;
    if (window_covers && (_now - stable_since_) >= cfg_.stable_enter_s)
      stability_ = Stability::STABLE;
    else if (stability_ != Stability::STABLE)
      stability_ = Stability::UNSTABLE;
  }
  else
  {
    if (stability_ == Stability::UNDETERMINED)
      stability_ = Stability::UNSTABLE;
  }
}

bool GimbalControl::TryReacquire(const cv::Mat &_bgr)
{
  auto [found, box] = tracker_.RetryCapture(_bgr);
  if (!found)
    return false;
  if (tracker_.Adopt(_bgr, box))
  {
    target_box_ = box;
    has_box_ = true;
    ResetFilters();
    client_message_ = "已重新捕获目标";
    return true;
  }
  return false;
}

void GimbalControl::UpdateMeasurement(double _now, const cv::Mat &_bgr,
                                      int _width, int _height, double _yaw,
                                      double _roll, double _pitch, double _watchdogNow)
{
  if (track_state_ == TrackState::LOST)
  {
    target_valid_ = false;
    ++invalid_frames_;
    return;
  }
  auto [valid, box] = tracker_.Update(_bgr);
  if (!valid && (track_state_ == TrackState::INIT ||
                 track_state_ == TrackState::TEMP_LOST) && TryReacquire(_bgr))
  {
    valid = true;
    box = target_box_;
  }
  if (valid)
  {
    target_box_ = box;
    has_box_ = true;
    target_valid_ = true;
    ++valid_frames_;
    const Vec3 d_s =
        PixelToSensorRay(box.Cx(), box.Cy(), _width, _height);
    auto [az, el] = SensorRayToAzEl(d_s, _yaw, _roll, _pitch);
    const Vec3 d_cam = LosMount(_yaw, _roll, _pitch);
    auto [az_cam, el_cam] = AzElFromDir(d_cam);
    const double az_err = WrapPi(az - az_cam);
    const double el_err = el - el_cam;
    measure_az_ = az;
    measure_el_ = el;
    const Vec3 d_t = MatVec(RMountSensor(_yaw, _roll, _pitch), d_s);
    center_err_deg_ = AngleBetween(d_cam, d_t) * 180.0 / M_PI;

    if (track_state_ == TrackState::INIT ||
        track_state_ == TrackState::TEMP_LOST)
    {
      track_state_ = TrackState::TRACK;
      has_lost_since_ = false;
    }
    err_window_.emplace_back(_now, center_err_deg_);

    double mdt = has_last_meas_t_ ? (_now - last_meas_t_) : 1.0 / 30.0;
    mdt = Clamp(mdt, 1.0 / 120.0, 0.5);
    if (has_last_meas_t_)
    {
      const double d_az_m = WrapPi(az - last_meas_[0]) / mdt;
      const double d_el_m = (el - last_meas_[1]) / mdt;
      constexpr double kRad2Deg = 180.0 / M_PI;
      meas_rate_[0] = 0.7 * meas_rate_[0] + 0.3 * d_az_m * kRad2Deg;
      meas_rate_[1] = 0.7 * meas_rate_[1] + 0.3 * d_el_m * kRad2Deg;
    }
    last_meas_ = {az, el};
    has_last_meas_t_ = true;
    last_meas_t_ = _now;

    // 没有载机运动补偿：前馈取目标角速度
    const double ff_az = meas_rate_[0];
    const double ff_el = meas_rate_[1];

    constexpr double kRad2Deg = 180.0 / M_PI;
    const double az_err_d = az_err * kRad2Deg;
    const double el_err_d = el_err * kRad2Deg;
    const bool fresh =
        has_last_err_t_ && (_now - last_err_t_) <= 0.15;
    const double d_az = fresh ? (az_err_d - last_err_[0]) / mdt : 0.0;
    const double d_el = fresh ? (el_err_d - last_err_[1]) / mdt : 0.0;
    last_err_ = {az_err_d, el_err_d};
    has_last_err_t_ = true;
    last_err_t_ = _now;

    if (std::abs(az_err_d) <= cfg_.integral_freeze_deg)
      integral_[0] = Clamp(integral_[0] + az_err_d * mdt, -cfg_.integral_limit,
                           cfg_.integral_limit);
    if (std::abs(el_err_d) <= cfg_.integral_freeze_deg)
      integral_[1] = Clamp(integral_[1] + el_err_d * mdt, -cfg_.integral_limit,
                           cfg_.integral_limit);

    const double az_rate = cfg_.kp * az_err_d + cfg_.ki * integral_[0] +
                           cfg_.kd * d_az + cfg_.ff * ff_az;
    const double el_rate = cfg_.kp * el_err_d + cfg_.ki * integral_[1] +
                           cfg_.kd * d_el + cfg_.ff * ff_el;
    ff_cmd_ = {-az_rate, el_rate, 0.0};
    has_ff_cmd_ = true;
  }
  else
  {
    target_valid_ = false;
    ++invalid_frames_;
    if (track_state_ == TrackState::TRACK)
    {
      track_state_ = TrackState::TEMP_LOST;
      has_lost_since_ = true;
      lost_since_ = _watchdogNow;
      ResetFilters();
    }
    else if (track_state_ == TrackState::TEMP_LOST)
    {
      const double since = has_lost_since_ ? lost_since_ : _watchdogNow;
      if (_watchdogNow - since >= cfg_.temp_lost_timeout_s)
        track_state_ = TrackState::LOST;
    }
  }
}

std::array<double, 3> GimbalControl::Step(double _now, bool _newFrame,
                                          const cv::Mat &_bgr, int _width,
                                          int _height, const JointSample &_js,
                                          double _watchdogNow)
{
  const double watchdog_now = _watchdogNow < 0.0 ? _now : _watchdogNow;
  double dt = has_last_cmd_t_ ? (_now - last_cmd_t_) : (1.0 / 100.0);
  dt = Clamp(dt, 1e-3, 0.2);

  bool safety_stop = false;
  if (mode_ == Mode::MANUAL)
  {
    if (!has_manual_time_)
    {
      manual_time_ = watchdog_now;
      has_manual_time_ = true;
    }
    if (watchdog_now - manual_time_ >= cfg_.manual_timeout_s)
    {
      WatchdogStop();
      safety_stop = true;
    }
  }
  if (mode_ == Mode::TRACK)
  {
    if (!has_init_started_)
    {
      init_started_ = watchdog_now;
      has_init_started_ = true;
    }
    if (!has_frame_time_)
    {
      frame_time_ = watchdog_now;
      has_frame_time_ = true;
    }
    const double age = watchdog_now - frame_time_;
    // 在处理恢复帧之前判断永久丢失，超过窗口不能用旧目标恢复。
    if (age >= cfg_.temp_lost_timeout_s ||
        (track_state_ == TrackState::INIT &&
         watchdog_now - init_started_ >= cfg_.temp_lost_timeout_s) ||
        (track_state_ == TrackState::TEMP_LOST && has_lost_since_ &&
         watchdog_now - lost_since_ >= cfg_.temp_lost_timeout_s))
    {
      track_state_ = TrackState::LOST;
      target_valid_ = false;
      ResetFilters();
      safety_stop = true;
    }
    if (_newFrame && !_bgr.empty())
    {
      frame_time_ = watchdog_now;
      UpdateMeasurement(_now, _bgr, _width, _height, _js.yaw, _js.roll, _js.pitch,
                        watchdog_now);
    }
    else if (age >= cfg_.frame_stop_timeout_s)
    {
      if (track_state_ == TrackState::TRACK || track_state_ == TrackState::INIT)
      {
        track_state_ = TrackState::TEMP_LOST;
        has_lost_since_ = true;
        lost_since_ = frame_time_;
      }
      target_valid_ = false;
      ResetFilters();
      safety_stop = true;
    }
  }

  if (mode_ == Mode::TRACK)
    TargetCmdFromTracking();
  else if (mode_ == Mode::MANUAL)
    raw_cmd_ = manual_cmd_;
  else if (mode_ == Mode::HOME)
    TargetCmdHome(_js.yaw, _js.pitch, _js.roll);
  else
    raw_cmd_ = {0.0, 0.0, 0.0};

  ApplyLimits(raw_cmd_, _js.roll, _js.pitch, _now);

  for (int i = 0; i < 3; ++i)
    cmd_[i] = safety_stop ? 0.0 : Slew(raw_cmd_[i], cmd_[i], dt);
  // 限位保护优先于加速度平滑，禁止残留速度继续向外。
  ApplyLimits(cmd_, _js.roll, _js.pitch, _now);
  has_last_cmd_t_ = true;
  last_cmd_t_ = _now;

  UpdateSaturation(_js, _now);
  UpdateStability(_now);
  return cmd_;
}

}  // namespace gimbal
