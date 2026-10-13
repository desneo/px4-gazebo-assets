#include "two_axis/control.hh"
#include <algorithm>
#include <cmath>
namespace two_axis
{
constexpr double D = 180 / M_PI;
const char *ToString(Mode s)
{
  switch (s)
  {
  case Mode::IDLE:
    return "IDLE";
  case Mode::MANUAL:
    return "MANUAL";
  case Mode::HOME:
    return "HOME";
  case Mode::TRACK:
    return "TRACK";
  }
  return "IDLE";
}
const char *ToString(TrackState s)
{
  switch (s)
  {
  case TrackState::IDLE:
    return "IDLE";
  case TrackState::INIT:
    return "INIT";
  case TrackState::TRACK:
    return "TRACK";
  case TrackState::TEMP_LOST:
    return "TEMP_LOST";
  case TrackState::LOST:
    return "LOST";
  case TrackState::FAULT:
    return "FAULT";
  }
  return "FAULT";
}
const char *ToString(Stability s)
{
  switch (s)
  {
  case Stability::STABLE:
    return "STABLE";
  case Stability::UNSTABLE:
    return "UNSTABLE";
  default:
    return "UNDETERMINED";
  }
}
void GimbalControl::Stop()
{
  mode_ = Mode::IDLE;
  state_ = TrackState::IDLE;
  cmd_ = {};
  manual_ = {};
  valid_ = false;
  has_box_ = false;
  stability_ = Stability::UNDETERMINED;
  stable_time_ = -1;
}
void GimbalControl::WatchdogStop()
{
  Stop();
  message_ = "控制输入超时，已停止";
}
void GimbalControl::GoHome()
{
  Stop();
  target_ = {};
  mode_ = Mode::HOME;
}
void GimbalControl::ResetFault()
{
  Stop();
  fault_.clear();
  message_.clear();
}
void GimbalControl::Manual(double y, double p, double now)
{
  mode_ = Mode::MANUAL;
  state_ = TrackState::IDLE;
  valid_ = false;
  has_box_ = false;
  manual_ = {y, p};
  manual_time_ = now;
  stability_ = Stability::UNDETERMINED;
  stable_time_ = -1;
}
bool GimbalControl::ResetAxis(const std::string &axis, const JointSample &j)
{
  if (axis != "yaw" && axis != "pitch")
    return false;
  GoHome();
  target_ = {j.yaw * D, j.pitch * D};
  target_[axis == "yaw" ? 0 : 1] = 0;
  return true;
}
std::pair<bool, std::string> GimbalControl::StartTracking(const cv::Mat &im, const std::array<double, 4>&b, int w, int h)
{
  if (!fault_.empty())
    return {false, "请先复位故障"};
  for (double v : b)
    if (!std::isfinite(v))
      return {false, "选框必须为有限数值"};
  if (w <= 0 || h <= 0 || im.empty() || b[0] < 0 || b[1] < 0 || b[2] <= 0 || b[3] <= 0 || b[0] + b[2] > 1 || b[1] + b[3] > 1)
    return {false, "无效选框或视频"};
  Box box{b[0]*w, b[1]*h, b[2]*w, b[3]*h};
  auto r = tracker_.Start(im, box);
  if (!r.first)
    return r;
  Stop();
  box_ = box;
  has_box_ = true;
  mode_ = Mode::TRACK;
  state_ = TrackState::INIT;
  frame_time_ = -1;
  lost_time_ = -1;
  message_.clear();
  return r;
}
std::array<double, 2> GimbalControl::Step(double now, bool fresh, const cv::Mat &im, int w, int h, const JointSample &j, double watch)
{
  if (!std::isfinite(j.yaw) || !std::isfinite(j.pitch) || !std::isfinite(j.yaw_vel) || !std::isfinite(j.pitch_vel))
  {
    Stop();
    state_ = TrackState::FAULT;
    fault_ = "关节采样无效";
    return cmd_;
  }
  if (!fault_.empty())
  {
    state_ = TrackState::FAULT;
    mode_ = Mode::IDLE;
    cmd_ = {};
    return cmd_;
  }
  if (watch < 0)
    watch = now;
  double dt = last_time_ < 0 ? .01 : std::clamp(now - last_time_, 0., .1);
  last_time_ = now;
  std::array<double, 2> raw{};
  limits_ = {};
  saturated_ = false;
  if (mode_ == Mode::MANUAL)
  {
    if (watch - manual_time_ >= cfg_.manual_timeout_s)
    {
      WatchdogStop();
      return cmd_;
    }
    raw = manual_;
  }
  if (mode_ == Mode::HOME)
  {
    raw = {cfg_.kp*(target_[0] - j.yaw * D), cfg_.kp*(target_[1] - j.pitch * D)};
    if (std::hypot(raw[0], raw[1]) < .1)
    {
      Stop();
      return cmd_;
    }
  }
  if (mode_ == Mode::TRACK)
  {
    if (fresh && !im.empty() && w > 0 && h > 0)
    {
      frame_time_ = watch;
      auto r = state_ == TrackState::TEMP_LOST ? tracker_.RetryCapture(im) : tracker_.Update(im);
      if (r.first)
      {
        if (state_ == TrackState::TEMP_LOST && !tracker_.Adopt(im, r.second))
          r.first = false;
      }
      if (r.first)
      {
        box_ = r.second;
        valid_ = true;
        has_box_ = true;
        ++valid_frames_;
        state_ = TrackState::TRACK;
        lost_time_ = -1;
        const auto ray = PixelToSensorRay(box_.Cx(), box_.Cy(), w, h);
        image_error_ = {std::atan2(-ray.y, ray.x)*D, std::atan2(-ray.z, ray.x)*D};
        error_ = std::hypot(image_error_[0], image_error_[1]);
      }
      else
      {
        valid_ = false;
        ++invalid_frames_;
        if (lost_time_ < 0)
          lost_time_ = watch;
        state_ = TrackState::TEMP_LOST;
      }
    }
    if (frame_time_ < 0)
      frame_time_ = watch;
    if (watch - frame_time_ >= cfg_.frame_stop_timeout_s)
    {
      cmd_ = {};
      valid_ = false;
      state_ = TrackState::TEMP_LOST;
      if (lost_time_ < 0)
        lost_time_ = watch;
    }
    if (lost_time_ >= 0 && watch - lost_time_ >= cfg_.occlusion_timeout_s)
    {
      Stop();
      state_ = TrackState::LOST;
      message_ = "目标丢失，请重新选框";
      return cmd_;
    }
    if (valid_ && watch - frame_time_ < cfg_.frame_stop_timeout_s)
      raw = ImageRates(j.pitch, cfg_.kp * image_error_[0], cfg_.kp * image_error_[1]);
    else
    {
      cmd_ = {};
      return cmd_;
    }
    if (error_ < 1)
    {
      if (stable_time_ < 0)
        stable_time_ = now;
      stability_ = now - stable_time_ >= 1 ? Stability::STABLE : Stability::UNDETERMINED;
    }
    else
    {
      stable_time_ = -1;
      stability_ = error_ > 2 ? Stability::UNSTABLE : Stability::UNDETERMINED;
    }
  }
  double pos[2] = {j.yaw * D, j.pitch * D}, lo[2] = {cfg_.yaw_min_deg, cfg_.pitch_min_deg}, hi[2] = {cfg_.yaw_max_deg, cfg_.pitch_max_deg};
  for (int i = 0; i < 2; ++i)
  {
    double v = std::clamp(raw[i], -cfg_.max_rate_dps, cfg_.max_rate_dps);
    saturated_ |= v != raw[i];
    cmd_[i] += std::clamp(v - cmd_[i], -cfg_.max_acc_dps2 * dt, cfg_.max_acc_dps2 * dt);
    // 速度指令在下一次控制更新前持续保持。先做加速度平滑，再限制
    // 可行进距离，避免100 Hz控制跨过0.2°软件缓冲后撞击物理硬限位。
    // 两倍观测控制间隔给离散更新和物理采样保留余量；只限制向外速度，
    // 在任一端点仍可立即反向退出。
    const double horizon = std::max(.02, 2 * dt);
    const double maxPositive = std::max(0., hi[i] - .2 - pos[i]) / horizon;
    const double maxNegative = std::max(0., pos[i] - lo[i] - .2) / horizon;
    const double safe = std::clamp(cmd_[i], -maxNegative, maxPositive);
    if (safe != cmd_[i] ||
        (pos[i] >= hi[i] - .2 && raw[i] > 0) ||
        (pos[i] <= lo[i] + .2 && raw[i] < 0))
    {
      limits_[i] = true;
      saturated_ = true;
    }
    cmd_[i] = safe;
  }
  return cmd_;
}
}
