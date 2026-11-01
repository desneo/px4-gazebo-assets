#ifndef TWO_AXIS_TELEMETRY_HH_
#define TWO_AXIS_TELEMETRY_HH_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "gimbal_controller/mini_json.hh"

namespace two_axis
{
constexpr const char *kDefaultTelemetryConfig =
  "model://two_axis_pod/controller/service/telemetry.json";

inline bool ValidTelemetryHz(double _hz)
{
  return std::isfinite(_hz) && _hz > 0.0 && _hz <= 1000.0;
}

inline bool ParseTelemetryConfig(const std::string &_text, double &_hz,
                                 std::string &_error)
{
  gimbal::json::Value root;
  if (!gimbal::json::Parse(_text, root) ||
      root.type != gimbal::json::Value::Type::OBJ)
  {
    _error = "遥测配置必须为有效JSON对象";
    return false;
  }
  const auto *hz = root.Find("hz");
  if (!hz || hz->type != gimbal::json::Value::Type::NUM ||
      !ValidTelemetryHz(hz->num))
  {
    _error = "遥测配置hz必须为有限数字，范围(0,1000]，不接受布尔值";
    return false;
  }
  _hz = hz->num;
  _error.clear();
  return true;
}

inline bool LoadTelemetryConfig(const std::string &_uri,
                                const std::vector<std::string> &_resourcePaths,
                                double &_hz, std::string &_error)
{
  namespace fs = std::filesystem;
  std::vector<fs::path> candidates;
  if (fs::path(_uri).is_absolute())
    candidates.emplace_back(_uri);
  else if (_uri.compare(0, 8, "model://") == 0 && _uri.size() > 8)
  {
    const fs::path relative(_uri.substr(8));
    if (!relative.is_absolute())
      for (const auto &root : _resourcePaths)
        candidates.emplace_back(fs::path(root) / relative);
  }
  else
  {
    _error = "telemetry_config必须为绝对路径或model://资源路径";
    return false;
  }
  for (const auto &candidate : candidates)
  {
    std::error_code ec;
    if (!fs::is_regular_file(candidate, ec))
      continue;
    std::ifstream stream(candidate);
    if (!stream)
    {
      _error = "无法读取遥测配置: " + candidate.string();
      return false;
    }
    const std::string text((std::istreambuf_iterator<char>(stream)),
                           std::istreambuf_iterator<char>());
    if (stream.bad())
    {
      _error = "读取遥测配置失败: " + candidate.string();
      return false;
    }
    if (!ParseTelemetryConfig(text, _hz, _error))
    {
      _error = candidate.string() + ": " + _error;
      return false;
    }
    return true;
  }
  _error = "未找到遥测配置: " + _uri;
  return false;
}

struct TelemetryTick
{
  bool publish{false};
  bool warn_rate_limited{false};
  double physical_step_hz{0.0};
};

/// 使用真实单调时钟保留周期相位，每个新物理状态至多发布一次。
class TelemetrySchedule
{
public:
  using Clock = std::chrono::steady_clock;

  bool Configure(double _hz)
  {
    this->hz_ = ValidTelemetryHz(_hz) ? _hz : 0.0;
    this->initialized_ = false;
    return this->hz_ > 0.0;
  }

  TelemetryTick Update(Clock::time_point _wallNow, double _simNow,
                        bool _paused = false)
  {
    TelemetryTick result;
    if (!std::isfinite(_simNow) || this->hz_ <= 0.0)
      return result;
    if (!this->initialized_ || _paused || this->paused_ ||
        _wallNow < this->lastWall_ || _simNow < this->lastSim_)
    {
      this->Reset(_wallNow, _simNow, _paused);
      return result;
    }
    const bool fresh = _simNow > this->lastSim_;
    const auto elapsed = std::chrono::duration<long double>(
      _wallNow - this->lastWall_).count();
    this->lastWall_ = _wallNow;
    this->lastSim_ = _simNow;
    const long double cycles = elapsed * this->hz_;
    this->phase_ += cycles;
    this->totalCycles_ += cycles;
    // hz的double表示有误差；误差界随累计周期数增长。
    const long double tolerance = std::min(1e-6L,
      32.0L * std::numeric_limits<double>::epsilon()
      * std::max(1.0L, this->totalCycles_));
    if (this->phase_ + tolerance >= 1.0L)
    {
      this->phase_ -= std::floor(this->phase_ + tolerance);
      result.publish = fresh;
    }

    // 比较真实观察窗内的新物理步数，避免把8/12ms等发布量化视为限频。
    if (fresh)
      ++this->windowSteps_;
    const double windowSeconds = std::chrono::duration<double>(
      _wallNow - this->windowStart_).count();
    if (windowSeconds >= 2.0)
    {
      result.physical_step_hz = this->windowSteps_ / windowSeconds;
      const bool limited = this->windowSteps_ + 1.0 < this->hz_ * windowSeconds;
      if (limited && (!this->hasWarning_ ||
          std::chrono::duration<double>(_wallNow - this->lastWarning_).count() >= 5.0))
      {
        result.warn_rate_limited = true;
        this->lastWarning_ = _wallNow;
        this->hasWarning_ = true;
      }
      this->windowStart_ = _wallNow;
      this->windowSteps_ = 0;
    }
    return result;
  }

private:
  void Reset(Clock::time_point _wallNow, double _simNow, bool _paused)
  {
    this->initialized_ = true;
    this->paused_ = _paused;
    this->lastWall_ = _wallNow;
    this->lastSim_ = _simNow;
    this->phase_ = this->totalCycles_ = 0.0L;
    this->windowStart_ = _wallNow;
    this->windowSteps_ = 0;
    this->hasWarning_ = false;
  }

  double hz_{0.0};
  bool initialized_{false};
  bool paused_{false};
  Clock::time_point lastWall_{};
  double lastSim_{0.0};
  long double phase_{0.0L};
  long double totalCycles_{0.0L};
  Clock::time_point windowStart_{};
  std::uint64_t windowSteps_{0};
  bool hasWarning_{false};
  Clock::time_point lastWarning_{};
};
}  // namespace two_axis

#endif  // TWO_AXIS_TELEMETRY_HH_
