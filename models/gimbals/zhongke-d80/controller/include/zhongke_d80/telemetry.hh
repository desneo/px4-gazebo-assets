#ifndef ZHONGKE_D80_TELEMETRY_HH_
#define ZHONGKE_D80_TELEMETRY_HH_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>


namespace zhongke_d80
{
inline bool ValidTelemetryHz(double hz)
{
  return std::isfinite(hz) && hz > 0;
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
}  // namespace zhongke_d80

#endif  // ZHONGKE_D80_TELEMETRY_HH_
