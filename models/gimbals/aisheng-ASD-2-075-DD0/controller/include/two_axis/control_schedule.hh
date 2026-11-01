#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace two_axis
{
/// 按仿真时间保留控制周期余相；每个新物理状态至多触发一次。
class ControlSchedule
{
public:
  bool Configure(double _hz)
  {
    this->hz_ = std::isfinite(_hz) && _hz > 0.0 ? _hz : 0.0;
    this->initialized_ = false;
    return this->hz_ > 0.0;
  }

  bool Update(double _simNow, bool _paused = false)
  {
    if (!std::isfinite(_simNow) || this->hz_ <= 0.0)
      return false;
    if (!this->initialized_)
    {
      this->Reset(_simNow, _paused);
      return !_paused;
    }
    if (_paused)
    {
      this->Reset(_simNow, true);
      return false;
    }
    if (this->paused_ || _simNow < this->lastSim_)
    {
      this->Reset(_simNow, false);
      return true;
    }
    if (_simNow == this->lastSim_)
      return false;

    const long double elapsed = _simNow - this->lastSim_;
    this->lastSim_ = _simNow;
    const long double cycles = elapsed * this->hz_;
    this->phase_ += cycles;
    this->totalCycles_ += cycles;
    // 仿真时间以double传入；容差只修正边界舍入，不改变一个物理步的量化界。
    const long double tolerance = std::min(1e-6L,
      32.0L * std::numeric_limits<double>::epsilon() *
      std::max(1.0L, this->totalCycles_));
    if (this->phase_ + tolerance < 1.0L)
      return false;
    this->phase_ -= std::floor(this->phase_ + tolerance);
    return true;
  }

private:
  void Reset(double _simNow, bool _paused)
  {
    this->initialized_ = true;
    this->paused_ = _paused;
    this->lastSim_ = _simNow;
    this->phase_ = 0.0L;
    this->totalCycles_ = 0.0L;
  }

  double hz_{0.0};
  bool initialized_{false};
  bool paused_{false};
  double lastSim_{0.0};
  long double phase_{0.0L};
  long double totalCycles_{0.0L};
};
}  // namespace two_axis
