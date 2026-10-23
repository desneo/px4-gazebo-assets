#include "two_axis/control_schedule.hh"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
void Check(bool _condition, const char *_message)
{
  if (!_condition)
    throw std::runtime_error(_message);
}

std::vector<double> Run(double _dt, int _steps)
{
  two_axis::ControlSchedule schedule;
  Check(schedule.Configure(100), "合法控制频率可配置");
  std::vector<double> times;
  for (int index = 1; index <= _steps; ++index)
  {
    const double now = index * _dt;
    if (schedule.Update(now))
      times.push_back(now);
    Check(!schedule.Update(now), "同一物理状态不可重复控制");
  }
  return times;
}
}  // namespace

int main()
{
  using two_axis::ControlSchedule;

  const auto fourMs = Run(.004, 2500);
  Check(fourMs.size() == 1000, "250Hz物理步长期准确执行100Hz控制");
  for (std::size_t index = 1; index < 20; ++index)
  {
    const double interval = fourMs[index] - fourMs[index - 1];
    const double expected = index % 2 ? .012 : .008;
    Check(std::abs(interval - expected) < 1e-12,
          "4ms物理步的100Hz控制应按8/12ms交替并保留余相");
  }

  const auto threeMs = Run(.003, 10000);
  Check(threeMs.size() == 3000, "3ms物理步长期控制次数正确");
  for (std::size_t index = 1; index < threeMs.size(); ++index)
  {
    const double interval = threeMs[index] - threeMs[index - 1];
    Check(std::abs(interval - .009) < 1e-10 ||
          std::abs(interval - .012) < 1e-10,
          "3ms物理步只按9/12ms量化100Hz控制");
  }

  ControlSchedule large;
  Check(large.Configure(100) && large.Update(0), "首个物理状态立即控制");
  Check(large.Update(.024) && !large.Update(.024),
        "大dt只运行一次且同一步不补发");
  Check(!large.Update(.028) && large.Update(.032),
        "大dt丢弃漏跑周期但保留周期余相");

  ControlSchedule reset;
  Check(reset.Configure(100) && reset.Update(0), "暂停测试建立控制基准");
  Check(!reset.Update(.004) && !reset.Update(.004, true) &&
        !reset.Update(.004, true), "暂停不运行控制且不重复");
  Check(reset.Update(.008) && !reset.Update(.008),
        "恢复首个新物理状态重新同步且至多运行一次");
  Check(!reset.Update(.012) && reset.Update(.020),
        "恢复后从新基准保留余相");
  Check(reset.Update(.004) && !reset.Update(.004),
        "仿真时间回退重新同步且同一步至多运行一次");
  Check(!reset.Update(.008) && reset.Update(.016),
        "时间回退后不补跑旧周期");

  for (double hz : {0., -1., std::nan(""),
                    std::numeric_limits<double>::infinity()})
  {
    ControlSchedule invalid;
    Check(!invalid.Configure(hz) && !invalid.Update(0),
          "无效控制频率不能启动调度");
  }
  Check(!reset.Update(std::nan("")) &&
        !reset.Update(std::numeric_limits<double>::infinity()),
        "无效仿真时间不运行控制");

  std::cout << "PASS: phase-preserving control cadence\n";
}
