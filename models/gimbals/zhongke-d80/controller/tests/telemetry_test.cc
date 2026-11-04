#include "zhongke_d80/telemetry.hh"

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace
{
void Check(bool _condition, const char *_message)
{
  if (!_condition)
    throw std::runtime_error(_message);
}

zhongke_d80::TelemetrySchedule::Clock::time_point Wall(double _seconds)
{
  return zhongke_d80::TelemetrySchedule::Clock::time_point(
    std::chrono::seconds(1000000) +
    std::chrono::nanoseconds(std::llround(_seconds * 1e9)));
}

void Cadence(double _hz, double _wallDt, double _simDt,
             int _steps, int _expected)
{
  zhongke_d80::TelemetrySchedule schedule;
  Check(schedule.Configure(_hz), "合法遥测频率可配置");
  Check(!schedule.Update(Wall(0), 0).publish, "首个物理样本建立真实时间基准");
  int published = 0;
  for (int i = 1; i <= _steps; ++i)
  {
    const auto tick = schedule.Update(Wall(i * _wallDt), i * _simDt);
    published += tick.publish;
    Check(!tick.warn_rate_limited, "充足真实物理更新频率不能因发布量化误报警");
    const double target = i * _wallDt * _hz;
    Check(std::abs(published - target) < 1.000001,
          "发布数量与累计真实时间保持在一个物理步量化误差内");
    Check(!schedule.Update(Wall(i * _wallDt), i * _simDt).publish,
          "同一物理状态不可重复发布");
  }
  if (published != _expected)
    std::cerr << "hz=" << _hz << " wall_dt=" << _wallDt << " steps=" << _steps
              << " published=" << published << " expected=" << _expected << '\n';
  Check(published == _expected, "相位余量避免长期丢失发布周期");
}
}  // namespace

int main()
{
  using namespace zhongke_d80;
  // 每次仍为4ms物理仿真步，三种RTF在真实10秒内均发1000个新姿态。
  Cadence(100.0, .005, .004, 2000, 1000);  // RTF 0.8
  Cadence(100.0, .008, .004, 1250, 1000);  // RTF 0.5
  Cadence(100.0, .002, .004, 5000, 1000);  // RTF 2.0
  Cadence(60.0, .004, .004, 2500, 600);
  Cadence(29.97, .004, .004, 25000, 2997);
  Cadence(123.45, .004, .004, 25000, 12345);

  TelemetrySchedule slow;
  Check(slow.Configure(100), "配置100Hz");
  slow.Update(Wall(0), 0);
  Check(slow.Update(Wall(.024), .004).publish, "跨越两真实周期的物理步仅触发一次");
  Check(!slow.Update(Wall(.028), .008).publish && slow.Update(Wall(.032), .012).publish,
        "大步长丢弃漏发周期但保留真实时间相位");
  Check(!slow.Update(Wall(.032), .012).publish, "同一步不能补发重复状态");

  Check(slow.Configure(100) && !slow.Update(Wall(0), 0).publish, "重新配置清空相位");
  Check(!slow.Update(Wall(.004), .004).publish, "不足真实周期不发布");
  Check(!slow.Update(Wall(10), .004, true).publish &&
        !slow.Update(Wall(20), .004, true).warn_rate_limited &&
        !slow.Update(Wall(30), .008).publish &&
        !slow.Update(Wall(30.004), .012).publish &&
        slow.Update(Wall(30.010), .016).publish,
        "暂停与恢复首步重建基准，恢复不补发暂停积压");
  Check(!slow.Update(Wall(31), 0).publish &&
        !slow.Update(Wall(31.009), .004).publish &&
        slow.Update(Wall(31.010), .008).publish,
        "仿真时间回退重建真实时间基准");
  Check(!slow.Update(Wall(30), .012).publish &&
        slow.Update(Wall(30.010), .016).publish, "时钟回退注入也安全重置");
  Check(!slow.Update(Wall(32), std::nan("")).publish &&
        !slow.Update(Wall(32), std::numeric_limits<double>::infinity()).publish,
        "无效仿真时间不产生遥测");

  slow.Configure(100);
  slow.Update(Wall(0), 0);
  Check(!slow.Update(Wall(.020), 0).publish &&
        !slow.Update(Wall(.024), .004).publish &&
        slow.Update(Wall(.030), .008).publish,
        "真实时间经过而物理状态不变时不得重发旧姿态");

  slow.Configure(100);
  slow.Update(Wall(0), 0);
  int packets = 0, warnings = 0;
  double lastWarning = -100;
  for (int i = 1; i <= 500; ++i)
  {
    const auto tick = slow.Update(Wall(i*.024), i*.004);
    packets += tick.publish;
    Check(!slow.Update(Wall(i*.024), i*.004).publish, "慢物理步同一状态也不可补发");
    if (tick.warn_rate_limited)
    {
      Check(i*.024-lastWarning >= 5 && std::abs(tick.physical_step_hz-1/.024) < 1e-9,
            "真实物理步频不足按观察窗测量，限频告警至少间隔5秒");
      ++warnings;
      lastWarning = i*.024;
    }
  }
  Check(packets == 500 && warnings == 2,
        "12秒慢物理更新仅500个新状态，明确限频而非重复凑包");

  for (double hz : {0., -1., std::nan(""), std::numeric_limits<double>::infinity()})
    Check(!slow.Configure(hz) && !slow.Update(Wall(1), 1).publish, "无效频率不能启动调度");
  Check(slow.Configure(1000) && !slow.Update(Wall(0), 0).publish &&
        slow.Update(Wall(.004), .004).publish && !slow.Update(Wall(.004), .004).publish,
        "允许1000Hz但每物理步至多一次");

  std::cout << "中科真实单调时钟调度测试通过\n";
}
