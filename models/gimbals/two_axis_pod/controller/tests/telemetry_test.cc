#include "two_axis/telemetry.hh"

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

two_axis::TelemetrySchedule::Clock::time_point Wall(double _seconds)
{
  return two_axis::TelemetrySchedule::Clock::time_point(
    std::chrono::seconds(1000000) +
    std::chrono::nanoseconds(std::llround(_seconds * 1e9)));
}

void Cadence(double _hz, double _wallDt, double _simDt,
             int _steps, int _expected)
{
  two_axis::TelemetrySchedule schedule;
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
  using namespace two_axis;
  // 每次仍为4ms物理仿真步，三种RTF在真实10秒内均发1000个新姿态。
  Cadence(100.0, .005, .004, 2000, 1000);  // RTF 0.8
  Cadence(100.0, .008, .004, 1250, 1000);  // RTF 0.5
  Cadence(100.0, .002, .004, 5000, 1000);  // RTF 2.0
  Cadence(50.0, .004, .004, 2500, 500);
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

  for (double hz : {0., -1., 1000.001, std::nan(""), std::numeric_limits<double>::infinity()})
    Check(!slow.Configure(hz) && !slow.Update(Wall(1), 1).publish, "无效频率不能启动调度");
  Check(slow.Configure(1000) && !slow.Update(Wall(0), 0).publish &&
        slow.Update(Wall(.004), .004).publish && !slow.Update(Wall(.004), .004).publish,
        "允许1000Hz但每物理步至多一次");

  double hz = 17.;
  std::string error;
  for (const auto &text : {"", "[]", "null", "{}", "{\"hz\":true}", "{\"hz\":false}",
       "{\"hz\":\"100\"}", "{\"hz\":null}", "{\"hz\":0}", "{\"hz\":-1}",
       "{\"hz\":1000.1}", "{\"hz\":1e999}", "{\"hz\":NaN}", "{\"hz\":100} trailing"})
  {
    Check(!ParseTelemetryConfig(text, hz, error) && !error.empty(), "无效JSON配置必须有明确错误");
    Check(hz == 17., "解析失败不得替换原有配置值");
  }
  Check(ParseTelemetryConfig("{\"hz\":100}", hz, error) && hz == 100 && error.empty(),
        "共享JSON配置100Hz");
  Check(ParseTelemetryConfig("{\"hz\":29.97}", hz, error) && hz == 29.97,
        "允许非整数频率");
  Check(ParseTelemetryConfig("{\"hz\":1e-20}", hz, error) && hz > 0 && hz < 1,
        "允许任意正有限频率");

  namespace fs = std::filesystem;
  const auto root = fs::temp_directory_path() / ("two-axis-telemetry-" +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  struct Cleanup
  {
    fs::path path;
    ~Cleanup() { std::error_code ec; fs::remove_all(path, ec); }
  } cleanup{root};
  const auto path = root / "two_axis_pod/controller/service/telemetry.json";
  fs::create_directories(path.parent_path());
  { std::ofstream file(path); file << "{\"hz\":100}"; }
  Check(LoadTelemetryConfig(kDefaultTelemetryConfig, {"/nonexistent", root.string()}, hz, error) && hz == 100,
        "model资源按照搜索路径解析共享配置");
  Check(LoadTelemetryConfig(path.string(), {}, hz, error) && hz == 100,
        "支持显式绝对配置路径");
  Check(!LoadTelemetryConfig("telemetry.json", {root.string()}, hz, error),
        "不接受依赖工作目录的相对配置");
  Check(!LoadTelemetryConfig("model://missing/config.json", {root.string()}, hz, error) && !error.empty(),
        "资源缺失明确失败");
  Check(!LoadTelemetryConfig((root/"missing.json").string(), {}, hz, error),
        "绝对配置缺失明确失败");
  { std::ofstream file(path); file << "{\"hz\":false}"; }
  Check(!LoadTelemetryConfig(kDefaultTelemetryConfig, {root.string()}, hz, error),
        "找到资源但配置无效时禁用而非回退默认频率");
  std::cout << "PASS: shared telemetry config and steady-clock cadence\n";
}
