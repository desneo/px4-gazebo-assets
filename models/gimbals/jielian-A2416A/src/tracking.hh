#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace jielian_a2416a
{
struct Vec3
{
  double x{0};
  double y{0};
  double z{0};
};

struct Box
{
  double x{0};
  double y{0};
  double w{0};
  double h{0};
  double CenterX() const { return x + w / 2.0; }
  double CenterY() const { return y + h / 2.0; }
};

struct Optics
{
  int width{1920};
  int height{1080};
  double fx{2869.138128807485};
  double fy{2913.578907196434};
  double cx{960};
  double cy{540};
  double nearClip{0.005};
  double farClip{30000};
};

struct Detection
{
  std::uint64_t entity{0};
  std::string name;
  Box box;
  double centerX{0};
  double centerY{0};
  double rangeM{0};
  double confidence{1.0};
  int type{4};
  bool geographicValid{false};
  double latitudeDeg{0};
  double longitudeDeg{0};
  double altitudeM{0};
};

/// 将世界 AABB 的八个角（已变换到相机光学坐标）投影为图像检测框。
/// 光学坐标为 +X 前、+Y 左、+Z 上，图像坐标为 X 右、Y 下。
std::optional<Detection> ProjectBounds(
  std::uint64_t entity, const std::string &name,
  const std::array<Vec3, 8> &cornersCamera, const Vec3 &centerCamera,
  const Optics &optics);

enum class TrackState { IDLE, DETECTING, TRACKING, TEMP_LOST, LOST };
enum class LockMode { NONE, MANUAL, AUTO };
const char *ToString(TrackState state);

struct TrackerConfig
{
  Optics optics;
  double autoRangeM{780};
  double manualRangeM{1200};
  double lostTimeoutS{2.5};
  std::size_t maxTargets{4};
  double misalignmentLimitDeg{20};
  double crosshairMaxSpeedPxS{400};
  double crosshairTimeoutS{0.5};
  double gateScaleMin{0.5};
  double gateScaleMax{4.0};
};

class TruthTracker
{
public:
  bool Configure(const TrackerConfig &config);
  void Update(double now, std::vector<Detection> detections);
  bool SetRecognition(bool enabled);
  bool SetAutoLock(bool enabled, double now);
  bool PointLock(double x, double y, double now);
  bool BoxLock(const Box &selection, double now);
  bool CrosshairLock(double now);
  void Cancel();
  void MoveCrosshair(double xPercent, double yPercent, double now);
  bool TrimAim(double dx, double dy);
  void ScaleGate(double factor);

  bool RecognitionEnabled() const { return recognitionEnabled_; }
  bool AutoLockEnabled() const { return autoLockEnabled_; }
  TrackState State() const { return state_; }
  LockMode Mode() const { return lockMode_; }
  std::uint64_t MeasurementSeq() const { return measurementSeq_; }
  const std::vector<Detection> &Slots() const { return slots_; }
  const std::optional<Detection> &Locked() const { return locked_; }
  std::array<double, 2> Crosshair() const { return crosshair_; }
  std::array<double, 2> MisalignmentDeg() const;
  std::optional<Box> Gate() const;

private:
  bool Lock(const Detection &detection, LockMode mode, double now);
  static double IntersectionArea(const Box &a, const Box &b);
  void BuildSlots(const std::vector<Detection> &automatic);

  TrackerConfig config_;
  bool configured_{false};
  bool recognitionEnabled_{false};
  bool autoLockEnabled_{false};
  TrackState state_{TrackState::IDLE};
  LockMode lockMode_{LockMode::NONE};
  std::optional<Detection> locked_;
  std::vector<Detection> allVisible_;
  std::vector<Detection> slots_;
  std::uint64_t measurementSeq_{0};
  double lostSince_{-1};
  double lastUpdate_{-1};
  std::array<double, 2> crosshair_{{960, 540}};
  std::array<double, 2> crosshairVelocity_{{0, 0}};
  double crosshairCommandTime_{-1};
  std::array<double, 2> aimOffset_{{0, 0}};
  double gateScale_{1.0};
};
}
