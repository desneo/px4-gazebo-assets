#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "zhongke_core/tracker.hh"
#include "zhongke_d80/kinematics.hh"

namespace zhongke_d80
{
inline constexpr double kTruthBoxPaddingPx{5.0};

enum class TrackingBackend { SIM_TRUTH, CSRT };
const char *ToString(TrackingBackend);
bool ParseTrackingBackend(const std::string &, TrackingBackend &);

struct TruthProjection
{
  std::uint64_t entity{0};
  std::string name;
  double center_x{0}, center_y{0}, depth{0}, range_m{0};
};

enum class TruthStatus { UNSYNCHRONIZED, INVISIBLE, VISIBLE };

struct TruthObservation
{
  TruthStatus status{TruthStatus::UNSYNCHRONIZED};
  Box box;
  double center_x{0}, center_y{0};
  std::int64_t frame_seq{-1};
};

std::optional<TruthProjection> ProjectTruthTarget(
  std::uint64_t entity, const std::string &name, const Vec3 &sensorPoint,
  int width, int height, double hfov, double nearClip, double farClip);

std::optional<TruthProjection> ProjectTruthTargetCalibrated(
  std::uint64_t entity, const std::string &name, const Vec3 &sensorPoint,
  int width, int height, double fx, double fy, double cx, double cy,
  double nearClip, double farClip);
// 仅归一化坐标变换和范数计算带来的double舍入边界，不增加业务距离裕量。
double CanonicalDiscoveryRange(double range, double maxRange);
// 几何元数据使用max_digits10有效位，发布后可精确还原原double。
std::string PreciseNumber(double value);
bool CanAcquire(const TruthProjection &, int width, int height, double maxRange);

std::optional<std::size_t> SelectTruthTarget(
  const std::vector<TruthProjection> &, const Box &selection,
  int width, int height);

TruthObservation MakeTruthObservation(
  const TruthProjection &, const Box &initialBox, double initialDepth,
  int width, int height, std::int64_t frameSeq);
}
