#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "gimbal_controller/tracker.hh"
#include "two_axis/kinematics.hh"

namespace two_axis
{
inline constexpr double kTruthBoxPaddingPx{5.0};

enum class TrackingBackend { SIM_TRUTH, CSRT };
const char *ToString(TrackingBackend);
bool ParseTrackingBackend(const std::string &, TrackingBackend &);

struct TruthProjection
{
  std::uint64_t entity{0};
  std::string name;
  double center_x{0}, center_y{0}, depth{0};
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

std::optional<std::size_t> SelectTruthTarget(
  const std::vector<TruthProjection> &, const Box &selection,
  int width, int height);

TruthObservation MakeTruthObservation(
  const TruthProjection &, const Box &initialBox, double initialDepth,
  int width, int height, std::int64_t frameSeq);
}
