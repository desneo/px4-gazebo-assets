#include "zhongke_d80/truth_tracking.hh"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>

namespace zhongke_d80
{
const char *ToString(TrackingBackend backend)
{
  return backend == TrackingBackend::CSRT ? "csrt" : "sim_truth";
}

bool ParseTrackingBackend(const std::string &text, TrackingBackend &backend)
{
  if (text == "sim_truth")
  {
    backend = TrackingBackend::SIM_TRUTH;
    return true;
  }
  if (text == "csrt")
  {
    backend = TrackingBackend::CSRT;
    return true;
  }
  return false;
}

std::optional<TruthProjection> ProjectTruthTarget(
  std::uint64_t entity, const std::string &name, const Vec3 &p,
  int width, int height, double hfov, double nearClip, double farClip)
{
  if (width <= 0 || height <= 0 || !std::isfinite(hfov) || hfov <= 0 ||
      hfov >= M_PI || !std::isfinite(nearClip) || !std::isfinite(farClip) ||
      nearClip <= 0 || farClip <= nearClip || !std::isfinite(p.x) ||
      !std::isfinite(p.y) || !std::isfinite(p.z) || p.x < nearClip ||
      p.x > farClip)
    return std::nullopt;
  const double focal = width / (2.0 * std::tan(hfov / 2.0));
  const double u = width / 2.0 - focal * p.y / p.x;
  const double v = height / 2.0 - focal * p.z / p.x;
  if (!std::isfinite(u) || !std::isfinite(v))
    return std::nullopt;
  return TruthProjection{entity, name, u, v, p.x, std::hypot(p.x, p.y, p.z)};
}

std::optional<TruthProjection> ProjectTruthTargetCalibrated(
  std::uint64_t entity, const std::string &name, const Vec3 &p,
  int width, int height, double fx, double fy, double cx, double cy,
  double nearClip, double farClip)
{
  if (width <= 0 || height <= 0 || !std::isfinite(fx) || !std::isfinite(fy) ||
      fx <= 0 || fy <= 0 || !std::isfinite(cx) || !std::isfinite(cy) ||
      !std::isfinite(nearClip) || !std::isfinite(farClip) || nearClip <= 0 ||
      farClip <= nearClip || !std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(p.z) || p.x < nearClip || p.x > farClip)
    return std::nullopt;
  const double u = cx - fx * p.y / p.x;
  const double v = cy - fy * p.z / p.x;
  if (!std::isfinite(u) || !std::isfinite(v)) return std::nullopt;
  return TruthProjection{entity, name, u, v, p.x, std::hypot(p.x, p.y, p.z)};
}
double CanonicalDiscoveryRange(double range, double maxRange)
{
  if (!std::isfinite(range) || !std::isfinite(maxRange) || range <= 0 || maxRange <= 0 ||
      range <= maxRange)
    return range;
  // 三维坐标变换、差分和hypot的少量浮点运算误差；300m时约1.07e-12m。
  // 不使用米/毫米等固定业务容差，明显超距仍原样保留并拒绝。
  const double roundoff = (16 * std::numeric_limits<double>::epsilon()) *
    std::max(range, maxRange);
  return range - maxRange <= roundoff ? maxRange : range;
}
std::string PreciseNumber(double value)
{
  if (!std::isfinite(value)) return "null";
  char buffer[64];
  const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value,
    std::chars_format::general, std::numeric_limits<double>::max_digits10);
  return result.ec == std::errc{} ? std::string(buffer, result.ptr) : "null";
}
bool CanAcquire(const TruthProjection &p, int width, int height, double maxRange)
{
  return width > 0 && height > 0 && std::isfinite(p.range_m) && p.range_m > 0 &&
    std::isfinite(maxRange) && maxRange > 0 && CanonicalDiscoveryRange(p.range_m, maxRange) <= maxRange &&
    std::isfinite(p.center_x) && std::isfinite(p.center_y) &&
    p.center_x >= 0 && p.center_x < width && p.center_y >= 0 && p.center_y < height;
}

std::optional<std::size_t> SelectTruthTarget(
  const std::vector<TruthProjection> &projections, const Box &selection,
  int width, int height)
{
  if (width <= 0 || height <= 0 || selection.w <= 0 || selection.h <= 0)
    return std::nullopt;
  std::optional<std::size_t> selected;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < projections.size(); ++i)
  {
    const auto &p = projections[i];
    if (!std::isfinite(p.center_x) || !std::isfinite(p.center_y) ||
        p.center_x < 0 || p.center_x > width ||
        p.center_y < 0 || p.center_y > height ||
        p.center_x < selection.x || p.center_x > selection.x + selection.w ||
        p.center_y < selection.y || p.center_y > selection.y + selection.h)
      continue;
    const double distance = std::hypot(p.center_x - selection.Cx(),
                                       p.center_y - selection.Cy());
    if (distance < best)
    {
      best = distance;
      selected = i;
    }
  }
  return selected;
}

TruthObservation MakeTruthObservation(
  const TruthProjection &projection, const Box &initialBox,
  double initialDepth, int width, int height, std::int64_t frameSeq)
{
  TruthObservation observation;
  observation.status = TruthStatus::INVISIBLE;
  observation.frame_seq = frameSeq;
  if (width <= 0 || height <= 0 || projection.center_x < 0 || projection.center_x >= width ||
      projection.center_y < 0 || projection.center_y >= height ||
      initialBox.w <= 0 || initialBox.h <= 0 ||
      !std::isfinite(initialDepth) || initialDepth <= 0 ||
      !std::isfinite(projection.depth) || projection.depth <= 0 ||
      !std::isfinite(projection.center_x) || !std::isfinite(projection.center_y))
    return observation;

  const double scale = initialDepth / projection.depth;
  const double boxWidth = initialBox.w * scale;
  const double boxHeight = initialBox.h * scale;
  if (!std::isfinite(boxWidth) || !std::isfinite(boxHeight) ||
      boxWidth <= 0 || boxHeight <= 0)
    return observation;

  const double rawLeft = std::max(
    0.0, projection.center_x - boxWidth / 2.0);
  const double rawTop = std::max(
    0.0, projection.center_y - boxHeight / 2.0);
  const double rawRight = std::min(
    static_cast<double>(width), projection.center_x + boxWidth / 2.0);
  const double rawBottom = std::min(
    static_cast<double>(height), projection.center_y + boxHeight / 2.0);
  if (rawRight <= rawLeft || rawBottom <= rawTop)
    return observation;

  const double left = std::max(0.0, rawLeft - kTruthBoxPaddingPx);
  const double top = std::max(0.0, rawTop - kTruthBoxPaddingPx);
  const double right = std::min(
    static_cast<double>(width), rawRight + kTruthBoxPaddingPx);
  const double bottom = std::min(
    static_cast<double>(height), rawBottom + kTruthBoxPaddingPx);

  observation.status = TruthStatus::VISIBLE;
  observation.box = {left, top, right - left, bottom - top};
  observation.center_x = projection.center_x;
  observation.center_y = projection.center_y;
  return observation;
}
}
