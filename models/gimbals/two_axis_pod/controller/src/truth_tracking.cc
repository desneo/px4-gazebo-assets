#include "two_axis/truth_tracking.hh"

#include <algorithm>
#include <cmath>
#include <limits>

namespace two_axis
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
  return TruthProjection{entity, name, u, v, p.x};
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
  if (width <= 0 || height <= 0 || initialBox.w <= 0 || initialBox.h <= 0 ||
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

  const double left = std::max(0.0, projection.center_x - boxWidth / 2.0);
  const double top = std::max(0.0, projection.center_y - boxHeight / 2.0);
  const double right = std::min(static_cast<double>(width),
                                projection.center_x + boxWidth / 2.0);
  const double bottom = std::min(static_cast<double>(height),
                                 projection.center_y + boxHeight / 2.0);
  if (right <= left || bottom <= top)
    return observation;

  observation.status = TruthStatus::VISIBLE;
  observation.box = {left, top, right - left, bottom - top};
  observation.center_x = projection.center_x;
  observation.center_y = projection.center_y;
  return observation;
}
}
