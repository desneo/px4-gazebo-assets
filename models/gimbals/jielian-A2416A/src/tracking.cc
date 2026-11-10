#include "tracking.hh"

#include <algorithm>
#include <cmath>
#include <limits>

namespace jielian_a2416a
{
namespace
{
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

bool Finite(const Vec3 &value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

double CenterDistance(const Detection &d, const Optics &o)
{
  return std::hypot(d.centerX - o.cx, d.centerY - o.cy);
}
}

std::optional<Detection> ProjectBounds(
  std::uint64_t entity, const std::string &name,
  const std::array<Vec3, 8> &corners, const Vec3 &center,
  const Optics &o)
{
  if (name.empty() || o.width <= 0 || o.height <= 0 ||
      !std::isfinite(o.fx) || !std::isfinite(o.fy) || o.fx <= 0 || o.fy <= 0 ||
      !std::isfinite(o.cx) || !std::isfinite(o.cy) ||
      !std::isfinite(o.nearClip) || !std::isfinite(o.farClip) ||
      o.nearClip <= 0 || o.farClip <= o.nearClip || !Finite(center) ||
      center.x < o.nearClip || center.x > o.farClip)
    return std::nullopt;

  const double centerX = o.cx - o.fx * center.y / center.x;
  const double centerY = o.cy - o.fy * center.z / center.x;
  if (!std::isfinite(centerX) || !std::isfinite(centerY) || centerX < 0 ||
      centerX >= o.width || centerY < 0 || centerY >= o.height)
    return std::nullopt;

  double left = std::numeric_limits<double>::infinity();
  double top = std::numeric_limits<double>::infinity();
  double right = -std::numeric_limits<double>::infinity();
  double bottom = -std::numeric_limits<double>::infinity();
  std::size_t projected = 0;
  for (const auto &corner : corners)
  {
    if (!Finite(corner) || corner.x < o.nearClip || corner.x > o.farClip)
      continue;
    const double u = o.cx - o.fx * corner.y / corner.x;
    const double v = o.cy - o.fy * corner.z / corner.x;
    if (!std::isfinite(u) || !std::isfinite(v))
      continue;
    left = std::min(left, u);
    top = std::min(top, v);
    right = std::max(right, u);
    bottom = std::max(bottom, v);
    ++projected;
  }
  if (projected == 0)
    return std::nullopt;
  left = std::clamp(left, 0.0, static_cast<double>(o.width - 1));
  right = std::clamp(right, 0.0, static_cast<double>(o.width - 1));
  top = std::clamp(top, 0.0, static_cast<double>(o.height - 1));
  bottom = std::clamp(bottom, 0.0, static_cast<double>(o.height - 1));
  if (right <= left || bottom <= top)
    return std::nullopt;
  return Detection{entity, name, {left, top, right - left, bottom - top},
                   centerX, centerY,
                   std::hypot(center.x, center.y, center.z), 1.0, 4};
}

const char *ToString(TrackState state)
{
  switch (state)
  {
    case TrackState::IDLE: return "IDLE";
    case TrackState::DETECTING: return "DETECTING";
    case TrackState::TRACKING: return "TRACKING";
    case TrackState::TEMP_LOST: return "TEMP_LOST";
    case TrackState::LOST: return "LOST";
  }
  return "IDLE";
}

bool TruthTracker::Configure(const TrackerConfig &config)
{
  if (config.optics.width <= 0 || config.optics.height <= 0 ||
      config.autoRangeM <= 0 || config.manualRangeM < config.autoRangeM ||
      config.lostTimeoutS <= 0 || config.maxTargets != 4 ||
      config.misalignmentLimitDeg <= 0 || config.crosshairMaxSpeedPxS <= 0 ||
      config.crosshairTimeoutS <= 0 || config.gateScaleMin <= 0 ||
      config.gateScaleMax < config.gateScaleMin)
    return false;
  config_ = config;
  crosshair_ = {config.optics.cx, config.optics.cy};
  configured_ = true;
  return true;
}

void TruthTracker::Update(double now, std::vector<Detection> detections)
{
  if (!configured_ || !std::isfinite(now))
    return;
  const double dt = lastUpdate_ >= 0 && now >= lastUpdate_ ? now - lastUpdate_ : 0;
  lastUpdate_ = now;
  if (crosshairCommandTime_ < 0 || now - crosshairCommandTime_ > config_.crosshairTimeoutS)
    crosshairVelocity_ = {0, 0};
  crosshair_[0] = std::clamp(crosshair_[0] + crosshairVelocity_[0] * dt,
                             0.0, static_cast<double>(config_.optics.width - 1));
  crosshair_[1] = std::clamp(crosshair_[1] + crosshairVelocity_[1] * dt,
                             0.0, static_cast<double>(config_.optics.height - 1));

  detections.erase(std::remove_if(detections.begin(), detections.end(),
    [&](const Detection &d)
    {
      return d.name.empty() || !std::isfinite(d.rangeM) || d.rangeM <= 0 ||
        d.rangeM > config_.manualRangeM || d.centerX < 0 ||
        d.centerX >= config_.optics.width || d.centerY < 0 ||
        d.centerY >= config_.optics.height;
    }), detections.end());
  std::sort(detections.begin(), detections.end(), [&](const Detection &a, const Detection &b)
  {
    const double da = CenterDistance(a, config_.optics);
    const double db = CenterDistance(b, config_.optics);
    if (std::abs(da - db) > 1e-9)
      return da < db;
    return a.name < b.name;
  });
  allVisible_ = detections;
  std::vector<Detection> automatic;
  if (recognitionEnabled_)
    std::copy_if(detections.begin(), detections.end(), std::back_inserter(automatic),
                 [&](const Detection &d) { return d.rangeM <= config_.autoRangeM; });

  if (locked_)
  {
    const double range = lockMode_ == LockMode::AUTO ? config_.autoRangeM : config_.manualRangeM;
    const auto current = std::find_if(detections.begin(), detections.end(),
      [&](const Detection &d) { return d.name == locked_->name && d.rangeM <= range; });
    if (current != detections.end())
    {
      locked_ = *current;
      state_ = TrackState::TRACKING;
      lostSince_ = -1;
    }
    else if (lostSince_ < 0)
    {
      lostSince_ = now;
      state_ = TrackState::TEMP_LOST;
    }
    else if (now - lostSince_ >= config_.lostTimeoutS)
    {
      locked_.reset();
      lockMode_ = LockMode::NONE;
      aimOffset_ = {0, 0};
      gateScale_ = 1.0;
      lostSince_ = -1;
      state_ = TrackState::LOST;
    }
  }

  if (!locked_ && autoLockEnabled_ && recognitionEnabled_ && !automatic.empty())
    Lock(automatic.front(), LockMode::AUTO, now);
  else if (!locked_ && state_ != TrackState::LOST)
    state_ = recognitionEnabled_ ? TrackState::DETECTING : TrackState::IDLE;

  BuildSlots(automatic);
  ++measurementSeq_;
}

bool TruthTracker::SetRecognition(bool enabled)
{
  recognitionEnabled_ = enabled;
  if (!enabled)
    autoLockEnabled_ = false;
  if (!enabled && lockMode_ == LockMode::AUTO)
  {
    locked_.reset();
    lockMode_ = LockMode::NONE;
    state_ = TrackState::IDLE;
  }
  else if (!locked_)
    state_ = enabled ? TrackState::DETECTING : TrackState::IDLE;
  return true;
}

bool TruthTracker::SetAutoLock(bool enabled, double now)
{
  autoLockEnabled_ = enabled;
  if (!enabled && lockMode_ == LockMode::AUTO)
  {
    locked_.reset();
    lockMode_ = LockMode::NONE;
    state_ = recognitionEnabled_ ? TrackState::DETECTING : TrackState::IDLE;
  }
  if (enabled && recognitionEnabled_)
  {
    const auto candidate = std::find_if(allVisible_.begin(), allVisible_.end(),
      [&](const Detection &d) { return d.rangeM <= config_.autoRangeM; });
    if (!locked_ && candidate != allVisible_.end())
      Lock(*candidate, LockMode::AUTO, now);
  }
  return true;
}

bool TruthTracker::Lock(const Detection &detection, LockMode mode, double now)
{
  if (detection.rangeM <= 0 || detection.rangeM >
      (mode == LockMode::AUTO ? config_.autoRangeM : config_.manualRangeM))
    return false;
  locked_ = detection;
  lockMode_ = mode;
  if (mode == LockMode::MANUAL)
    autoLockEnabled_ = false;
  state_ = TrackState::TRACKING;
  lostSince_ = -1;
  aimOffset_ = {0, 0};
  gateScale_ = 1.0;
  (void)now;
  return true;
}

bool TruthTracker::PointLock(double x, double y, double now)
{
  const Detection *selected = nullptr;
  double best = std::numeric_limits<double>::infinity();
  for (const auto &d : allVisible_)
  {
    if (d.rangeM > config_.manualRangeM || x < d.box.x || x > d.box.x + d.box.w ||
        y < d.box.y || y > d.box.y + d.box.h)
      continue;
    const double distance = std::hypot(d.centerX - x, d.centerY - y);
    if (distance < best)
    {
      selected = &d;
      best = distance;
    }
  }
  return selected && Lock(*selected, LockMode::MANUAL, now);
}

double TruthTracker::IntersectionArea(const Box &a, const Box &b)
{
  const double width = std::max(0.0, std::min(a.x + a.w, b.x + b.w) - std::max(a.x, b.x));
  const double height = std::max(0.0, std::min(a.y + a.h, b.y + b.h) - std::max(a.y, b.y));
  return width * height;
}

bool TruthTracker::BoxLock(const Box &selection, double now)
{
  if (selection.w <= 0 || selection.h <= 0)
    return false;
  const Detection *selected = nullptr;
  double best = 0;
  for (const auto &d : allVisible_)
  {
    if (d.rangeM > config_.manualRangeM)
      continue;
    const double area = IntersectionArea(selection, d.box);
    if (area > best)
    {
      selected = &d;
      best = area;
    }
  }
  return selected && Lock(*selected, LockMode::MANUAL, now);
}

bool TruthTracker::CrosshairLock(double now)
{
  return PointLock(crosshair_[0], crosshair_[1], now);
}

void TruthTracker::Cancel()
{
  autoLockEnabled_ = false;
  locked_.reset();
  lockMode_ = LockMode::NONE;
  lostSince_ = -1;
  aimOffset_ = {0, 0};
  gateScale_ = 1.0;
  state_ = recognitionEnabled_ ? TrackState::DETECTING : TrackState::IDLE;
}

void TruthTracker::MoveCrosshair(double xPercent, double yPercent, double now)
{
  xPercent = std::clamp(xPercent, -100.0, 100.0);
  yPercent = std::clamp(yPercent, -100.0, 100.0);
  crosshairVelocity_[0] = config_.crosshairMaxSpeedPxS * xPercent / 100.0;
  crosshairVelocity_[1] = config_.crosshairMaxSpeedPxS * yPercent / 100.0;
  crosshairCommandTime_ = now;
}

bool TruthTracker::TrimAim(double dx, double dy)
{
  if (!locked_ || !std::isfinite(dx) || !std::isfinite(dy))
    return false;
  const double minX = locked_->box.x - locked_->centerX;
  const double maxX = locked_->box.x + locked_->box.w - locked_->centerX;
  const double minY = locked_->box.y - locked_->centerY;
  const double maxY = locked_->box.y + locked_->box.h - locked_->centerY;
  aimOffset_[0] = std::clamp(aimOffset_[0] + dx, minX, maxX);
  aimOffset_[1] = std::clamp(aimOffset_[1] + dy, minY, maxY);
  return true;
}

void TruthTracker::ScaleGate(double factor)
{
  if (locked_ && std::isfinite(factor) && factor > 0)
    gateScale_ = std::clamp(gateScale_ * factor, config_.gateScaleMin, config_.gateScaleMax);
}

void TruthTracker::BuildSlots(const std::vector<Detection> &automatic)
{
  slots_.clear();
  if (locked_)
    slots_.push_back(*locked_);
  if (recognitionEnabled_)
  {
    for (const auto &d : automatic)
    {
      if (locked_ && d.name == locked_->name)
        continue;
      slots_.push_back(d);
      if (slots_.size() == config_.maxTargets)
        break;
    }
  }
}

std::array<double, 2> TruthTracker::MisalignmentDeg() const
{
  if (!locked_)
    return {0, 0};
  const double x = locked_->centerX + aimOffset_[0];
  const double y = locked_->centerY + aimOffset_[1];
  const double yaw = std::atan2(x - config_.optics.cx, config_.optics.fx) * kRadToDeg;
  const double pitch = std::atan2(config_.optics.cy - y, config_.optics.fy) * kRadToDeg;
  return {std::clamp(pitch, -config_.misalignmentLimitDeg, config_.misalignmentLimitDeg),
          std::clamp(yaw, -config_.misalignmentLimitDeg, config_.misalignmentLimitDeg)};
}

std::optional<Box> TruthTracker::Gate() const
{
  if (!locked_)
    return std::nullopt;
  const double w = std::min(static_cast<double>(config_.optics.width), locked_->box.w * gateScale_);
  const double h = std::min(static_cast<double>(config_.optics.height), locked_->box.h * gateScale_);
  const double centerX = locked_->centerX + aimOffset_[0];
  const double centerY = locked_->centerY + aimOffset_[1];
  const double x = std::clamp(centerX - w / 2.0, 0.0,
                              static_cast<double>(config_.optics.width) - w);
  const double y = std::clamp(centerY - h / 2.0, 0.0,
                              static_cast<double>(config_.optics.height) - h);
  return Box{x, y, w, h};
}
}
