#pragma once

#include <cmath>
#include <algorithm>
#include <array>
#include "gimbal_controller/tracker.hh"

namespace two_axis
{
// 阈值使用 1920×1080 相机原图像素，任一边严格超过阈值即切广角。
inline bool ShouldSwitchToWide(bool tele, bool trackingValid, const gimbal::Box &box)
{
  return tele && trackingValid && std::isfinite(box.w) &&
    std::isfinite(box.h) && box.w > 0 && box.h > 0 &&
    (box.w > 500.0 || box.h > 300.0);
}

inline double CameraFocalRatio(double fromHfov, double toHfov)
{
  return std::tan(fromHfov / 2.0) / std::tan(toHfov / 2.0);
}

inline std::array<double, 4> ReprojectTrackingBox(
  const gimbal::Box &box, int width, int height, double ratio)
{
  const double cx = width / 2.0 + (box.Cx() - width / 2.0) * ratio;
  const double cy = height / 2.0 + (box.Cy() - height / 2.0) * ratio;
  const double left = std::clamp(cx - box.w * ratio / 2.0, 0.0,
                                 static_cast<double>(width));
  const double top = std::clamp(cy - box.h * ratio / 2.0, 0.0,
                                static_cast<double>(height));
  const double right = std::clamp(cx + box.w * ratio / 2.0, left,
                                  static_cast<double>(width));
  const double bottom = std::clamp(cy + box.h * ratio / 2.0, top,
                                   static_cast<double>(height));
  return {left / width, top / height,
          (right - left) / width, (bottom - top) / height};
}
}
