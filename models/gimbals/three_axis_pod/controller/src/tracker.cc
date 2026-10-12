#include "gimbal_controller/tracker.hh"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace gimbal {

namespace {
constexpr int kMinBoxPx = 8;
constexpr double kMinTextureStd = 6.0;
constexpr double kMaxSizeStep = 1.25;
constexpr double kMaxFrameAreaRatio = 0.25;
constexpr double kCenterGateFactor = 1.2;
constexpr double kCenterGateBasePx = 120.0;
constexpr double kSearchMargin = 4.0;
constexpr double kSearchMarginMinPx = 250.0;
constexpr double kMatchThreshold = 0.55;
}  // namespace

cv::Rect Box::ToInt() const
{
  return cv::Rect(static_cast<int>(std::lround(x)),
                  static_cast<int>(std::lround(y)),
                  static_cast<int>(std::lround(w)),
                  static_cast<int>(std::lround(h)));
}

Box Box::Clip(int _width, int _height) const
{
  int x0 = std::max(0, std::min(_width - 1, static_cast<int>(x)));
  int y0 = std::max(0, std::min(_height - 1, static_cast<int>(y)));
  int x1 = std::max(x0 + 1, std::min(_width, static_cast<int>(x + w)));
  int y1 = std::max(y0 + 1, std::min(_height, static_cast<int>(y + h)));
  return Box{static_cast<double>(x0), static_cast<double>(y0),
             static_cast<double>(x1 - x0), static_cast<double>(y1 - y0)};
}

std::pair<bool, std::string> ValidateBox(const cv::Mat &_bgr, const Box &_box)
{
  const int h = _bgr.rows;
  const int w = _bgr.cols;
  const int x0 = std::max(0, static_cast<int>(_box.x));
  const int y0 = std::max(0, static_cast<int>(_box.y));
  const int x1 = std::min(w, static_cast<int>(_box.x + _box.w));
  const int y1 = std::min(h, static_cast<int>(_box.y + _box.h));
  if (x1 - x0 < kMinBoxPx || y1 - y0 < kMinBoxPx)
    return {false, "selection_too_small"};
  const cv::Mat roi = _bgr(cv::Rect(x0, y0, x1 - x0, y1 - y0));
  cv::Mat gray;
  if (roi.channels() == 3)
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
  else
    gray = roi;
  cv::Scalar mean, stddev;
  cv::meanStdDev(gray, mean, stddev);
  if (stddev[0] < kMinTextureStd)
    return {false, "selection_low_texture"};
  return {true, ""};
}

Tracker::Tracker(const std::string &_kind) : kind_(_kind) {}

std::pair<bool, std::string> Tracker::Start(const cv::Mat &_bgr,
                                            const Box &_box)
{
  auto [ok, reason] = ValidateBox(_bgr, _box);
  if (!ok)
    return {false, reason};

  const int h = _bgr.rows;
  const int w = _bgr.cols;
  Box box = _box.Clip(w, h);

  cv::Ptr<cv::Tracker> tracker;
  if (kind_ == "KCF")
    tracker = cv::TrackerKCF::create();
  else
    tracker = cv::TrackerCSRT::create();

  tracker->init(_bgr, box.ToInt());
  cv::Rect rect;
  if (!tracker->update(_bgr, rect))
    return {false, "init_update_failed"};

  tracker_ = tracker;
  last_box_ = Box{static_cast<double>(rect.x), static_cast<double>(rect.y),
                  static_cast<double>(rect.width),
                  static_cast<double>(rect.height)};
  has_last_box_ = true;

  cv::Rect ri = last_box_.ToInt();
  const int gx = std::max(0, ri.x);
  const int gy = std::max(0, ri.y);
  const int gw = std::min(w - gx, std::max(1, ri.width));
  const int gh = std::min(h - gy, std::max(1, ri.height));
  if (gw <= 0 || gh <= 0)
    return {false, "init_update_failed"};
  cv::Mat roi = _bgr(cv::Rect(gx, gy, gw, gh));
  if (roi.channels() == 3)
    cv::cvtColor(roi, gray_template_, cv::COLOR_BGR2GRAY);
  else
    gray_template_ = roi.clone();
  template_w_ = gw;
  template_h_ = gh;
  return {true, ""};
}

bool Tracker::BoxSane(const Box &_box, int _w, int _h) const
{
  if (_box.w < kMinBoxPx || _box.h < kMinBoxPx)
    return false;
  if (_box.Cx() < -_box.w || _box.Cx() > _w + _box.w ||
      _box.Cy() < -_box.h || _box.Cy() > _h + _box.h)
    return false;
  if (_box.w * _box.h > kMaxFrameAreaRatio * _w * _h)
    return false;
  return true;
}

std::pair<bool, Box> Tracker::Update(const cv::Mat &_bgr)
{
  if (tracker_ == nullptr)
    return {false, Box{}};
  const int h = _bgr.rows;
  const int w = _bgr.cols;
  cv::Rect rect;
  if (tracker_->update(_bgr, rect))
  {
    Box raw{static_cast<double>(rect.x), static_cast<double>(rect.y),
            static_cast<double>(rect.width), static_cast<double>(rect.height)};
    Box box = raw;
    if (has_last_box_)
    {
      const Box &last = last_box_;
      const double nw = std::min(std::max(raw.w, last.w / kMaxSizeStep),
                                 last.w * kMaxSizeStep);
      const double nh = std::min(std::max(raw.h, last.h / kMaxSizeStep),
                                 last.h * kMaxSizeStep);
      const double gate =
          kCenterGateFactor * std::max(last.w, last.h) + kCenterGateBasePx;
      const double dx = raw.Cx() - last.Cx();
      const double dy = raw.Cy() - last.Cy();
      const double dist = std::hypot(dx, dy);
      double cx = raw.Cx();
      double cy = raw.Cy();
      if (dist > gate && dist > 0.0)
      {
        const double f = gate / dist;
        cx = last.Cx() + dx * f;
        cy = last.Cy() + dy * f;
      }
      box = Box{cx - nw / 2.0, cy - nh / 2.0, nw, nh};
    }
    if (BoxSane(box, w, h))
    {
      last_box_ = box;
      has_last_box_ = true;
      return {true, box};
    }
  }
  return {false, Box{}};
}

std::pair<bool, Box> Tracker::RetryCapture(const cv::Mat &_bgr)
{
  if (gray_template_.empty() || !has_last_box_)
    return {false, Box{}};
  const int h = _bgr.rows;
  const int w = _bgr.cols;
  const int bw = template_w_;
  const int bh = template_h_;
  const double cx = last_box_.Cx();
  const double cy = last_box_.Cy();
  const double side = std::max(bw, bh);
  const double margin = std::max(kSearchMargin * side, kSearchMarginMinPx);
  const int x0 = static_cast<int>(std::max(0.0, cx - margin));
  const int y0 = static_cast<int>(std::max(0.0, cy - margin));
  const int x1 = static_cast<int>(std::min<double>(w, cx + margin));
  const int y1 = static_cast<int>(std::min<double>(h, cy + margin));
  if (x1 - x0 < bw || y1 - y0 < bh)
    return {false, Box{}};

  cv::Mat gray;
  if (_bgr.channels() == 3)
    cv::cvtColor(_bgr, gray, cv::COLOR_BGR2GRAY);
  else
    gray = _bgr;
  cv::Mat search = gray(cv::Rect(x0, y0, x1 - x0, y1 - y0));
  cv::Mat res;
  cv::matchTemplate(search, gray_template_, res, cv::TM_CCOEFF_NORMED);
  double maxVal = 0.0;
  cv::Point maxLoc;
  cv::minMaxLoc(res, nullptr, &maxVal, nullptr, &maxLoc);
  if (maxVal < kMatchThreshold)
    return {false, Box{}};
  Box box{static_cast<double>(x0 + maxLoc.x), static_cast<double>(y0 + maxLoc.y),
          static_cast<double>(bw), static_cast<double>(bh)};
  const cv::Rect g = box.ToInt();
  if (g.width >= kMinBoxPx && g.height >= kMinBoxPx && g.x >= 0 && g.x < w &&
      g.y >= 0 && g.y < h)
    return {true, box};
  return {false, Box{}};
}

bool Tracker::Adopt(const cv::Mat &_bgr, const Box &_box)
{
  auto [ok, reason] = Start(_bgr, _box);
  (void)reason;
  return ok;
}

}  // namespace gimbal
