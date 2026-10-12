// 视觉跟踪：OpenCV CSRT + 模板匹配重捕获。
#ifndef GIMBAL_CONTROLLER_TRACKER_HH_
#define GIMBAL_CONTROLLER_TRACKER_HH_

#include <string>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>

namespace gimbal {

struct Box
{
  double x{0.0};
  double y{0.0};
  double w{0.0};
  double h{0.0};

  double Cx() const { return x + w / 2.0; }
  double Cy() const { return y + h / 2.0; }
  cv::Rect ToInt() const;
  Box Clip(int _width, int _height) const;
};

/// 选框有效性检查（尺寸 + 纹理）；失败返回 (false, reason)。
std::pair<bool, std::string> ValidateBox(const cv::Mat &_bgr, const Box &_box);

class Tracker
{
 public:
  explicit Tracker(const std::string &_kind = "CSRT");

  /// 初始化跟踪器（含一次 update 验证与模板抓取）。失败返回 reason。
  std::pair<bool, std::string> Start(const cv::Mat &_bgr, const Box &_box);

  /// 跟踪一帧。返回是否有效，有效时输出 box。
  std::pair<bool, Box> Update(const cv::Mat &_bgr);

  /// 模板匹配重捕获（暂时丢失时使用）。
  std::pair<bool, Box> RetryCapture(const cv::Mat &_bgr);

  /// 用给定箱体重新初始化（重捕获成功后调用）。
  bool Adopt(const cv::Mat &_bgr, const Box &_box);

  bool ready() const { return tracker_ != nullptr; }
  const Box &last_box() const { return last_box_; }
  bool has_last_box() const { return has_last_box_; }

 private:
  bool BoxSane(const Box &_box, int _w, int _h) const;

  std::string kind_;
  cv::Ptr<cv::Tracker> tracker_;
  bool has_last_box_{false};
  Box last_box_;
  cv::Mat gray_template_;
  int template_w_{0};
  int template_h_{0};
};

}  // namespace gimbal

#endif  // GIMBAL_CONTROLLER_TRACKER_HH_
