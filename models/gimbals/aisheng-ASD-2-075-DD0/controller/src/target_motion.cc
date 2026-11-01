#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <sdf/Element.hh>

namespace aisheng
{
/// 以仿真步长平滑移动静态目标模型，用于视线角速度验收。
class TargetMotion : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    auto get = [&](const char *_k, const std::string & _def)
    {
      return _sdf->Get<std::string>(_k, _def).first;
    };
    auto getd = [&](const char *_k, double _def)
    {
      return _sdf->Get<double>(_k, _def).first;
    };
    targetName_ = get("target_name", "target_aircraft");
    centerX_ = getd("center_x", 3.7);
    centerY_ = getd("center_y", 0.55);
    centerZ_ = getd("center_z", 1.5);
    ampY_ = getd("amp_y", 0.55);
    ampZ_ = getd("amp_z", 0.0);
    periodY_ = getd("period_y", 2.0);
    periodZ_ = getd("period_z", 8.0);
    phaseY_ = getd("phase_y", 0.0);
    phaseZ_ = getd("phase_z", 0.0);
    targetEntity_ = _ecm.EntityByComponents(
      gz::sim::components::Model(),
      gz::sim::components::Name(targetName_));
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    if (targetEntity_ == gz::sim::kNullEntity)
    {
      targetEntity_ = _ecm.EntityByComponents(
        gz::sim::components::Model(),
        gz::sim::components::Name(targetName_));
      if (targetEntity_ == gz::sim::kNullEntity)
        return;
    }
    const double t = std::chrono::duration<double>(_info.simTime).count();
    const double y = centerY_ + ampY_ * std::sin(
      2.0 * M_PI * t / std::max(1e-6, periodY_) + phaseY_);
    const double z = centerZ_ + ampZ_ * std::sin(
      2.0 * M_PI * t / std::max(1e-6, periodZ_) + phaseZ_);
    const gz::math::Pose3d pose(centerX_, y, z, 0.0, 0.0, 0.0);
    auto cmd = _ecm.Component<gz::sim::components::WorldPoseCmd>(targetEntity_);
    if (cmd)
      cmd->Data() = pose;
    else
      _ecm.CreateComponent(targetEntity_,
                           gz::sim::components::WorldPoseCmd(pose));
    auto current = _ecm.Component<gz::sim::components::Pose>(targetEntity_);
    if (current)
      current->Data() = pose;
    else
      _ecm.CreateComponent(targetEntity_,
                           gz::sim::components::Pose(pose));
  }

private:
  std::string targetName_{"target_aircraft"};
  gz::sim::Entity targetEntity_{gz::sim::kNullEntity};
  double centerX_{3.7}, centerY_{0.55}, centerZ_{1.5};
  double ampY_{0.55}, ampZ_{0.0};
  double periodY_{2.0}, periodZ_{8.0};
  double phaseY_{0.0}, phaseZ_{0.0};
};
}  // namespace aisheng

GZ_ADD_PLUGIN(aisheng::TargetMotion, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(aisheng::TargetMotion, "aisheng::TargetMotion")
