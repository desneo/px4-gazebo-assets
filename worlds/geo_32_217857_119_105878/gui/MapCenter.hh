#pragma once

#include <QFutureWatcher>
#include <QString>
#include <gz/gui/Plugin.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>

namespace geographic_world
{
class MapCenter final : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(bool busy READ Busy NOTIFY StateChanged)
  Q_PROPERTY(QString status READ Status NOTIFY StateChanged)

  public: MapCenter();
  public: ~MapCenter() override;
  public: Q_INVOKABLE void ReturnToCenter();
  public: bool Busy() const { return this->busy; }
  public: QString Status() const { return this->status; }

  signals: void StateChanged();

  protected: void LoadConfig(const tinyxml2::XMLElement *_config) override;

  private: gz::math::Pose3d centerPose{0, 0, 200, 0,
      GZ_PI / 2, GZ_PI / 2};
  private: gz::transport::Node node;
  private: QFutureWatcher<bool> request;
  private: bool busy{false};
  private: QString status;
};
}
