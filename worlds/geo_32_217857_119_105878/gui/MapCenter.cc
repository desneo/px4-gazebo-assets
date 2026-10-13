#include "MapCenter.hh"

#include <sstream>
#include <QtConcurrent/QtConcurrentRun>
#include <gz/common/KeyEvent.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/gui_camera.pb.h>
#include <gz/plugin/Register.hh>
#include <tinyxml2.h>

namespace geographic_world
{
MapCenter::MapCenter()
{
  connect(&this->request, &QFutureWatcher<bool>::finished, this, [this]
  {
    this->busy = false;
    this->status = this->request.result() ? QString{} :
        QStringLiteral("视角尚未就绪，请稍后重试");
    emit this->StateChanged();
  });
}

MapCenter::~MapCenter()
{
  // Keep the transport node alive until the bounded request completes.
  this->request.waitForFinished();
}

void MapCenter::LoadConfig(const tinyxml2::XMLElement *_config)
{
  if (this->title.empty())
    this->title = "地图中心";
  if (_config)
  {
    const auto *pose = _config->FirstChildElement("center_pose");
    if (pose && pose->GetText())
    {
      std::istringstream input(pose->GetText());
      gz::math::Pose3d value;
      if (input >> value && value.IsFinite())
        this->centerPose = value;
    }
  }
}

void MapCenter::ReturnToCenter()
{
  if (this->busy)
    return;

  // Clear CameraTracking's follow / track state through its normal Escape
  // handler before requesting a new view. Scene entities are unaffected.
  if (auto *window = gz::gui::App()->findChild<gz::gui::MainWindow *>())
  {
    gz::common::KeyEvent key;
    key.SetKey(Qt::Key_Escape);
    key.SetType(gz::common::KeyEvent::RELEASE);
    gz::gui::events::KeyReleaseOnScene escape(key);
    QCoreApplication::sendEvent(window, &escape);
  }

  gz::msgs::GUICamera message;
  gz::msgs::Set(message.mutable_pose(), this->centerPose);
  this->busy = true;
  this->status.clear();
  emit this->StateChanged();
  this->request.setFuture(QtConcurrent::run([this, message]
  {
    gz::msgs::Boolean response;
    bool result = false;
    return this->node.Request("/gui/move_to/pose", message, 1500,
        response, result) && result && response.data();
  }));
}
}

GZ_ADD_PLUGIN(geographic_world::MapCenter, gz::gui::Plugin)
