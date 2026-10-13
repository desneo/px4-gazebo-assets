#include "MapCenter.hh"

#include <atomic>
#include <cmath>
#include <QQuickItem>
#include <QSignalSpy>
#include <QTest>
#include <gz/gui/Application.hh>
#include <gz/msgs/gui_camera.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <tinyxml2.h>

class MapCenterTest : public QObject
{
  Q_OBJECT
  private slots: void buttonRequestsMapCenter()
  {
    gz::transport::Node service;
    std::atomic<int> calls{0};
    gz::msgs::GUICamera received;
    std::function<bool(const gz::msgs::GUICamera &, gz::msgs::Boolean &)> handler =
      [&](const auto &request, auto &response)
      {
        received = request;
        response.set_data(true);
        ++calls;
        return true;
      };
    QVERIFY(service.Advertise("/gui/move_to/pose", handler));
    tinyxml2::XMLDocument config;
    config.Parse(R"(<plugin filename="MapCenter"><center_pose>0 0 200 0 1.5707963267948966 1.5707963267948966</center_pose></plugin>)");
    geographic_world::MapCenter plugin;
    plugin.Load(config.RootElement());
    QVERIFY(plugin.PluginItem());
    auto *button = plugin.PluginItem()->findChild<QQuickItem *>(
        "returnToMapCenterButton");
    QVERIFY(button);
    QCOMPARE(button->property("text").toString(), QString("回到地图中心"));
    QVERIFY(QMetaObject::invokeMethod(button, "clicked", Qt::DirectConnection));
    QTRY_VERIFY_WITH_TIMEOUT(!plugin.Busy(), 4000);
    QCOMPARE(calls.load(), 1);
    QVERIFY(plugin.Status().isEmpty());
    QCOMPARE(received.pose().position().x(), 0.0);
    QCOMPARE(received.pose().position().y(), 0.0);
    QCOMPARE(received.pose().position().z(), 200.0);
    const auto &q = received.pose().orientation();
    // The camera's forward axis points down; its image-up axis points north.
    QVERIFY(std::abs(1-2*(q.y()*q.y()+q.z()*q.z())) < 1e-10);
    QVERIFY(std::abs(2*(q.x()*q.y()+q.w()*q.z())) < 1e-10);
    QVERIFY(std::abs(2*(q.x()*q.z()-q.w()*q.y())+1) < 1e-10);
    QVERIFY(std::abs(2*(q.y()*q.z()-q.w()*q.x())-1) < 1e-10);

    // Repeated clicks while a request is running issue only one move request.
    plugin.ReturnToCenter();
    plugin.ReturnToCenter();
    QTRY_VERIFY_WITH_TIMEOUT(!plugin.Busy(), 4000);
    QCOMPARE(calls.load(), 2);
  }

  private slots: void unavailableCameraReportsFailure()
  {
    geographic_world::MapCenter plugin;
    plugin.ReturnToCenter();
    QTRY_VERIFY_WITH_TIMEOUT(!plugin.Busy(), 4000);
    QVERIFY(!plugin.Status().isEmpty());
  }
};

int main(int argc, char **argv)
{
  gz::gui::Application app(argc, argv, gz::gui::WindowType::kDialog);
  MapCenterTest test;
  return QTest::qExec(&test, argc, argv);
}

#include "MapCenter_test.moc"
