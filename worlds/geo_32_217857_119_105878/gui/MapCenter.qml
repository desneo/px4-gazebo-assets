import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

ColumnLayout {
  anchors.fill: parent
  anchors.margins: 6
  Layout.minimumWidth: 166
  Layout.minimumHeight: statusLabel.visible ? 90 : 56

  Button {
    objectName: "returnToMapCenterButton"
    Layout.fillWidth: true
    text: "回到地图中心"
    enabled: !MapCenter.busy
    onClicked: MapCenter.ReturnToCenter()
    ToolTip.visible: hovered
    ToolTip.text: "回到中心点上方 200 米，俯视地图，北向上"
  }

  Label {
    id: statusLabel
    Layout.fillWidth: true
    visible: text.length > 0
    text: MapCenter.status
    wrapMode: Text.WordWrap
    font.pixelSize: 12
  }
}
