#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Box.hh>
#include <sdf/Camera.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Sphere.hh>

#include "tracking.hh"

namespace jielian_a2416a
{
namespace
{
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

std::string Escape(const std::string &text)
{
  std::ostringstream out;
  for (const unsigned char c : text)
  {
    switch (c)
    {
      case '\\': out << "\\\\"; break;
      case '"': out << "\\\""; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20)
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(c) << std::dec;
        else
          out << static_cast<char>(c);
    }
  }
  return out.str();
}

std::optional<std::string> StringField(const std::string &json, const std::string &key)
{
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;
  if (std::regex_search(json, match, pattern))
    return match[1].str();
  return std::nullopt;
}

double NumberField(const std::string &json, const std::string &key, double fallback = 0)
{
  const std::regex pattern("\\\"" + key +
    "\\\"\\s*:\\s*(-?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][+-]?[0-9]+)?)");
  std::smatch match;
  if (!std::regex_search(json, match, pattern))
    return fallback;
  try
  {
    const double value = std::stod(match[1].str());
    return std::isfinite(value) ? value : fallback;
  }
  catch (...)
  {
    return fallback;
  }
}

struct Bounds
{
  gz::math::Vector3d min;
  gz::math::Vector3d max;
};

std::optional<gz::math::Vector3d> GeometryHalfExtents(const sdf::Geometry &geometry)
{
  switch (geometry.Type())
  {
    case sdf::GeometryType::BOX:
      if (const auto *shape = geometry.BoxShape())
        return shape->Size() / 2.0;
      break;
    case sdf::GeometryType::CYLINDER:
      if (const auto *shape = geometry.CylinderShape())
        return gz::math::Vector3d(shape->Radius(), shape->Radius(), shape->Length() / 2.0);
      break;
    case sdf::GeometryType::SPHERE:
      if (const auto *shape = geometry.SphereShape())
        return gz::math::Vector3d(shape->Radius(), shape->Radius(), shape->Radius());
      break;
    case sdf::GeometryType::CONE:
      if (const auto *shape = geometry.ConeShape())
        return gz::math::Vector3d(shape->Radius(), shape->Radius(), shape->Length() / 2.0);
      break;
    case sdf::GeometryType::CAPSULE:
      if (const auto *shape = geometry.CapsuleShape())
        return gz::math::Vector3d(shape->Radius(), shape->Radius(),
                                  shape->Length() / 2.0 + shape->Radius());
      break;
    default: break;
  }
  return std::nullopt;
}

void Merge(Bounds &bounds, const gz::math::AxisAlignedBox &box, bool &valid)
{
  if (!box.Min().IsFinite() || !box.Max().IsFinite() ||
      box.Min().X() > box.Max().X() || box.Min().Y() > box.Max().Y() ||
      box.Min().Z() > box.Max().Z())
    return;
  bounds.min.X(std::min(bounds.min.X(), box.Min().X()));
  bounds.min.Y(std::min(bounds.min.Y(), box.Min().Y()));
  bounds.min.Z(std::min(bounds.min.Z(), box.Min().Z()));
  bounds.max.X(std::max(bounds.max.X(), box.Max().X()));
  bounds.max.Y(std::max(bounds.max.Y(), box.Max().Y()));
  bounds.max.Z(std::max(bounds.max.Z(), box.Max().Z()));
  valid = true;
}

std::array<gz::math::Vector3d, 8> Corners(const Bounds &bounds)
{
  std::array<gz::math::Vector3d, 8> result;
  std::size_t index = 0;
  for (const double x : {bounds.min.X(), bounds.max.X()})
    for (const double y : {bounds.min.Y(), bounds.max.Y()})
      for (const double z : {bounds.min.Z(), bounds.max.Z()})
        result[index++] = {x, y, z};
  return result;
}

int WorkState(TrackState state)
{
  switch (state)
  {
    case TrackState::IDLE: return 0;
    case TrackState::DETECTING: return 1;
    case TrackState::TRACKING: return 2;
    case TrackState::TEMP_LOST: return 3;
    case TrackState::LOST: return 4;
  }
  return 0;
}
}

class StrapdownController final : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override;
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override;

private:
  void OnCommand(const gz::msgs::StringMsg &message);
  void Result(const std::string &command, bool ok, const std::string &detail);
  std::optional<Bounds> ModelBounds(gz::sim::Entity model,
                                    const gz::sim::EntityComponentManager &ecm) const;
  void Publish(double now, const gz::math::Pose3d &modelPose);

  gz::sim::Entity modelEntity_{gz::sim::kNullEntity};
  gz::sim::Entity worldEntity_{gz::sim::kNullEntity};
  gz::sim::Entity sensorEntity_{gz::sim::kNullEntity};
  std::vector<std::string> targetNames_;
  std::unordered_set<std::string> warnedUnsupported_;
  std::string commandTopic_{"/jielian_a2416a/command"};
  std::string telemetryTopic_{"/jielian_a2416a/telemetry"};
  std::string resultTopic_{"/jielian_a2416a/command_result"};
  double measurementHz_{50};
  double lastMeasurement_{-1};
  double lastSimTime_{0};
  Optics optics_;
  TruthTracker tracker_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher telemetryPublisher_;
  gz::transport::Node::Publisher resultPublisher_;
  std::mutex mutex_;
  bool configured_{false};
};

void StrapdownController::Configure(
  const gz::sim::Entity &entity,
  const std::shared_ptr<const sdf::Element> &sdf,
  gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &)
{
  modelEntity_ = entity;
  auto text = [&](const char *key, const std::string &fallback)
  {
    return sdf->HasElement(key) ? sdf->Get<std::string>(key) : fallback;
  };
  auto number = [&](const char *key, double fallback)
  {
    return sdf->HasElement(key) ? sdf->Get<double>(key) : fallback;
  };
  commandTopic_ = text("command_topic", commandTopic_);
  telemetryTopic_ = text("telemetry_topic", telemetryTopic_);
  resultTopic_ = text("command_result_topic", resultTopic_);
  measurementHz_ = number("measurement_hz", measurementHz_);
  const std::string linkName = text("camera_link", "seeker_link");
  const std::string sensorName = text("camera_sensor", "camera");
  for (auto child = sdf->GetFirstElement(); child; child = child->GetNextElement())
    if (child->GetName() == "sim_truth_target_name")
    {
      const std::string name = child->Get<std::string>();
      if (!name.empty() && std::find(targetNames_.begin(), targetNames_.end(), name) == targetNames_.end())
        targetNames_.push_back(name);
    }

  gz::sim::Model model(entity);
  const auto linkEntity = model.LinkByName(ecm, linkName);
  if (linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[jielian_a2416a] 未找到相机 link [" << linkName << "]\n";
    return;
  }
  gz::sim::Link link(linkEntity);
  sensorEntity_ = link.SensorByName(ecm, sensorName);
  const auto *cameraComponent = ecm.Component<gz::sim::components::Camera>(sensorEntity_);
  if (sensorEntity_ == gz::sim::kNullEntity || !cameraComponent ||
      !cameraComponent->Data().CameraSensor())
  {
    gzerr << "[jielian_a2416a] 未找到相机 sensor [" << sensorName << "]\n";
    return;
  }
  const auto *camera = cameraComponent->Data().CameraSensor();
  if (!camera->HasLensIntrinsics())
  {
    gzerr << "[jielian_a2416a] 相机必须显式配置 fx/fy/cx/cy\n";
    return;
  }
  optics_ = {static_cast<int>(camera->ImageWidth()), static_cast<int>(camera->ImageHeight()),
             camera->LensIntrinsicsFx(), camera->LensIntrinsicsFy(),
             camera->LensIntrinsicsCx(), camera->LensIntrinsicsCy(),
             camera->NearClip(), camera->FarClip()};
  TrackerConfig trackerConfig;
  trackerConfig.optics = optics_;
  trackerConfig.autoRangeM = number("auto_detection_range_m", 780);
  trackerConfig.manualRangeM = number("manual_tracking_range_m", 1200);
  trackerConfig.lostTimeoutS = number("lost_timeout_s", 2.5);
  trackerConfig.maxTargets = static_cast<std::size_t>(number("max_targets", 4));
  trackerConfig.misalignmentLimitDeg = number("misalignment_limit_deg", 20);
  trackerConfig.crosshairMaxSpeedPxS = number("crosshair_max_speed_px_s", 400);
  trackerConfig.crosshairTimeoutS = number("crosshair_timeout_s", .5);
  trackerConfig.gateScaleMin = number("gate_scale_min", .5);
  trackerConfig.gateScaleMax = number("gate_scale_max", 4);
  if (!std::isfinite(measurementHz_) || measurementHz_ <= 0 || targetNames_.empty() ||
      !tracker_.Configure(trackerConfig))
  {
    gzerr << "[jielian_a2416a] 跟踪参数无效或未配置目标，插件禁用\n";
    return;
  }
  worldEntity_ = ecm.EntityByComponents(gz::sim::components::World());
  if (worldEntity_ == gz::sim::kNullEntity ||
      !node_.Subscribe(commandTopic_, &StrapdownController::OnCommand, this))
  {
    gzerr << "[jielian_a2416a] 世界实体或命令订阅不可用，插件禁用\n";
    return;
  }
  telemetryPublisher_ = node_.Advertise<gz::msgs::StringMsg>(telemetryTopic_);
  resultPublisher_ = node_.Advertise<gz::msgs::StringMsg>(resultTopic_);
  configured_ = telemetryPublisher_.Valid() && resultPublisher_.Valid();
  if (configured_)
    gzmsg << "[jielian_a2416a] 捷联真值跟踪已启用，测量频率 " << measurementHz_ << " Hz\n";
}

std::optional<Bounds> StrapdownController::ModelBounds(
  gz::sim::Entity model, const gz::sim::EntityComponentManager &ecm) const
{
  Bounds bounds{{std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity()},
                {-std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity()}};
  bool valid = false;
  for (const auto entity : ecm.Descendants(model))
  {
    // 优先使用 link collision 的完整 AABB；它可加载真实飞机的 mesh。
    if (ecm.Component<gz::sim::components::Link>(entity))
    {
      const gz::sim::Link link(entity);
      const auto local = link.ComputeAxisAlignedBox(ecm);
      if (local)
        Merge(bounds, gz::sim::transformAxisAlignedBox(
          *local, gz::sim::worldPose(entity, ecm)), valid);
    }
    if (!ecm.Component<gz::sim::components::Visual>(entity))
      continue;
    const auto *geometry = ecm.Component<gz::sim::components::Geometry>(entity);
    if (!geometry)
      continue;
    const auto pose = gz::sim::worldPose(entity, ecm);
    // visual mesh 无 collision 时仍按模型 URI 加载真实顶点 AABB。
    const auto localBox = geometry->Data().AxisAlignedBox(
      [](const sdf::Mesh &mesh) { return gz::sim::meshAxisAlignedBox(mesh); });
    if (localBox)
    {
      Merge(bounds, gz::sim::transformAxisAlignedBox(*localBox, pose), valid);
      continue;
    }
    const auto half = GeometryHalfExtents(geometry->Data());
    if (!half)
      continue;
    for (const double x : {-half->X(), half->X()})
      for (const double y : {-half->Y(), half->Y()})
        for (const double z : {-half->Z(), half->Z()})
        {
          const auto point = pose.Pos() + pose.Rot().RotateVector({x, y, z});
          bounds.min.X(std::min(bounds.min.X(), point.X()));
          bounds.min.Y(std::min(bounds.min.Y(), point.Y()));
          bounds.min.Z(std::min(bounds.min.Z(), point.Z()));
          bounds.max.X(std::max(bounds.max.X(), point.X()));
          bounds.max.Y(std::max(bounds.max.Y(), point.Y()));
          bounds.max.Z(std::max(bounds.max.Z(), point.Z()));
          valid = true;
        }
  }
  return valid ? std::optional<Bounds>(bounds) : std::nullopt;
}

void StrapdownController::PostUpdate(
  const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm)
{
  if (!configured_ || info.paused)
    return;
  const double now = std::chrono::duration<double>(info.simTime).count();
  std::lock_guard<std::mutex> lock(mutex_);
  lastSimTime_ = now;
  if (lastMeasurement_ >= 0 && now >= lastMeasurement_ &&
      now - lastMeasurement_ + 1e-12 < 1.0 / measurementHz_)
    return;
  if (lastMeasurement_ > now)
    lastMeasurement_ = -1;
  lastMeasurement_ = now;

  const auto cameraPose = gz::sim::worldPose(sensorEntity_, ecm);
  std::vector<Detection> detections;
  const gz::sim::World world(worldEntity_);
  const auto spherical = world.SphericalCoordinates(ecm);
  for (const auto &name : targetNames_)
  {
    const auto entity = world.ModelByName(ecm, name);
    if (entity == gz::sim::kNullEntity || entity == modelEntity_)
      continue;
    const auto bounds = ModelBounds(entity, ecm);
    if (!bounds)
    {
      if (warnedUnsupported_.insert(name).second)
        gzwarn << "[jielian_a2416a] 目标 [" << name
               << "] 没有可投影的 primitive visual，已忽略\n";
      continue;
    }
    const auto worldCorners = Corners(*bounds);
    std::array<Vec3, 8> cameraCorners;
    for (std::size_t i = 0; i < worldCorners.size(); ++i)
    {
      const auto relative = cameraPose.Rot().RotateVectorReverse(
        worldCorners[i] - cameraPose.Pos());
      cameraCorners[i] = {relative.X(), relative.Y(), relative.Z()};
    }
    const auto centerWorld = (bounds->min + bounds->max) / 2.0;
    const auto center = cameraPose.Rot().RotateVectorReverse(centerWorld - cameraPose.Pos());
    auto projected = ProjectBounds(static_cast<std::uint64_t>(entity), name,
      cameraCorners, {center.X(), center.Y(), center.Z()}, optics_);
    if (projected)
    {
      if (spherical)
      {
        const auto geographic = spherical->PositionTransform(centerWorld,
          gz::math::SphericalCoordinates::LOCAL2,
          gz::math::SphericalCoordinates::SPHERICAL);
        projected->geographicValid = geographic.IsFinite();
        if (projected->geographicValid)
        {
          projected->latitudeDeg = geographic.X() * kRadToDeg;
          projected->longitudeDeg = geographic.Y() * kRadToDeg;
          projected->altitudeM = geographic.Z();
        }
      }
      detections.push_back(*projected);
    }
  }
  tracker_.Update(now, std::move(detections));
  Publish(now, gz::sim::worldPose(modelEntity_, ecm));
}

void StrapdownController::OnCommand(const gz::msgs::StringMsg &message)
{
  const auto command = StringField(message.data(), "cmd");
  if (!command)
  {
    Result("", false, "缺少 cmd 字段");
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const double p1 = NumberField(message.data(), "p1");
  const double p2 = NumberField(message.data(), "p2");
  const double p3 = NumberField(message.data(), "p3");
  const double p4 = NumberField(message.data(), "p4");
  bool ok = true;
  std::string detail = "accepted";
  if (*command == "heartbeat" || *command == "noop") {}
  else if (*command == "recognition_on" || *command == "recognition_start")
    tracker_.SetRecognition(true);
  else if (*command == "recognition_off" || *command == "recognition_stop")
    tracker_.SetRecognition(false);
  else if (*command == "auto_lock_on") tracker_.SetAutoLock(true, lastSimTime_);
  else if (*command == "auto_lock_off") tracker_.SetAutoLock(false, lastSimTime_);
  else if (*command == "point_lock") ok = tracker_.PointLock(p1, p2, lastSimTime_);
  else if (*command == "box_lock") ok = tracker_.BoxLock({p1, p2, p3, p4}, lastSimTime_);
  else if (*command == "crosshair_lock") ok = tracker_.CrosshairLock(lastSimTime_);
  else if (*command == "cancel_track") tracker_.Cancel();
  else if (*command == "crosshair_move") tracker_.MoveCrosshair(p1, p2, lastSimTime_);
  else if (*command == "aim_trim") ok = tracker_.TrimAim(p1, p2);
  else if (*command == "gate_grow") tracker_.ScaleGate(1.1);
  else if (*command == "gate_shrink") tracker_.ScaleGate(1.0 / 1.1);
  else if (*command == "stream_set" || *command == "zoom_in" ||
           *command == "zoom_out" || *command == "image_enhance_on" ||
           *command == "image_enhance_off" || *command == "ip_set" ||
           *command == "video_on" || *command == "video_off" ||
           *command == "osd_full" || *command == "osd_half" ||
           *command == "osd_off")
    detail = "handled_by_gateway";
  else
  {
    ok = false;
    detail = "unsupported_command";
  }
  if (!ok && detail == "accepted")
    detail = "no_matching_target_or_invalid_state";
  Result(*command, ok, detail);
}

void StrapdownController::Result(
  const std::string &command, bool ok, const std::string &detail)
{
  gz::msgs::StringMsg result;
  std::ostringstream json;
  json << "{\"cmd\":\"" << Escape(command) << "\",\"ok\":"
       << (ok ? "true" : "false") << ",\"detail\":\""
       << Escape(detail) << "\"}";
  result.set_data(json.str());
  resultPublisher_.Publish(result);
}

void StrapdownController::Publish(double now, const gz::math::Pose3d &modelPose)
{
  const auto misalignment = tracker_.MisalignmentDeg();
  const auto crosshair = tracker_.Crosshair();
  const auto gate = tracker_.Gate();
  const auto rpy = modelPose.Rot().Euler() * kRadToDeg;
  std::ostringstream json;
  json << std::setprecision(std::numeric_limits<double>::max_digits10)
       << "{\"type\":\"telemetry\",\"source\":\"gazebo_plugin\""
       << ",\"measurement_seq\":" << tracker_.MeasurementSeq()
       << ",\"sim_uptime_s\":" << now
       << ",\"work_state\":" << WorkState(tracker_.State())
       << ",\"tracking_state\":\"" << ToString(tracker_.State()) << "\""
       << ",\"recognition_enabled\":" << (tracker_.RecognitionEnabled() ? "true" : "false")
       << ",\"auto_lock_enabled\":" << (tracker_.AutoLockEnabled() ? "true" : "false")
       << ",\"locked_target_name\":";
  if (tracker_.Locked()) json << "\"" << Escape(tracker_.Locked()->name) << "\"";
  else json << "null";
  json << ",\"crosshair_px\":[" << crosshair[0] << "," << crosshair[1] << "]"
       << ",\"gate_px\":";
  if (gate) json << "[" << gate->x << "," << gate->y << "," << gate->w << "," << gate->h << "]";
  else json << "null";
  json << ",\"misalignment_deg\":{\"pitch\":" << misalignment[0]
       << ",\"yaw\":" << misalignment[1] << "}"
       << ",\"platform_frame_deg\":[0,0]"
       << ",\"platform_gyro_rate_dps\":[0,0]"
       << ",\"los_rate_dps\":[0,0]"
       << ",\"seeker_attitude_deg\":[0,0,0]"
       << ",\"aircraft_attitude_deg\":{\"roll\":" << rpy.X()
       << ",\"pitch\":" << rpy.Y() << ",\"yaw\":" << rpy.Z() << "}"
       << ",\"targets\":[";
  bool first = true;
  for (const auto &target : tracker_.Slots())
  {
    if (!first) json << ',';
    first = false;
    json << "{\"name\":\"" << Escape(target.name) << "\",\"entity_id\":"
         << target.entity << ",\"box_px\":[" << target.box.x << ',' << target.box.y
         << ',' << target.box.w << ',' << target.box.h << "],\"center_px\":["
         << target.centerX << ',' << target.centerY << "],\"range_m\":" << target.rangeM
         << ",\"confidence\":" << target.confidence << ",\"type\":" << target.type
         << ",\"geographic_valid\":" << (target.geographicValid ? "true" : "false")
         << ",\"lat_deg\":" << target.latitudeDeg
         << ",\"lon_deg\":" << target.longitudeDeg
         << ",\"alt_m\":" << target.altitudeM << '}';
  }
  json << "]}";
  gz::msgs::StringMsg message;
  message.set_data(json.str());
  telemetryPublisher_.Publish(message);
}
}

GZ_ADD_PLUGIN(jielian_a2416a::StrapdownController,
              gz::sim::System,
              jielian_a2416a::StrapdownController::ISystemConfigure,
              jielian_a2416a::StrapdownController::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(jielian_a2416a::StrapdownController,
                    "jielian_a2416a::StrapdownController")
