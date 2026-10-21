#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "two_axis/control.hh"
#include "two_axis/kinematics.hh"
#include "gimbal_controller/mini_json.hh"

namespace two_axis
{
namespace json = gimbal::json;

namespace
{
constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kDeg2Rad = M_PI / 180.0;

/// 关节 -> 轴索引：0 yaw, 1 pitch
struct AxisJoints
{
  std::array<std::string, 2> names{"yaw_joint", "pitch_joint"};
};

struct Frame
{
  bool valid{false};
  std::string rgb;
  int width{0};
  int height{0};
  double simStamp{0.0};
  double arrive{0.0};
  std::int64_t seq{0};
};

struct ProcessedFrame
{
  std::int64_t seq{0};
  cv::Mat bgr;
  double arrive{0.0};
};
}  // namespace

class GimbalController : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  void OnImage(const gz::msgs::Image &_msg);
  void OnCommand(const gz::msgs::StringMsg &_msg);
  void PublishResult(const std::string &_cmd, bool _ok,
                     const std::string &_message,
                     const std::string &_requestId = "");
  void PublishTelemetry(double _now, const JointSample &_js,
                        const std::array<double, 2> &_accel, int _width,
                        int _height, const Frame &_frame);

  // SDF 配置
  ControlConfig cfg_;
  AxisJoints axes_;
  std::string mountLinkName_{"mount_link"};
  std::string linkName_{"camera_link"};
  std::string sensorName_{"camera"};
  std::string commandTopic_{"/two_axis_pod/command"};
  std::string telemetryTopic_{"/two_axis_pod/telemetry"};
  std::string resultTopic_{"/two_axis_pod/command_result"};
  std::string mountMode_{"upright"};
  bool mountDynamic_{false};
  bool hasMountSample_{false};
  Mat3 mountWorldRotation_;
  double controlHz_{100.0};
  double telemetryHz_{50.0};

  gz::sim::Entity modelEntity_{gz::sim::kNullEntity};
  gz::sim::Entity mountEntity_{gz::sim::kNullEntity};
  std::array<gz::sim::Entity, 2> jointEntities_
  {
    gz::sim::kNullEntity, gz::sim::kNullEntity};

  gz::transport::Node node_;
  gz::transport::Node::Publisher telemetryPub_;
  gz::transport::Node::Publisher resultPub_;

  // 图像（transport 线程写入）
  std::mutex imgMutex_;
  Frame frame_;
  std::int64_t imgCounter_{0};

  // 控制（sim 线程 + 指令线程）
  std::mutex ctrlMutex_;
  GimbalControl control_;
  cv::Mat latestBgr_;
  int latestWidth_{0};
  int latestHeight_{0};
  std::deque<ProcessedFrame> processedFrames_;
  std::int64_t lastProcessedSeq_{-1};
  std::int64_t meanSeq_{0};
  std::chrono::steady_clock::time_point manualReceived_{};

  // 计时
  double startTime_{-1.0};
  double controlAccum_{0.0};
  double telemetryAccum_{0.0};
  bool configured_{false};
  bool jointsInitialized_{false};
  std::array<double, 2> lastCmd_{{0.0, 0.0}};

  // 关节遥测（角加速度平滑）
  JointSample latestJointSample_;
  bool hasJointSample_{false};
  JointSample prevJs_;
  bool hasPrevJs_{false};
  std::array<double, 2> accel_{{0.0, 0.0}};
};

void GimbalController::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &)
{
  this->modelEntity_ = _entity;

  // ---------------- SDF 参数 ----------------
  auto getStr = [&](const char *_k, const std::string & _def)
  {
    return _sdf->HasElement(_k) ? _sdf->Get<std::string>(_k) : _def;
  };
  auto getDouble = [&](const char *_k, double _def)
  {
    return _sdf->HasElement(_k) ? _sdf->Get<double>(_k) : _def;
  };

  axes_.names[0] = getStr("yaw_joint", axes_.names[0]);
  axes_.names[1] = getStr("pitch_joint", axes_.names[1]);
  mountLinkName_ = getStr("mount_link", mountLinkName_);
  linkName_ = getStr("camera_link", linkName_);
  sensorName_ = getStr("camera_sensor", sensorName_);
  commandTopic_ = getStr("command_topic", commandTopic_);
  telemetryTopic_ = getStr("telemetry_topic", telemetryTopic_);
  resultTopic_ = getStr("command_result_topic", resultTopic_);
  controlHz_ = std::max(1.0, getDouble("control_rate", controlHz_));
  telemetryHz_ = std::max(1.0, getDouble("telemetry_rate", telemetryHz_));

  mountMode_ = getStr("mount_mode", mountMode_);
  mountDynamic_ = _sdf->HasElement("mount_dynamic") &&
    _sdf->Get<bool>("mount_dynamic");
  if (mountMode_ != "upright")
  {
    gzerr << "[two_axis] mount_mode 必须为 upright\n";
    return;
  }
  // 安装外参包括模型姿态；动态载机在每步仿真中更新。
  gz::sim::Model model(this->modelEntity_);
  const auto mountEntity = model.LinkByName(_ecm, mountLinkName_);
  mountEntity_ = mountEntity;
  if (mountEntity == gz::sim::kNullEntity)
  {
    gzerr << "[gimbal] 未找到安装 link [" << mountLinkName_ << "]，插件禁用\n";
    return;
  }
  const auto staticRotation = gz::sim::worldPose(mountEntity, _ecm).Rot();
  mountWorldRotation_ = MatMul(Rz(staticRotation.Yaw()),
                               MatMul(Ry(staticRotation.Pitch()), Rx(staticRotation.Roll())));
  cfg_.mount_world_rotation = mountWorldRotation_;
  const auto cameraLinkEntity = model.LinkByName(_ecm, linkName_);
  gz::sim::Link cameraLink(cameraLinkEntity);
  const auto sensorEntity = cameraLink.SensorByName(_ecm, sensorName_);
  if (cameraLinkEntity == gz::sim::kNullEntity || sensorEntity == gz::sim::kNullEntity ||
      !_ecm.Component<gz::sim::components::Camera>(sensorEntity))
  {
    gzerr << "[gimbal] 未找到实际相机sensor，插件禁用\n";
    return;
  }
  const auto optical = (gz::sim::worldPose(cameraLinkEntity, _ecm).Rot().Inverse() *
                        gz::sim::worldPose(sensorEntity, _ecm).Rot());
  cfg_.sensor_rotation = MatMul(Rz(optical.Yaw()), MatMul(Ry(optical.Pitch()), Rx(optical.Roll())));
  if (!ValidSensorRotation(cfg_.sensor_rotation))
  {
    gzerr << "[two_axis] camera sensor 光学旋转必须为 RPY=(0,-pi/2,0)，插件禁用\n";
    return;
  }

  cfg_.max_rate_dps = getDouble("max_rate_dps", cfg_.max_rate_dps);
  cfg_.max_acc_dps2 = getDouble("max_acc_dps2", cfg_.max_acc_dps2);
  cfg_.kp = getDouble("servo_kp", cfg_.kp);
  cfg_.yaw_min_deg = getDouble("yaw_min_deg", cfg_.yaw_min_deg);
  cfg_.yaw_max_deg = getDouble("yaw_max_deg", cfg_.yaw_max_deg);
  cfg_.pitch_min_deg = getDouble("pitch_min_deg", cfg_.pitch_min_deg);
  cfg_.pitch_max_deg = getDouble("pitch_max_deg", cfg_.pitch_max_deg);
  cfg_.occlusion_timeout_s =
    getDouble("occlusion_timeout_s", cfg_.occlusion_timeout_s);
  if (!std::isfinite(cfg_.max_rate_dps) || !std::isfinite(cfg_.max_acc_dps2) ||
      !std::isfinite(cfg_.kp) || cfg_.max_rate_dps <= 0 || cfg_.max_acc_dps2 <= 0 ||
      cfg_.kp <= 0 || !std::isfinite(cfg_.yaw_min_deg) || !std::isfinite(cfg_.yaw_max_deg) ||
      !std::isfinite(cfg_.pitch_min_deg) || !std::isfinite(cfg_.pitch_max_deg) ||
      cfg_.yaw_min_deg >= cfg_.yaw_max_deg || cfg_.pitch_min_deg >= cfg_.pitch_max_deg ||
      cfg_.yaw_min_deg > 0 || cfg_.yaw_max_deg < 0 ||
      cfg_.pitch_min_deg > 0 || cfg_.pitch_max_deg < 0)
  {
    gzerr << "[two_axis] 无效控制参数或限位不包含机械零位，插件禁用\n";
    return;
  }
  control_.Configure(cfg_);

  // ---------------- 实体与关节 ----------------
  for (std::size_t i = 0; i < 2; ++i)
  {
    const auto j = model.JointByName(_ecm, axes_.names[i]);
    if (j == gz::sim::kNullEntity)
    {
      gzerr << "[gimbal] 未找到关节 [" << axes_.names[i] << "]，插件禁用\n";
      return;
    }
    jointEntities_[i] = j;
    if (!_ecm.Component<gz::sim::components::JointPosition>(j))
      _ecm.CreateComponent(j, gz::sim::components::JointPosition({0.0}));
    if (!_ecm.Component<gz::sim::components::JointVelocity>(j))
      _ecm.CreateComponent(j, gz::sim::components::JointVelocity({0.0}));
    if (!_ecm.Component<gz::sim::components::JointVelocityCmd>(j))
      _ecm.CreateComponent(j, gz::sim::components::JointVelocityCmd({0.0}));
  }

  // ---------------- 话题名 ----------------
  std::string worldName;
  const auto world = gz::sim::worldEntity(_ecm);
  if (world != gz::sim::kNullEntity)
  {
    if (auto *n = _ecm.Component<gz::sim::components::Name>(world))
      worldName = n->Data();
  }
  std::string modelName;
  if (auto *n = _ecm.Component<gz::sim::components::Name>(this->modelEntity_))
    modelName = n->Data();

  const std::string imageTopic = getStr("image_topic",
    "/world/" + worldName + "/model/" + modelName + "/link/" + linkName_ +
    "/sensor/" + sensorName_ + "/image");

  this->node_.Subscribe(imageTopic, &GimbalController::OnImage, this);
  this->node_.Subscribe(commandTopic_, &GimbalController::OnCommand, this);
  this->telemetryPub_ =
    this->node_.Advertise<gz::msgs::StringMsg>(telemetryTopic_);
  this->resultPub_ =
    this->node_.Advertise<gz::msgs::StringMsg>(resultTopic_);

  configured_ = true;
  gzmsg << "[gimbal] 插件已配置: 模型=" << modelName << " 图像=" << imageTopic
        << " 指令=" << commandTopic_ << " 遥测=" << telemetryTopic_
        << " 控制=" << controlHz_ << "Hz\n";
}

void GimbalController::OnImage(const gz::msgs::Image &_msg)
{
  std::int64_t seq = -1;
  for (int i = 0; i < _msg.header().data_size(); ++i)
  {
    const auto &entry = _msg.header().data(i);
    if (entry.key() == "seq" && entry.value_size() > 0)
    {
      try
      {
        seq = std::stoll(entry.value(0));
      }
      catch (...)
      {
        seq = -1;
      }
    }
  }
  if (seq < 0)
    seq = this->imgCounter_ + 1;

  std::lock_guard<std::mutex> lock(this->imgMutex_);
  ++this->imgCounter_;
  this->frame_.valid = true;
  this->frame_.rgb = _msg.data();
  this->frame_.width = static_cast<int>(_msg.width());
  this->frame_.height = static_cast<int>(_msg.height());
  this->frame_.simStamp =
    _msg.header().stamp().sec() + _msg.header().stamp().nsec() * 1e-9;
  this->frame_.arrive = std::chrono::duration<double>(
                          std::chrono::steady_clock::now().time_since_epoch())
                        .count();
  this->frame_.seq = seq;
}

void GimbalController::PublishResult(const std::string &_cmd, bool _ok,
                                     const std::string &_message,
                                     const std::string &_requestId)
{
  std::ostringstream os;
  os << "{\"type\":\"command_result\",\"cmd\":\"" << json::Escape(_cmd)
     << "\",\"ok\":" << (_ok ? "true" : "false") << ",\"message\":";
  if (_message.empty())
    os << "null";
  else
    os << "\"" << json::Escape(_message) << "\"";
  if (!_requestId.empty())
    os << ",\"request_id\":\"" << json::Escape(_requestId) << "\"";
  os << "}";
  gz::msgs::StringMsg msg;
  msg.set_data(os.str());
  this->resultPub_.Publish(msg);
}

void GimbalController::OnCommand(const gz::msgs::StringMsg &_msg)
{
  json::Value v;
  if (!json::Parse(_msg.data(), v) || v.type != json::Value::Type::OBJ)
  {
    this->PublishResult("", false, "无效 JSON");
    return;
  }
  const json::Value *cmdV = v.Find("cmd");
  const std::string cmd = cmdV ? cmdV->AsStr() : "";
  const json::Value *idV = v.Find("request_id");
  const std::string requestId = idV ? idV->AsStr() : "";
  auto reply = [&](bool ok, const std::string & message)
  {
    this->PublishResult(cmd, ok, message, requestId);
  };
  {
    std::lock_guard<std::mutex> lock(this->ctrlMutex_);
    if (cmd != "stop" && cmd != "reset_fault" && cmd != "ping" &&
        !this->control_.fault_reason().empty())
    {
      reply(false, "请先复位故障");
      return;
    }
  }
  if (cmd == "track")
  {
    const json::Value *boxV = v.Find("box");
    if (!boxV || boxV->type != json::Value::Type::ARR || boxV->arr.size() != 4)
    {
      reply(false, "缺少选框参数");
      return;
    }
    std::array<double, 4> box{};
    for (int i = 0; i < 4; ++i)
    {
      if (boxV->arr[i].type != json::Value::Type::NUM ||
          !std::isfinite(boxV->arr[i].num))
      {
        reply(false, "选框必须为有限数值");
        return;
      }
      box[i] = boxV->arr[i].AsNum();
    }
    if (box[0] < 0 || box[1] < 0 || box[2] <= 0 || box[3] <= 0 ||
        box[0] + box[2] > 1 || box[1] + box[3] > 1)
    {
      reply(false, "选框必须位于归一化图像范围内");
      return;
    }
    std::optional<std::int64_t> requestedFrameSeq;
    if (const json::Value *seqV = v.Find("frame_seq"))
    {
      if (seqV->type != json::Value::Type::NUM || !std::isfinite(seqV->num) ||
          std::floor(seqV->num) != seqV->num || seqV->num < 0 ||
          seqV->num > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
      {
        reply(false, "frame_seq 必须是非负整数");
        return;
      }
      requestedFrameSeq = static_cast<std::int64_t>(seqV->num);
    }
    double frameAge;
    {
      std::lock_guard<std::mutex> imgLock(this->imgMutex_);
      frameAge = std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch()).count() -
                 this->frame_.arrive;
    }
    if (frameAge >= 0.5)
    {
      reply(false, "视频帧已超时，请等待新视频帧");
      return;
    }
    std::lock_guard<std::mutex> lock(this->ctrlMutex_);
    const cv::Mat *trackingImage = &this->latestBgr_;
    int trackingWidth = this->latestWidth_;
    int trackingHeight = this->latestHeight_;
    if (requestedFrameSeq)
    {
      const auto it = std::find_if(this->processedFrames_.rbegin(),
        this->processedFrames_.rend(), [&](const ProcessedFrame &_frame)
        {
          return _frame.seq == *requestedFrameSeq;
        });
      if (it == this->processedFrames_.rend())
      {
        reply(false, "指定视频帧已过期或尚未处理");
        return;
      }
      const double selectedAge = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count() - it->arrive;
      if (selectedAge >= 0.5)
      {
        reply(false, "指定视频帧已超时");
        return;
      }
      trackingImage = &it->bgr;
      trackingWidth = it->bgr.cols;
      trackingHeight = it->bgr.rows;
    }
    if (trackingImage->empty())
    {
      reply(false, "尚无视频帧");
      return;
    }
    auto [ok, message] =
      this->control_.StartTracking(*trackingImage, box, trackingWidth,
                                   trackingHeight);
    reply(ok, message);
  }
  else if (cmd == "stop")
  {
    std::lock_guard<std::mutex> lock(this->ctrlMutex_);
    this->control_.Stop();
    reply(true, "");
  }
  else if (cmd == "home")
  {
    std::lock_guard<std::mutex> lock(this->ctrlMutex_);
    this->control_.GoHome();
    reply(true, "");
  }
  else if (cmd == "reset")
  {
    std::lock_guard<std::mutex> lock(this->ctrlMutex_);
    this->control_.GoHome();
    reply(true, "");
  }
  else if (cmd == "reset_axis")
  {
    const json::Value *axisV = v.Find("axis");
    const std::string axis = axisV ? axisV->AsStr() : "";
    if (axis != "yaw" && axis != "pitch")
    {
      reply(false, "axis 必须为 yaw 或 pitch");
      return;
    }
    std::lock_guard<std::mutex> lock(this->ctrlMutex_);
    if (!this->hasJointSample_)
    {
      reply(false, "尚无关节采样");
      return;
    }
    if (!this->control_.ResetAxis(axis, this->latestJointSample_))
    {
      reply(false, "轴零位不可达");
      return;
    }
    reply(true, "");
  }
  else if (cmd == "manual")
  {
    if (v.Find("roll"))
    {
      reply(false, "两轴吊舱不支持 roll");
      return;
    }
    for (const char *key :
         {"yaw", "pitch"
         })
    {
      const json::Value *x = v.Find(key);
      if (x && (x->type != json::Value::Type::NUM || !std::isfinite(x->num)))
      {
        reply(false, "手动角速度必须为有限数值");
        return;
      }
    }
    auto num = [&](const char *_k)
    {
      const json::Value *x = v.Find(_k);
      return x ? x->AsNum() : 0.0;
    };
    std::lock_guard<std::mutex> lock(this->ctrlMutex_);
    if (!this->hasJointSample_)
    {
      reply(false, "尚无关节采样");
      return;
    }
    const double watchdogNow = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    this->control_.Manual(num("yaw"), num("pitch"), watchdogNow);
    reply(true, "");
    this->manualReceived_ = std::chrono::steady_clock::now();
  }
  else if (cmd == "reset_fault")
  {
    std::lock_guard<std::mutex> lock(this->ctrlMutex_);
    this->control_.ResetFault();
    reply(true, "");
  }
  else if (cmd == "ping")
  {
    reply(true, "");
  }
  else
  {
    reply(false, "未知指令 " + cmd);
  }
}

void GimbalController::PreUpdate(const gz::sim::UpdateInfo &_info,
                                 gz::sim::EntityComponentManager &_ecm)
{
  if (!configured_ || _info.paused || this->modelEntity_ == gz::sim::kNullEntity)
    return;

  const double now =
    std::chrono::duration<double>(_info.simTime).count();
  const double dt = std::chrono::duration<double>(_info.dt).count();
  if (this->startTime_ < 0.0)
    this->startTime_ = now;

  // ---- 读取关节 ----
  JointSample js;
  double *pos[2] = {&js.yaw, &js.pitch};
  double *vel[2] = {&js.yaw_vel, &js.pitch_vel};
  for (int i = 0; i < 2; ++i)
  {
    if (auto *p =
          _ecm.Component<gz::sim::components::JointPosition>(jointEntities_[i]))
      if (!p->Data().empty())
        * pos[i] = p->Data()[0];
    if (auto *v =
          _ecm.Component<gz::sim::components::JointVelocity>(jointEntities_[i]))
      if (!v->Data().empty())
        * vel[i] = v->Data()[0];
  }

  {
    std::lock_guard<std::mutex> lock(this->ctrlMutex_);
    if (this->mountDynamic_)
    {
      const auto rotation = gz::sim::worldPose(this->mountEntity_, _ecm).Rot();
      const auto current = MatMul(Rz(rotation.Yaw()),
        MatMul(Ry(rotation.Pitch()), Rx(rotation.Roll())));
      this->cfg_.mount_angular_velocity = this->hasMountSample_ ?
        MountAngularVelocity(this->mountWorldRotation_, current, dt) : Vec3{};
      this->hasMountSample_ = true;
      this->mountWorldRotation_ = current;
      this->cfg_.mount_world_rotation = this->mountWorldRotation_;
      this->control_.Configure(this->cfg_);
    }
    this->latestJointSample_ = js;
    this->hasJointSample_ = true;
  }

  // 角加速度（平滑）
  if (this->hasPrevJs_ && dt > 1e-6)
  {
    const double a[2] = {(js.yaw_vel - this->prevJs_.yaw_vel) / dt,
                         (js.pitch_vel - this->prevJs_.pitch_vel) / dt
                        };
    for (int i = 0; i < 2; ++i)
      this->accel_[i] = 0.85 * this->accel_[i] + 0.15 * a[i];
  }
  this->prevJs_ = js;
  this->hasPrevJs_ = true;

  // ---- 控制循环 ----
  this->controlAccum_ += dt;
  const double controlPeriod = 1.0 / this->controlHz_;
  std::array<double, 2> cmd{0.0, 0.0};
  bool ranControl = false;
  if (this->controlAccum_ >= controlPeriod || !this->jointsInitialized_)
  {
    this->controlAccum_ = 0.0;
    this->jointsInitialized_ = true;
    ranControl = true;

    // 取最新图像 -> BGR（仅在新帧时转换）
    cv::Mat bgr;
    std::int64_t processedSeq{-1};
    double processedArrive{0.0};
    {
      std::lock_guard<std::mutex> imgLock(this->imgMutex_);
      if (this->frame_.valid && this->frame_.seq != this->lastProcessedSeq_ &&
          this->frame_.width > 0 && this->frame_.height > 0)
      {
        const int w = this->frame_.width;
        const int h = this->frame_.height;
        const std::size_t need =
          static_cast<std::size_t>(w) * h * 3u;
        if (this->frame_.rgb.size() >= need)
        {
          const cv::Mat rgb(h, w, CV_8UC3,
                            const_cast<char *>(this->frame_.rgb.data()));
          cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
          processedSeq = this->frame_.seq;
          processedArrive = this->frame_.arrive;
        }
        this->lastProcessedSeq_ = this->frame_.seq;
        this->meanSeq_ = this->frame_.seq;
      }
    }

    std::lock_guard<std::mutex> lock(this->ctrlMutex_);
    if (!bgr.empty())
    {
      this->latestBgr_ = bgr.clone();
      this->latestWidth_ = bgr.cols;
      this->latestHeight_ = bgr.rows;
      this->processedFrames_.push_back(
        ProcessedFrame{processedSeq, this->latestBgr_, processedArrive});
      while (this->processedFrames_.size() > 4)
        this->processedFrames_.pop_front();
    }
    const bool newFrame = !bgr.empty();
    const cv::Mat &useImg = newFrame ? bgr : this->latestBgr_;
    const int useW = newFrame ? bgr.cols : this->latestWidth_;
    const int useH = newFrame ? bgr.rows : this->latestHeight_;
    const auto steadyNow = std::chrono::steady_clock::now();
    const double watchdogNow = std::chrono::duration<double>(
                                 steadyNow.time_since_epoch()).count();
    if (this->control_.mode() == Mode::MANUAL &&
        std::chrono::duration<double>(steadyNow - this->manualReceived_).count()
        >= 0.5)
      this->control_.WatchdogStop();
    cmd = this->control_.Step(now, newFrame && !useImg.empty(), useImg, useW,
                              useH, js, watchdogNow, processedSeq);
  }

  // ---- 写入关节速度指令（每步写入，物理系统在 Update 中应用后清零）----
  if (ranControl)
  {
    for (int i = 0; i < 2; ++i)
      this->lastCmd_[i] = cmd[i];
  }
  for (int i = 0; i < 2; ++i)
  {
    const double v = this->lastCmd_[i] * kDeg2Rad;
    auto comp =
      _ecm.Component<gz::sim::components::JointVelocityCmd>(jointEntities_[i]);
    if (!comp)
    {
      _ecm.CreateComponent(jointEntities_[i],
                           gz::sim::components::JointVelocityCmd({v}));
    }
    else
    {
      auto &d = comp->Data();
      if (d.empty())
        d.push_back(v);
      else
        d[0] = v;
    }
  }

  // ---- 遥测 ----
  this->telemetryAccum_ += dt;
  if (this->telemetryAccum_ >= 1.0 / this->telemetryHz_)
  {
    this->telemetryAccum_ = 0.0;
    Frame f;
    {
      std::lock_guard<std::mutex> imgLock(this->imgMutex_);
      f = this->frame_;
    }
    this->PublishTelemetry(now, js, this->accel_, f.width, f.height, f);
  }
}

void GimbalController::PublishTelemetry(double _now, const JointSample &_js,
                                        const std::array<double, 2> &_accel,
                                        int _width, int _height,
                                        const Frame &_frame)
{
  using json::Num;
  std::lock_guard<std::mutex> lock(this->ctrlMutex_);
  GimbalControl &c = this->control_;

  const Mat3 rCam = OpticalRotation(_js.yaw, _js.pitch, cfg_.sensor_rotation);
  const auto qBody = QuatFromR(rCam), qWorld = QuatFromR(MatMul(mountWorldRotation_, rCam)), qMount = QuatFromR(mountWorldRotation_);
  const auto rpyBody = RToRpyDeg(rCam), rpyWorld = RToRpyDeg(MatMul(mountWorldRotation_, rCam));
  const auto los = MatVec(rCam, {1, 0, 0});
  const bool singular = Singular(_js.pitch);
  const auto &lf = c.limit_flags();
  const auto &cmd = c.cmd();

  std::ostringstream os;
  os << "{\"type\":\"telemetry\",\"source\":\"gazebo_plugin\""
     << ",\"mount\":{\"mode\":\"" << mountMode_ << "\",\"static\":"
     << (mountDynamic_ ? "false" : "true") << ",\"quat_world\":["
     << Num(qMount[0], 6) << "," << Num(qMount[1], 6) << ","
     << Num(qMount[2], 6) << "," << Num(qMount[3], 6) << "]}"
     << ",\"ts_mono\":" << Num(_now, 6)
     << ",\"sim_uptime\":" << Num(_now - this->startTime_, 1)
     << ",\"mode\":\"" << ToString(c.mode()) << "\""
     << ",\"tracking\":{\"state\":\"" << ToString(c.track_state())
     << "\",\"fault\":";
  if (c.fault_reason().empty())
    os << "null";
  else
    os << "\"" << json::Escape(c.fault_reason()) << "\"";
  os << ",\"message\":";
  if (c.client_message().empty())
    os << "null";
  else
    os << "\"" << json::Escape(c.client_message()) << "\"";
  os << ",\"valid_frames\":" << c.valid_frames()
     << ",\"invalid_frames\":" << c.invalid_frames();
  const char *constraint = c.mode() != Mode::TRACK ? "NONE" :
    singular ? "SINGULAR" : (lf[0] || lf[1]) ? "LIMITED" :
    c.saturated() ? "RATE_LIMITED" : "NONE";
  os << ",\"constraint\":\"" << constraint << "\"}";

  os << ",\"stability\":{\"state\":\"" << ToString(c.stability())
     << "\",\"center_err_deg\":" << Num(c.center_err_deg(), 3) << "}";

  os << ",\"target\":{\"valid\":" << (c.target_valid() ? "true" : "false");
  if (c.has_target_box())
  {
    const Box &b = c.target_box();
    os << ",\"box_px\":[" << Num(b.x, 1) << "," << Num(b.y, 1) << ","
       << Num(b.w, 1) << "," << Num(b.h, 1) << "]";
    if (_width > 0 && _height > 0)
      os << ",\"box_norm\":[" << Num(b.x / _width, 5) << ","
         << Num(b.y / _height, 5) << "," << Num(b.w / _width, 5) << ","
         << Num(b.h / _height, 5) << "]";
    else
      os << ",\"box_norm\":null";
  }
  else
  {
    os << ",\"box_px\":null,\"box_norm\":null";
  }
  const auto &miss = c.target_miss();
  os << ",\"miss\":{\"valid\":" << (miss.valid ? "true" : "false");
  if (miss.valid)
  {
    os << ",\"center_px\":[" << Num(miss.center_px[0], 2) << ","
       << Num(miss.center_px[1], 2) << "]"
       << ",\"offset_px\":[" << Num(miss.offset_px[0], 2) << ","
       << Num(miss.offset_px[1], 2) << "]"
       << ",\"offset_deg\":[" << Num(miss.offset_deg[0], 4) << ","
       << Num(miss.offset_deg[1], 4) << "]"
       << ",\"frame_seq\":";
    if (miss.frame_seq >= 0)
      os << miss.frame_seq;
    else
      os << "null";
  }
  else
    os << ",\"center_px\":null,\"offset_px\":null,\"offset_deg\":null,\"frame_seq\":null";
  os << "}}";

  os << ",\"protocol_version\":6,\"angle_convention\":\"motion_head_joint\",\"singular\":" << (singular ? "true" : "false")
     << ",\"los_mount\":[" << Num(los.x, 6) << "," << Num(los.y, 6) << "," << Num(los.z, 6) << "]";
  const double positions[2] = {_js.yaw, _js.pitch}, velocities[2] = {_js.yaw_vel, _js.pitch_vel};
  const char *names[2] = {"yaw", "pitch"};
  for (const char *field :
       {"attitude_deg", "joints_deg"
       })
  {
    os << ",\"" << field << "\":{";
    for (int i = 0; i < 2; ++i)
    {
      if (i)
        os << ",";
      os << "\"" << names[i] << "\":{\"pos\":" << Num(positions[i]*kRad2Deg, 3)
         << ",\"vel\":" << Num(velocities[i]*kRad2Deg, 3) << ",\"acc\":" << Num(_accel[i]*kRad2Deg, 3) << "}";
    }
    os << "}";
  }
  os << ",\"camera\":{\"quat_body\":[" << Num(qBody[0], 6) << ","
     << Num(qBody[1], 6) << "," << Num(qBody[2], 6) << "," << Num(qBody[3], 6)
     << "],\"rpy_body_deg\":[" << Num(rpyBody[0], 3) << ","
     << Num(rpyBody[1], 3) << "," << Num(rpyBody[2], 3)
     << "],\"quat_world\":[" << Num(qWorld[0], 6) << "," << Num(qWorld[1], 6)
     << "," << Num(qWorld[2], 6) << "," << Num(qWorld[3], 6)
     << "],\"rpy_world_deg\":[" << Num(rpyWorld[0], 3) << ","
     << Num(rpyWorld[1], 3) << "," << Num(rpyWorld[2], 3)
     << "],\"frame_seq\":" << this->meanSeq_ << "}";

  os << ",\"limits\":{\"yaw\":" << (lf[0] ? "true" : "false")
     << ",\"pitch\":" << (lf[1] ? "true" : "false")
     << ",\"saturated\":" << (c.saturated() ? "true" : "false") << "}";

  os << ",\"cmd_dps\":{\"yaw\":" << Num(cmd[0], 3)
     << ",\"pitch\":" << Num(cmd[1], 3)
     << "}";

  os << ",\"frame\":{\"seq\":" << _frame.seq
     << ",\"sim_stamp\":" << Num(_frame.simStamp, 6)
     << ",\"t0_mono\":" << Num(_frame.simStamp, 6)
     << ",\"arrive_mono\":" << Num(_frame.arrive, 6) << "}}";

  gz::msgs::StringMsg msg;
  msg.set_data(os.str());
  this->telemetryPub_.Publish(msg);
}

}  // namespace two_axis

GZ_ADD_PLUGIN(two_axis::GimbalController, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(two_axis::GimbalController, "two_axis::GimbalController")
