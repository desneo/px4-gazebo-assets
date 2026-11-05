#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>
#include "vibration_signal.hh"

namespace rocket {
class FlightVibration final : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    auto config = sdf->Clone();
    model_ = gz::sim::Model(entity);
    link_ = model_.LinkByName(ecm, "base_link");
    if (link_ == gz::sim::kNullEntity) {
      gzerr << "[FlightVibration] base_link missing\n";
      return;
    }
    reference_ = config->Get<double>("rotor_reference_rad_s");
    minimum_ = config->Get<double>("minimum_rotor_rad_s");
    slowdown_ = config->Get<double>("rotor_velocity_slowdown");
    height_ = config->Get<double>("airborne_height_m");
    rampTime_ = config->Get<double>("ramp_time_s");
    fundamental_ = config->Get<double>("fundamental_hz");
    scaleMin_ = config->Get<double>("frequency_scale_min");
    scaleMax_ = config->Get<double>("frequency_scale_max");
    if (!std::isfinite(reference_) || reference_ <= 0 || !std::isfinite(minimum_) || minimum_ <= 0 ||
        minimum_ >= reference_ || !std::isfinite(slowdown_) || slowdown_ <= 0 ||
        !std::isfinite(height_) || height_ <= 0 || !std::isfinite(rampTime_) || rampTime_ <= 0 ||
        !std::isfinite(fundamental_) || fundamental_ <= 0 || !std::isfinite(scaleMin_) ||
        scaleMin_ <= 0 || !std::isfinite(scaleMax_) || scaleMax_ < scaleMin_) {
      gzerr << "[FlightVibration] invalid envelope / rotor configuration\n";
      return;
    }
    for (auto rotor = config->GetElement("rotor"); rotor;
         rotor = rotor->GetNextElement("rotor")) {
      const auto joint = model_.JointByName(ecm, rotor->Get<std::string>("joint"));
      if (joint == gz::sim::kNullEntity) {
        gzerr << "[FlightVibration] rotor joint missing\n";
        return;
      }
      joints_.push_back(joint);
      if (!ecm.Component<gz::sim::components::JointVelocity>(joint))
        ecm.CreateComponent(joint, gz::sim::components::JointVelocity({0.0}));
    }
    if (joints_.size() != 4) {
      gzerr << "[FlightVibration] exactly four rotors required\n";
      return;
    }
    torque_ = Components(config, "torque_component", "frequency_hz", "amplitude_nm");
    force_ = Components(config, "force_component", "frequency_hz", "amplitude_n");
    gyro_ = Components(config, "gyro_component", "harmonic", "amplitude_rad_s");
    accel_ = Components(config, "accel_component", "harmonic", "amplitude_m_s2");
    const std::string raw = config->Get<std::string>("raw_imu_topic");
    const std::string output = config->Get<std::string>("output_imu_topic");
    if (raw.empty() || output.empty() || raw == output) {
      gzerr << "[FlightVibration] input/output topics must be distinct\n";
      return;
    }
    publisher_ = node_.Advertise<gz::msgs::IMU>(output);
    if (!publisher_ || !node_.Subscribe(raw, &FlightVibration::OnImu, this)) {
      gzerr << "[FlightVibration] could not establish IMU relay\n";
      return;
    }
    valid_ = true;
    gzmsg << "[FlightVibration] enabled for " << model_.Name(ecm)
          << ": " << raw << " -> " << output << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (!valid_ || info.paused) return;
    const auto pose = gz::sim::worldPose(link_, ecm);
    if (!haveInitialZ_) { initialZ_ = pose.Pos().Z(); haveInitialZ_ = true; }
    double speed = 0;
    for (const auto joint : joints_) {
      const auto component = ecm.Component<gz::sim::components::JointVelocity>(joint);
      if (component && !component->Data().empty()) speed += std::abs(component->Data()[0]) * slowdown_;
    }
    speed /= joints_.size();
    const bool airborne = pose.Pos().Z() > initialZ_ + height_;
    const bool active = speed > minimum_ && airborne;
    const double dt = std::chrono::duration<double>(info.dt).count();
    // Immediate ground/rotor suppression; smooth onset in flight. No ground torque.
    gate_ = active ? Ramp(gate_, true, dt, rampTime_) : 0.0;
    const double frequency = fundamental_ * std::clamp(speed / reference_, scaleMin_, scaleMax_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sensorGate_ = gate_;
      sensorFrequency_ = frequency;
    }
    if (gate_ == 0) return;
    const double time = std::chrono::duration<double>(info.simTime).count();
    const auto torque = Signal(torque_, kTwoPi * time, gate_);
    const auto force = Signal(force_, kTwoPi * time, gate_);
    const gz::math::Vector3d local(torque[0], torque[1], torque[2]);
    const gz::math::Vector3d localForce(force[0], force[1], force[2]);
    gz::sim::Link(link_).AddWorldWrench(ecm, pose.Rot().RotateVector(localForce),
                                      pose.Rot().RotateVector(local));
  }

 private:
  static std::vector<SignalComponent> Components(const sdf::ElementPtr &config,
      const std::string &tag, const std::string &rate, const std::string &amplitude) {
    std::vector<SignalComponent> result;
    if (!config->HasElement(tag)) return result;
    for (auto e = config->GetElement(tag); e; e = e->GetNextElement(tag)) {
      SignalComponent c;
      c.rate = e->Get<double>(rate);
      const auto a = e->Get<gz::math::Vector3d>(amplitude);
      const auto p = e->Get<gz::math::Vector3d>("phase_rad");
      for (unsigned int i = 0; i < 3; ++i) { c.amplitude[i] = a[i]; c.phase[i] = p[i]; }
      result.push_back(c);
    }
    return result;
  }

  void OnImu(const gz::msgs::IMU &input) {
    // Preserve source stamp/orientation; identify the opt-in relay for PX4 timing.
    auto output = input;
    auto *marker = output.mutable_header()->add_data();
    marker->set_key("rocket_flight_vibration");
    marker->add_value("1");
    const double time = input.header().stamp().sec() + input.header().stamp().nsec() * 1e-9;
    std::array<double, 3> gyro{}, accel{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (haveSensorTime_ && time >= lastSensorTime_) {
        sensorPhase_ = std::fmod(sensorPhase_ + kTwoPi * sensorFrequency_ * (time - lastSensorTime_), kTwoPi);
      } else {
        sensorPhase_ = 0;
      }
      haveSensorTime_ = true;
      lastSensorTime_ = time;
      gyro = Signal(gyro_, sensorPhase_, sensorGate_);
      accel = Signal(accel_, sensorPhase_, sensorGate_);
    }
    auto *g = output.mutable_angular_velocity();
    g->set_x(g->x() + gyro[0]); g->set_y(g->y() + gyro[1]); g->set_z(g->z() + gyro[2]);
    auto *a = output.mutable_linear_acceleration();
    a->set_x(a->x() + accel[0]); a->set_y(a->y() + accel[1]); a->set_z(a->z() + accel[2]);
    publisher_.Publish(output);
  }

  bool valid_ = false, haveInitialZ_ = false, haveSensorTime_ = false;
  double initialZ_ = 0, reference_ = 0, minimum_ = 0, slowdown_ = 0;
  double height_ = 0, rampTime_ = 0, fundamental_ = 0, scaleMin_ = 0, scaleMax_ = 0;
  double gate_ = 0, sensorGate_ = 0, sensorFrequency_ = 0;
  double sensorPhase_ = 0, lastSensorTime_ = 0;
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity link_ = gz::sim::kNullEntity;
  std::vector<gz::sim::Entity> joints_;
  std::vector<SignalComponent> torque_, force_, gyro_, accel_;
  std::mutex mutex_;
  gz::transport::Node::Publisher publisher_;
  // Destroy the subscription node before the publisher and callback state.
  gz::transport::Node node_;
};
}  // namespace rocket

GZ_ADD_PLUGIN(rocket::FlightVibration, gz::sim::System,
              rocket::FlightVibration::ISystemConfigure,
              rocket::FlightVibration::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(rocket::FlightVibration, "rocket::FlightVibration")
