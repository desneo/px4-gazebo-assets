// 一次性手抛初速；触发后由 Gazebo 物理和 PX4 控制飞行。
#include <atomic>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
namespace assets {
class HandLaunch: public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 gz::sim::Entity model; gz::transport::Node node; std::atomic<bool> pending{false}; double speed=14,up=2,mass=1,remaining=0,duration=.14; bool used=false;
 public: void Configure(const gz::sim::Entity &e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager &,gz::sim::EventManager &) override {
 model=e;speed=s->Get<double>("speed",14).first;up=s->Get<double>("upward",2).first;mass=s->Get<double>("mass",1).first;
 node.Subscribe("/model/xiaobianfu/hand_launch",&HandLaunch::OnLaunch,this);
 }
 void OnLaunch(const gz::msgs::Boolean &m){if(m.data())pending=true;}
 void PreUpdate(const gz::sim::UpdateInfo&i,gz::sim::EntityComponentManager &ecm) override {
 if(i.paused)return;
 if(pending.exchange(false)&&!used){remaining=duration;used=true;}
 if(remaining<=0)return;
 auto e=gz::sim::Model(model).LinkByName(ecm,"base_link");
 auto rotation=gz::sim::worldPose(model,ecm).Rot();
 auto force=rotation.RotateVector(gz::math::Vector3d(mass*speed/duration,0,mass*up/duration));
 force.Z()+=mass*9.80665;
 gz::sim::Link(e).AddWorldForce(ecm,force);
 remaining-=std::chrono::duration<double>(i.dt).count();
 }
};}
GZ_ADD_PLUGIN(assets::HandLaunch,gz::sim::System,assets::HandLaunch::ISystemConfigure,assets::HandLaunch::ISystemPreUpdate)
