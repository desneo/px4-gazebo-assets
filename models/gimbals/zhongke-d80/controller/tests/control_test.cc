#include "zhongke_d80/control.hh"
#include <cmath>
#include <iostream>
#include <limits>
using namespace zhongke_d80;
namespace {
int failures=0;
void Check(bool ok,const char *message) {if(!ok){std::cerr<<"失败："<<message<<'\n';++failures;}}
}
int main() {
  constexpr double d=M_PI/180;
  ControlConfig cfg; cfg.max_rate_dps=30;
  GimbalControl control; control.Configure(cfg);
  Check(!control.SetAngles(91,0) && !control.SetAngles(0,-91),"指定角拒绝超行程");
  Check(!control.SetAngles(std::nan(""),0),"指定角拒绝非有限值");
  Check(control.SetAngles(45,-30) && control.mode()==Mode::HOME,"指定角采用连续HOME路径");
  JointSample zero{};
  auto cmd=control.Step(0,false,{},0,0,zero,0);
  Check(cmd[0]>0 && cmd[0]<=1.6+1e-9 && cmd[1]<0,"指定角按配置加速度起步");
  control.Manual(30,0,.01);
  control.Step(.01,false,{},0,0,zero,.01);
  Check(control.SetAngles(0,20),"手动后接受指定角");
  cmd=control.Step(.02,false,{},0,0,zero,2);
  Check(control.mode()==Mode::HOME && cmd[1]>0,"旧手动超时不终止后续HOME");
  control.Stop();
  Check(control.cmd()[0]==0 && control.cmd()[1]==0,"停止清除运动输出");
  control.Manual(30,0,3);
  control.Step(.03,false,{},0,0,zero,3);
  cmd=control.Step(.04,false,{},0,0,zero,3.6);
  Check(control.mode()==Mode::IDLE && cmd[0]==0,"有效手动速度超时停车");
  control.Manual(-100,0,4);
  JointSample lower{-89.9*d,0,0,0};
  cmd=control.Step(.05,false,{},0,0,lower,4);
  Check(cmd[0]==0 && control.limit_flags()[0],"软件端点停止向外运动");
  control.Manual(100,0,4.1);
  cmd=control.Step(.06,false,{},0,0,lower,4.1);
  Check(cmd[0]>0,"端点允许反向退出");
  const auto miss=MeasureTargetMiss(970,530,1920,1080,1000,2000,960,540);
  Check(miss.valid && std::abs(miss.offset_deg[0]-std::atan(.01)/d)<1e-10 &&
        std::abs(miss.offset_deg[1]+std::atan(.005)/d)<1e-10,"独立fx/fy准确还原水平右正和垂直下正内部偏差");
  const auto target=ProjectTruthTargetCalibrated(7,"target",{100,-1,1},1920,1080,1000,2000,960,540,.005,30000);
  Check(target && target->center_x==970 && target->center_y==520,"真值投影使用独立fx/fy");
  Check(target && CanAcquire(*target,1920,1080,101) && !CanAcquire(*target,1920,1080,100),"发现距离使用三维距离而非前向深度");
  const TruthProjection boundary{7,"target",960,540,300,300};
  Check(CanAcquire(boundary,1920,1080,300),"距离等于上限允许新绑定");
  auto outside=boundary; outside.range_m=301;
  Check(!CanAcquire(outside,1920,1080,300),"301m不允许重新选取");
  const auto observation=MakeTruthObservation(outside,{944,524,32,32},300,1920,1080,8);
  Check(observation.status==TruthStatus::VISIBLE,"已锁实体超发现距离持续可见");
  outside.center_x=1920;
  Check(MakeTruthObservation(outside,{944,524,32,32},300,1920,1080,9).status==TruthStatus::INVISIBLE,
        "目标中心出画面即失效，框边重叠不能维持锁定");
  const auto forward=MatVec(Ry(M_PI/2),Los(0,0));
  Check(std::abs(forward.x-1)<1e-12 && std::abs(forward.y)<1e-12 && std::abs(forward.z)<1e-12,"一次Y90安装零位前视");
  const auto right=MatVec(Ry(M_PI/2),Los(.01,0));
  const auto up=MatVec(Ry(M_PI/2),Los(0,.01));
  Check(right.y<0 && up.z>0,"前视安装机械yaw右正pitch上正");
  Check(Singular(-M_PI/2),"负90度明确报告视线退化");
  JointSample invalid{}; invalid.yaw=std::nan("");
  cmd=control.Step(.1,false,{},0,0,invalid,5);
  Check(cmd[0]==0 && !control.fault_reason().empty(),"无效机械测量故障停车");
  for(double speed:{10.,20.,30.}) {
    ControlConfig edgeConfig;
    edgeConfig.yaw_min_deg=0; edgeConfig.yaw_max_deg=90;
    edgeConfig.pitch_min_deg=-90; edgeConfig.pitch_max_deg=0;
    edgeConfig.max_rate_dps=speed;
    GimbalControl edgeHome; edgeHome.Configure(edgeConfig); edgeHome.GoHome();
    JointSample plant{30*d,-30*d,0,0};
    bool completed=false;
    for(int i=0;i<2500;++i) {
      const auto output=edgeHome.Step(i*.01,false,{},0,0,plant,i*.01);
      Check(std::abs(output[0])<=speed+1e-10 && std::abs(output[1])<=speed+1e-10,
            "端点零位HOME沿当前配置限速连续运动");
      plant.yaw+=output[0]*.01*d; plant.pitch+=output[1]*.01*d;
      plant.yaw_vel=output[0]*d; plant.pitch_vel=output[1]*d;
      Check(plant.yaw>=-1e-12 && plant.yaw<=90*d+1e-12 &&
            plant.pitch>=-90*d-1e-12 && plant.pitch<=1e-12,
            "端点零位HOME不越过硬行程边界");
      if(edgeHome.mode()==Mode::IDLE) {completed=true;break;}
    }
    Check(completed && std::abs(plant.yaw/d)<.03 && std::abs(plant.pitch/d)<.03,
          "yaw[0,90]/pitch[-90,0]连续HOME可以结束且到达机械零位");
  }
  std::cout<<(failures?"中科控制测试未通过":"中科控制测试通过")<<'\n';
  return failures?1:0;
}
