#include "zhongke_d80/control.hh"
#include <cmath>
#include <iostream>
#include <limits>
using namespace zhongke_d80;
namespace {
int failures=0;
void Check(bool ok,const char *message) {
  if(!ok) {std::cerr << "失败：" << message << '\n'; ++failures;}
}
void Bound(const std::array<double,2> &output,double rate) {
  for(double value:output)
    Check(std::isfinite(value) && std::abs(value)<=rate+1e-10,"所有运动模式均满足配置实际限速");
}
}
int main() {
  constexpr double d=M_PI/180;
  ControlConfig defaults;
  Check(defaults.max_rate_dps==20,"默认每轴20度每秒");
  Check(defaults.yaw_min_deg==-90 && defaults.yaw_max_deg==90 &&
        defaults.pitch_min_deg==-90 && defaults.pitch_max_deg==30,"默认中科两轴机械范围");
  for(double rate:{10.,20.,30.}) {
    Check(JointRateLimit(rate)==rate,"合法10/20/30配置不得被固定20或60截断");
    ControlConfig cfg;
    cfg.max_rate_dps=rate;
    cfg.yaw_min_deg=-90; cfg.yaw_max_deg=90;
    cfg.pitch_min_deg=-90; cfg.pitch_max_deg=30;
    GimbalControl manual; manual.Configure(cfg);
    for(int i=0;i<60;++i) {
      double time=i*.01;
      manual.Manual(1000,-1000,time);
      Bound(manual.Step(time,false,{},0,0,{},time),rate);
    }
    Check(std::abs(manual.cmd()[0]-rate)<1e-9 && std::abs(manual.cmd()[1]+rate)<1e-9,
          "人工饱和能达到当前配置速度");
    manual.Manual(0,0,.6);
    Bound(manual.Step(.6,false,{},0,0,{},.6),0);
    GimbalControl home; home.Configure(cfg); home.GoHome();
    JointSample displaced{80*d,-80*d,0,0};
    for(int i=0;i<60;++i) Bound(home.Step(i*.01,false,{},0,0,displaced,i*.01),rate);
    Check(std::abs(home.cmd()[0]+rate)<1e-9 && std::abs(home.cmd()[1]-rate)<1e-9,
          "大偏角HOME沿同一配置限速连续运动");
    for(const std::string axis:{"yaw","pitch"}) {
      GimbalControl reset; reset.Configure(cfg); Check(reset.ResetAxis(axis,displaced),"接受机械单轴回零");
      for(int i=0;i<60;++i) Bound(reset.Step(i*.01,false,{},0,0,displaced,i*.01),rate);
      Check(reset.cmd()[axis=="yaw"?1:0]==0,"单轴回零保持另一轴");
    }
    GimbalControl track; track.Configure(cfg);
    Check(track.StartTruthTracking({.7,.1,.2,.2},1920,1080).first,"真值跟踪无需检测网络");
    TruthObservation observation; observation.status=TruthStatus::VISIBLE;
    observation.box={1400,100,160,120}; observation.center_x=1480; observation.center_y=160;
    for(int i=0;i<60;++i) {
      observation.frame_seq=i;
      Bound(track.Step(i*.01,true,{},1920,1080,{},i*.01,i,&observation),rate);
    }
    for(double pitch:{-M_PI/2,-89.99*d,-88*d})
      for(double value:ImageRates(pitch,1000,1000)) Check(std::isfinite(value),"负90度附近速度解有限");
  }
  for(double invalid:{0.,-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
    Check(JointRateLimit(invalid)==0,"非法速度关闭运动输出");
  std::cout << (failures?"中科控制契约未通过":"中科控制契约通过") << '\n';
  return failures?1:0;
}
