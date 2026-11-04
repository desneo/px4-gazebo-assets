#include "zhongke_d80/truth_tracking.hh"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace zhongke_d80;
void Check(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
int main(){
  const double roundedBoundary = std::nextafter(300.0, std::numeric_limits<double>::infinity());
  TruthProjection boundary{1,"boundary",960,540,300,roundedBoundary};
  Check(CanAcquire(boundary,1920,1080,300),"恰好300m的机器舍入误差仍可绑定");
  Check(CanonicalDiscoveryRange(roundedBoundary,300)==300,"候选发布与绑定共用边界规范化");
  const double roundTrip=std::stod(PreciseNumber(CanonicalDiscoveryRange(roundedBoundary,300)));
  Check(roundTrip==300 && CanAcquire(boundary,1920,1080,300)==(roundTrip<=300),
        "同快照候选序列化与插件绑定距离判定一致");
  for(double coordinate:{.07969999782134567,-.0000101267364531287,1.4850000091234567,300.00000000000006})
    Check(std::stod(PreciseNumber(coordinate))==coordinate,"几何位置及距离保留double往返精度");
  for(double delta:{1e-9,1e-6})
    Check(std::stod(PreciseNumber(CanonicalDiscoveryRange(300+delta,300)))>300,
          "实际超距候选不会因序列化舍入伪装成300m");
  boundary.range_m=300.0+1e-9;
  Check(!CanAcquire(boundary,1920,1080,300),"300m加1纳米不能当作机器舍入容差接受");
  boundary.range_m=300.0+1e-6;
  Check(!CanAcquire(boundary,1920,1080,300),"业务超距300m加1微米必须拒绝");
  for(double range:{299.,300.,301.}){
    const auto p=ProjectTruthTargetCalibrated(1,"target",{range,0,0},1920,1080,15628.7,15659.5,960,540,.005,30000);
    Check(p && CanAcquire(*p,1920,1080,300)==(range<=300),"发现距离299/300/301边界");
    Check(p && MakeTruthObservation(*p,{944,524,32,32},299,1920,1080,8).status==TruthStatus::VISIBLE,
          "持续观测不限制发现距离");
  }
  const auto diagonal=ProjectTruthTargetCalibrated(1,"target",{299,-18,18},1920,1080,1000,1000,960,540,.005,30000);
  Check(diagonal && diagonal->depth<300 && diagonal->range_m>300 && !CanAcquire(*diagonal,1920,1080,300),
        "斜向候选按三维range拒绝，不误用前向depth");
  Check(!ProjectTruthTargetCalibrated(1,"target",{-1,0,0},1920,1080,1000,1000,960,540,.005,30000),"相机后方无候选");
  Check(!ProjectTruthTargetCalibrated(1,"target",{std::nan(""),0,0},1920,1080,1000,1000,960,540,.005,30000),"无效位置无候选");
  for(double u:{-1.,1920.}){
    TruthProjection p{1,"target",u,540,20,20};
    Check(!CanAcquire(p,1920,1080,300),"画面外中心不能新绑定");
    Check(MakeTruthObservation(p,{0,0,32,32},20,1920,1080,9).status==TruthStatus::INVISIBLE,"出画面不沿用旧有效目标");
  }
  const auto a=ProjectTruthTargetCalibrated(1,"a",{50,-1,1},1920,1080,1000,2000,960,540,.005,30000);
  const auto b=ProjectTruthTargetCalibrated(2,"b",{50,-1,1},1920,1080,2000,4000,960,540,.005,30000);
  Check(a && b && std::abs((b->center_x-960)-2*(a->center_x-960))<1e-10 &&
        std::abs((b->center_y-540)-2*(a->center_y-540))<1e-10,"焦距翻倍投影使用同源双向内参");
  std::cout<<"中科候选投影与发现距离契约通过\n";
}
