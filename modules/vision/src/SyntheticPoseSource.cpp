#include "poselink/vision/PoseSource.h"
#include <cmath>
namespace poselink { namespace { class Synthetic final:public IPoseSource { uint32_t id; uint64_t start{}; public: explicit Synthetic(uint32_t i):id(i){} PoseSample Next(uint64_t now) override {if(!start)start=now;double t=double(now-start)/1e6;double h=t*.5;return {id,now,{.5*std::sin(t),0.0,2.0},{0.0,std::sin(h),0.0,std::cos(h)},true};}};} std::unique_ptr<IPoseSource> MakeSyntheticPoseSource(uint32_t id){return std::make_unique<Synthetic>(id);} }
