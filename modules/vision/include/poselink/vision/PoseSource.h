#pragma once
#include "poselink/pose/Pose.h"
#include <memory>
namespace poselink { class IPoseSource { public: virtual ~IPoseSource()=default; virtual PoseSample Next(uint64_t nowUs)=0; }; std::unique_ptr<IPoseSource> MakeSyntheticPoseSource(uint32_t objectId); std::unique_ptr<IPoseSource> MakeArUcoPoseSource(uint32_t objectId,const char* calibrationPath,double markerSizeM); }
