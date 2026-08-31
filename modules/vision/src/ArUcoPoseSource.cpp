#include "poselink/vision/PoseSource.h"
#include <stdexcept>
namespace poselink { std::unique_ptr<IPoseSource> MakeArUcoPoseSource(uint32_t,const char*,double){ throw std::runtime_error("ArUco runtime source requires an OpenCV camera implementation; use synthetic until camera arguments are configured"); } }
