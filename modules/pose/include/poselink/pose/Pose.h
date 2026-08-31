#pragma once
#include <cstdint>
namespace poselink { struct Vec3 { double x{}, y{}, z{}; }; struct Quaternion { double x{}, y{}, z{}, w{1.0}; }; struct PoseSample { uint32_t objectId{}; uint64_t timestampUs{}; Vec3 position{}; Quaternion orientation{}; bool valid{}; }; enum class TrackingState { Lost, Stale, Detected }; }
