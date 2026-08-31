#pragma once
#include "poselink/pose/Pose.h"
#include <array>
#include <cstdint>
#include <string>
namespace poselink { constexpr uint32_t kProtocolMagic=0x504C4E4B; constexpr uint8_t kProtocolVersion=1; constexpr size_t kPosePacketSize=64; struct DecodedPacket { uint32_t sequence{}; PoseSample pose{}; }; std::array<uint8_t,kPosePacketSize> EncodePose(uint32_t sequence,const PoseSample& pose); bool DecodePose(const uint8_t* data,size_t size,DecodedPacket& output,std::string& error); bool IsFiniteAndNormalized(const PoseSample& pose); }
