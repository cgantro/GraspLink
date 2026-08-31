#pragma once
#include "poselink/pose/Pose.h"
#include <cstddef>
#include <deque>
namespace poselink { struct NetworkMetrics { uint64_t received{},lost{},reordered{},duplicates{},invalid{}; double jitterUs{}; uint64_t lastArrivalUs{}; uint32_t lastSequence{}; bool hasSequence{}; }; class SequenceAnalyzer { public: void Observe(uint32_t sequence,uint64_t arrivalUs,NetworkMetrics& metrics); }; class PoseBuffer { public: explicit PoseBuffer(size_t maxSamples=256,uint64_t retentionUs=2'000'000); void Push(const PoseSample& pose); bool Sample(uint64_t renderTimeUs,PoseSample& out,bool& interpolated) const; size_t Size() const { return m_samples.size(); } private: size_t m_maxSamples; uint64_t m_retentionUs; std::deque<PoseSample> m_samples; }; Quaternion Slerp(const Quaternion& a,const Quaternion& b,double t); Vec3 Lerp(const Vec3& a,const Vec3& b,double t); }
