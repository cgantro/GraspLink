#pragma once

#include "IPoseSource.h"

namespace PoseLink
{

/*
    네트워크나 카메라 없이
    Pose pipeline 자체를 검증하기 위한 입력원.

    같은 timeSeconds가 들어오면
    항상 같은 Pose가 나온다.
*/
class SyntheticPoseSource final : public IPoseSource
{
public:
    std::optional<Pose> Sample(double timeSeconds) override;
};

} // namespace PoseLink