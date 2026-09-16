#pragma once
#include "Pose.h"

#include <optional>

namespace PoseLink
{
/*
    Pose 생성하는 입력원의 공통 인터페이스
    SystheticPoseSoruce: 주어진 시간으로 결정적인 Pose 생성

    ArUcoPoseSource:
        카메라에서 Pose 추정
        검출 실패 시 std::nullopt 반환 가능
*/
class IPoseSource
{
private:
    /* data */
public:

    virtual ~IPoseSource() = default;
    virtual std::optional<Pose> Sample(double timeSeconds) = 0;
};


} // namespace PoseLink
