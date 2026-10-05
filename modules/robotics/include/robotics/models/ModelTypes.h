#pragma once

namespace grasplink::robotics::models
{

// Vec3의 단위와 좌표계는 사용하는 필드가 정한다.

struct Vec3
{
    // 단위와 기준은 이 값을 담는 필드의 의미를 따른다.
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 회전축 방향값을 나타낼 때 Vec3와 같은 형식을 사용한다.
using Axis3 = Vec3;

struct QuaternionWxyz
{
    // 성분 순서: w, x, y, z
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Pose3
{
    // 위치 [m]와 회전. 기준 frame은 이 필드를 포함한 specification에서 정한다.
    Vec3 positionMeters{};
    QuaternionWxyz rotation{};
};

} // namespace grasplink::robotics::models
