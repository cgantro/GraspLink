#pragma once

namespace grasplink::robotics::models
{

/**
 * @brief 모델 상수의 3차원 벡터 형식.
 * @details 단위와 좌표계는 이 형식을 쓰는 필드가 정한다. 관절 중심은 보통 [m], 회전축은 길이 1인 무차원 방향값이다.
 */
struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/** @brief 3차원 회전축 방향을 저장하기 위한 Vec3 별칭이다. 방향 벡터에는 길이 단위가 없으며 기준 좌표계는 이 별칭을 담은 필드가 정한다. */
using Axis3 = Vec3;

/** @brief 3차원 회전 방향을 네 숫자로 저장하는 quaternion이다. 성분 순서는 w, x, y, z이며 (1,0,0,0)은 회전하지 않는 방향이다. */
struct QuaternionWxyz
{
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/**
 * @brief 물체의 위치와 회전을 한 묶음으로 나타낸다.
 * @details 위치 단위는 meter다. 기준 frame은 해당 pose 필드의 계약이 정한다.
 */
struct Pose3
{
    Vec3 positionMeters{};
    QuaternionWxyz rotation{};
};

} // namespace grasplink::robotics::models
