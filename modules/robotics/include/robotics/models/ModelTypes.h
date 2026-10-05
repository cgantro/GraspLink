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

/** @brief 회전축 방향을 나타내는 Vec3 별칭. 단위와 기준 frame은 사용하는 필드가 정한다. */
using Axis3 = Vec3;

/** @brief w, x, y, z 순서의 회전 사원수. 기본값은 회전이 없는 항등 회전이다. */
struct QuaternionWxyz
{
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/**
 * @brief 위치와 회전을 함께 나타내는 강체 pose.
 * @details 위치 단위는 meter다. 기준 frame은 해당 pose 필드의 계약이 정한다.
 */
struct Pose3
{
    Vec3 positionMeters{};
    QuaternionWxyz rotation{};
};

} // namespace grasplink::robotics::models
