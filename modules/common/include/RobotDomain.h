#pragma once

#include <array>

namespace PoseLink
{
/**
 * @brief HCR-12A의 J1~J6 joint angle 상태.
 *
 * 순서는 J1부터 J6, 단위는 radian이다. renderer matrix나 degree 표현을 포함하지 않아
 * kinematics 계산과 simulation state에서 동일한 domain value로 사용한다.
 */
struct JointState
{
    std::array<float, 6U> radians{};
};
} // namespace PoseLink
