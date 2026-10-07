#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <array>
#include <optional>

namespace grasplink::robotics::planning
{

/**
 * @brief 현재 TCP 위치를 유지하면서 J6만 돌려 목표 방향을 만들 수 있는지 계산한다.
 * @details 현재 TCP가 J6 축에서 2 mm보다 더 벗어나면 J6만 돌릴 때 공구 끝 위치가 유지되지 않으므로 거부한다.
 * 파지한 상자는 축에서 벗어난 만큼 회전 중 원호를 그린다. 예상 이동이 20 mm를 넘으면 거부하고,
 * 호출자는 회전 뒤 상자 중심을 다시 목표 위치에 맞춰야 한다. 관절 목표는 J1부터 J5까지 현재값을 유지하고,
 * J6은 2π 등가각 중 관절 허용 범위 안에서 현재 각도에 가장 가까운 값을 사용한다.
 */
struct WristAlignmentTarget
{
    /// J1부터 Jn까지의 목표 관절각 [rad]다. 이 계획에서는 J1부터 J5까지 현재값을 유지한다.
    JointVector jointPositionRadians;

    /// J6 회전축 방향을 기준으로 계산한 최단 목표 회전량 [rad]다.
    double rotationRadians = 0.0;

    /// 정지한 TCP를 중심으로 상자 중심이 회전하며 이동할 최대 거리 [m]다.
    double attachedBoxSweepMeters = 0.0;
};

/**
 * @brief TCP가 J6 축과 거의 일치하고 목표 방향이 그 축 주위 회전만으로 만들어질 때 J6 목표를 계산한다.
 * @param specification 관절 순서와 허용 범위를 제공하는 로봇 사양이다.
 * @param state 현재 관절각 [rad]과 TCP 자세 [m, quaternion x,y,z,w]를 담은 Controller 상태다.
 * @param targetOrientationXyzw 목표 방향 quaternion이며 성분 순서는 [x,y,z,w]다.
 * @param boxOffsetInTcpMeters 파지한 상자 중심의 TCP 기준 오프셋 [m]이다. 값이 없으면 상자 이동량은 검사하지 않는다.
 * @return 조건을 만족하면 J6 목표와 예상 상자 이동량을 반환하고, 하나라도 만족하지 않으면 빈 값을 반환한다.
 */
std::optional<WristAlignmentTarget> PlanWristOnlyTarget(
    const models::RobotSpecification& specification,
    const RobotState& state,
    const std::array<double, 4>& targetOrientationXyzw,
    const std::optional<std::array<double, 3>>& boxOffsetInTcpMeters = std::nullopt);

} // namespace grasplink::robotics::planning
