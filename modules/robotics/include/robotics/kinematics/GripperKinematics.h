#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/models/GripperSpecification.h"

#include <vector>

namespace grasplink::robotics::kinematics
{

/**
 * @brief 한 Gripper 상태에서 계산한 master 각과 분기 관절의 Local 회전 변화.
 * @details masterAngleRadians는 nominal closed 각에 연속 closureFraction을 곱한 값이다.
 * jointAnglesRadians는 specification 순서의 mimic 관절각이며 jointLocalRotations는 각 joint-local
 * 축 기준 axis-angle 결과 [w,x,y,z]다. 위치와 World pose는 계산하지 않는다. Viewer가 GLB bind 회전에
 * 이 변화량을 합성하고 기존 TransformSystem이 계층의 부모 회전을 전파한다.
 */
struct GripperKinematicState
{
    /// 자유공간 linkage 기준 master 각 [rad].
    double masterAngleRadians = 0.0;

    /// specification 순서의 관절 각 [rad].
    std::vector<double> jointAnglesRadians;

    /// specification 순서의 관절 Local 회전 변화. Quaternion 순서는 [w,x,y,z].
    std::vector<models::QuaternionWxyz> jointLocalRotations;
};

/**
 * @brief 연속 Gripper closure 위치를 분기 mimic 관절 회전으로 변환한다.
 * @details 팔의 직렬 RobotKinematics와 달리 branch linkage만 다룬다. 유효한 closure fraction은
 * 0=open, 1=nominal closed 기준이며 각도는 `masterMultiplier * masterAngleRadians`다.
 * 이 계산은 각 joint-local 회전 변화만 제공한다. bind pivot으로 World 위치를 다시 계산하지 않고,
 * 접촉에 따른 under-actuated 적응이나 grasp도 모델링하지 않는다. GripperState.valid가 true이면
 * inactive 상태여도 현재 연속 위치를 pose로 계산할 수 있다.
 *
 * @throws std::invalid_argument 사양 배열·이름·축·각도 계수·nominal 각 또는 관절 한계가 유효하지 않을 때.
 */
class GripperKinematics final
{
public:
    /**
     * @brief GripperSpecification을 빌려 검사하고 결과 저장 공간을 준비한다.
     * @param specification 관절 배열과 nominal closed master 각을 제공하는 사양. 객체보다 오래 살아야 한다.
     * @throws std::invalid_argument 비어 있거나 중복된 이름, 영벡터/비유한 축, 잘못된 계수 또는
     * nominal open-to-closed 각도를 모두 포함하지 않는 관절 제한이 있을 때.
     */
    explicit GripperKinematics(const models::GripperSpecification& specification);

    /**
     * @brief 유효한 연속 Gripper 위치에서 분기 관절 pose를 갱신한다.
     * @param state closureFraction과 valid 표시를 제공하는 Controller snapshot. raw 8-bit 위치는 사용하지 않는다.
     * @return 이 계산기가 소유한 결과. 다음 유효 Update에서 같은 저장 공간을 덮어쓴다.
     * @throws std::invalid_argument 전체 상태가 invalid이거나 closureFraction이 유효하지 않거나
     * [0,1] 밖·비유한 값인 경우. 거부할 때 이전 결과를 유지한다.
     */
    const GripperKinematicState& Update(const GripperState& state);

private:
    const models::GripperSpecification& specification_;
    GripperKinematicState state_;
    // 두 버퍼를 교환해 실패 시 이전 결과를 보존하고 4 ms tick마다 재할당하지 않는다.
    GripperKinematicState pending_;
};

} // namespace grasplink::robotics::kinematics
