#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <vector>

namespace grasplink::robotics::kinematics
{

/**
 * @brief 한 번의 FK 계산에서 만든 관절 회전과 Robot base 기준 pose 묶음.
 * @details 같은 값을 화면용 GLB 계층과 Kinematic 충돌 proxy에 각각 전달한다. 두 경로는 이 결과를 소비하며
 * Entity 변환을 다시 FK 입력으로 읽지 않는다. 위치 단위는 [m]이고 quaternion은 모델 형식의 [w,x,y,z] 순서다.
 * toolFrameInBaseFrame은 장착 공구 TCP나 Controller feedback을 뜻하지 않는다.
 */
struct RobotKinematicState
{
    /// 각 관절의 bind frame 기준 Local 회전. adapter 적용 대상 GLB의 초기 관절 회전은 항등이어야 한다.
    std::vector<models::QuaternionWxyz> jointLocalRotations;
    /// 모델 joint 순서의 관절 중심 [m]과 base부터 누적한 방향을 담는 pose 배열.
    std::vector<models::Pose3> linkPosesInBaseFrame;
    /// 마지막 관절 pose에 고정 ToolFrame 변환을 합성한 Robot base 기준 pose.
    models::Pose3 toolFrameInBaseFrame{};
    /// RobotSpecification에 유효한 ToolFrame 정의가 있을 때만 true. Controller TCP 유효성과 독립이다.
    bool toolFrameValid = false;
};

/**
 * @brief 관절각을 Robot base 기준 관절 pose로 변환하는 순방향 운동학 계산기.
 * @details joints는 Controller 순서의 직렬 체인이고 bindPivotMeters는 모두 Robot base 기준 bind 위치다.
 * joint axis는 항등 관절 bind 회전에서의 local 방향이다. 각 관절의 local 회전을 부모 누적 회전에 오른쪽으로
 * 합성한다. Local 점에는 해당 관절 회전 후 부모 누적 회전이 적용된다.
 * Scene에 배치한 robot root 변환은 결과에 포함하지 않는다.
 * 모델 참조는 빌림이며 객체보다 오래 살아야 한다.
 */
class RobotKinematics final
{
public:
    /**
     * @brief 모델 참조를 검사하고 FK 출력 공간을 준비한다.
     * @param specification 유효한 joints 배열과 선택적 ToolFrame을 제공하는 빌린 모델 참조.
     * @throws std::invalid_argument 관절 배열이 비었거나 pivot, 축 또는 지정된 ToolFrame이 유효하지 않을 때.
     */
    explicit RobotKinematics(const models::RobotSpecification& specification);

    /**
     * @brief 유효한 Controller snapshot의 관절각으로 FK 결과를 갱신한다.
     * @param state J1..Jn 순서의 현재 관절 위치 [rad]. TCP feedback은 FK 입력에 사용하지 않는다.
     * @return 이 객체가 소유하는 Robot base 기준 결과. 다음 Update 호출에서 같은 저장 공간을 덮어쓴다.
     * @throws std::invalid_argument snapshot이 유효하지 않거나 관절 수가 다르거나 각도가 유한수가 아닐 때.
     */
    const RobotKinematicState& Update(const RobotState& state);

private:
    const models::RobotSpecification& specification_;
    RobotKinematicState state_;
};

}
