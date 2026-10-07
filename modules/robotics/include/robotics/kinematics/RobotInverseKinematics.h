#pragma once

#include "robotics/kinematics/RobotKinematics.h"

#include <cstddef>
#include <string>

namespace grasplink::robotics::kinematics
{
/**
 * @brief IK 반복 계산이 목표 자세를 찾았는지와 찾지 못한 이유를 구분한다.
 * @details Unreachable은 링크 길이로 계산한 보수적 거리 상한 밖에 있는 목표만 뜻한다.
 * 상한 안에 있다는 사실만으로 실제로 도달 가능하다고 판단하지 않는다.
 * JointLimitReached는 관절 경계에 막혀 오차를 더 줄이지 못한 경우이고 DidNotConverge는 국소 계산의 반복 한도 또는 정체다.
 * 두 실패 모두 다른 시작 관절각에서 해를 찾을 가능성을 배제하지 않는다.
 */
enum class IkStatus { Success, InvalidInput, MissingToolFrame, Unreachable, JointLimitReached, DidNotConverge };

/**
 * @brief 위치와 방향의 허용 오차 및 Damped Least Squares 반복 계산 설정을 정한다.
 * @details damping은 팔을 펴서 일부 방향으로 움직일 수 없는 특이 자세에서도 작은 수로 나누어 관절 변화가 폭증하는 것을 줄이는 양수 계수다.
 * 이 값에서 시작해 오차 감소가 확인되면 최소 1%까지 낮추고 감소하지 않으면 높여 안정성과 작은 잔여 오차를 함께 다룬다.
 * orientationWeightMetersPerRadian은 회전 오차 [rad]를 위치 오차 [m]와 함께 최소화하기 위한 길이 비율이며 실제 공구 길이나 속도 제한은 아니다.
 * 수렴은 위치와 방향의 개별 허용 오차를 모두 만족해야 하며 반복 계산만으로 항상 해를 찾는다고 보장하지 않는다.
 */
struct IkOptions
{
    std::size_t maxIterations = 200;
    double positionToleranceMeters = 1e-5;
    double orientationToleranceRadians = 1e-4;
    double damping = 1e-3;
    double orientationWeightMetersPerRadian = 0.3;
    double maxJointStepRadians = 0.2;
};

/**
 * @brief IK의 성공 여부와 가장 가까워진 관절각, 잔여 오차를 값으로 반환한다.
 * @details jointPositionRadians는 RobotSpecification 순서의 [rad] 값이다.
 * 실패 결과의 관절각은 진단용 후보이므로 Controller 목표로 실행하지 않는다.
 * iterations는 수행한 반복 수이며 오차는 Robot base 기준 위치 거리 [m]와 최단 회전각 [rad]다.
 * 입력이나 거리 사전검사에서 반환되어 FK 오차를 계산하지 않았다면 두 오차 값은 infinity다.
 */
struct IkResult
{
    IkStatus status = IkStatus::InvalidInput;
    JointVector jointPositionRadians;
    std::size_t iterations = 0;
    double positionErrorMeters = 0.0;
    double orientationErrorRadians = 0.0;
    std::string message;
    [[nodiscard]] bool Ok() const noexcept { return status == IkStatus::Success; }
    explicit operator bool() const noexcept { return Ok(); }
};

/**
 * @brief TCP 목표 위치·방향과 현재 관절각에서 목표 관절각을 찾는다.
 * @details IK(역기구학)는 공구 끝의 원하는 위치·방향을 만드는 관절각을 찾는 계산이고 FK는 관절각에서 그 자세를 구하는 계산이다.
 * ToolFrame은 모델 공구 장착 기준점이고 TCP는 작업 기준점이며 tcpInToolFrame의 고정 변환으로 둘을 연결한다.
 * 기본 변환은 항등이라 TCP를 ToolFrame에 놓지만 실제 공구가 길면 그 공구의 offset을 명시적으로 전달해야 한다.
 * 모든 목표와 결과 위치는 Robot base 기준이며 Scene의 robotRoot World 배치는 포함하지 않는다.
 * 현재 관절각을 시작점으로 사용하는 국소 계산이며 다른 IK 분기를 무작위로 탐색하지 않는다.
 * 사양을 빌리므로 그 배열과 문자열은 계산기보다 오래 살아야 하고 같은 객체를 여러 스레드에서 동시에 사용하지 않는다.
 */
class DampedLeastSquaresIk final
{
public:
    /** @brief 사양과 ToolFrame 기준 TCP 고정 변환 [m], quaternion [w,x,y,z]를 검사해 계산기를 만든다. 잘못된 사양이나 변환에는 std::invalid_argument를 던진다. */
    explicit DampedLeastSquaresIk(const models::RobotSpecification& specification, models::Pose3 tcpInToolFrame = {});

    /**
     * @brief 현재 관절각에 가까운 목표 관절각을 관절 한계 안에서 반복 계산한다.
     * @param targetInBase TCP 목표 위치 [m]와 quaternion [x,y,z,w]다. 유한하고 길이가 0이 아닌 quaternion은 정규화한다.
     * @param currentSeed 사양 순서의 시작 관절각 [rad]이며 유한하고 관절 제한 안에 있어야 한다.
     * @param options 반복 한도와 위치·방향 오차, damping 설정이다. 모든 실수 설정은 유한한 양수여야 한다.
     * @return 성공 시 실행 가능한 목표 관절각이다. 실패 시 상태를 변경하지 않고 실패 이유와 진단 후보를 반환한다.
     */
    [[nodiscard]] IkResult Solve(const CartesianPose& targetInBase, const JointVector& currentSeed, const IkOptions& options = {});

    /**
     * @brief 지정한 시작 관절각 하나에서만 IK를 계산한다.
     * @details 다른 자세 가지를 추가로 탐색하지 않는다. Controller가 각 후보를 따로 충돌 검사할 때 중복 탐색을 막는 용도다.
     */
    [[nodiscard]] IkResult SolveSingleSeed(const CartesianPose& targetInBase,
        const JointVector& seed, const IkOptions& options = {});

    /**
     * @brief 관절각의 FK에 TCP offset을 적용해 Robot base 기준 TCP 자세를 계산한다.
     * @param jointPositionRadians 사양 순서의 관절각 [rad]다. FK와 같이 이 함수 자체는 관절 한계를 검사하지 않는다.
     * @return 모델로 계산한 TCP 위치 [m]와 quaternion [x,y,z,w]다. 물리 장치에서 측정한 feedback은 아니다.
     * @throws std::invalid_argument ToolFrame이 없거나 관절 수·각도가 잘못된 경우다.
     */
    [[nodiscard]] CartesianPose EvaluateTcp(const JointVector& jointPositionRadians);

private:
    const models::RobotSpecification& specification_;
    RobotKinematics forward_;
    models::Pose3 tcpInToolFrame_;
    double maximumReachMeters_ = 0.0;
};
}
