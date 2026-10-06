#pragma once

#include "robotics/core/IRobotController.h"
#include "robotics/models/RobotSpecification.h"

namespace grasplink::robotics::backends::simulation
{

/**
 * @brief 목표 관절각을 향해 관절 위치와 속도를 계산하는 로봇 Simulation Controller다.
 * @details Controller는 관절 상태만 갱신하며 화면 메시, Scene Entity, 물리 Body는 직접 움직이지 않는다.
 * 앱은 RobotState를 RobotKinematics에 보내 FK를 계산한다.
 * FK는 관절각에서 Link 위치와 방향을 구하며 어댑터가 이 결과를 화면 계층에 적용한다.
 * 한 번의 Update에서 관절은 모델 최대 각속도에 velocityScale을 곱한 한계 안에서 목표를 향한다.
 * 가속도 제한이나 속도를 부드럽게 올리는 ramp는 적용하지 않는다.
 * 역기구학(IK)은 목표 TCP 위치에서 이를 만드는 관절각을 찾는 계산이다.
 * 이 Controller에는 IK와 직선 경로 실행이 없어 MoveLinear은 Unsupported를 반환한다.
 * FK로 계산한 모델 ToolFrame과 Controller가 보고하는 공구 TCP는 서로 다른 값이다.
 * Connect 뒤 관절 상태 valid는 true지만 TCP feedback을 제공하지 않으므로 tcpPoseValid는 false다.
 * 사양 문자열과 배열은 빌려 쓰므로 원본은 Controller보다 오래 살아야 한다.
 */
class SimRobotController final : public IRobotController
{
public:
    /**
     * @brief 관절 이름·각도 제한·최대 속도를 제공하는 로봇 사양을 참조하는 Controller를 만든다.
     * @param specification 관절 제한과 최대 속도를 제공하는 사양. 배열과 문자열 저장소도 수명 동안 유효해야 한다.
     * @throws std::invalid_argument 관절 배열이 비었거나, q=0이 제한 밖이거나, 제한/최대 속도가 유한한 유효값이 아닐 때.
     * @details 시작 상태가 항상 q=0이므로 0을 포함하지 않는 모델은 이 구현의 초기화 계약과 맞지 않는다.
     */
    explicit SimRobotController(const models::RobotSpecification& specification);

    /** @brief 관절각과 속도를 0으로 한 유효한 Simulation 상태로 연결한다.
     * @return 항상 성공하며 TCP feedback은 계산하지 않아 tcpPoseValid는 false다.
     * @details 목표 관절각도 0으로 두고 속도와 가속도 비율을 1.0으로 초기화한다.
     */
    Result Connect() override;

    /** @brief 연결을 해제하고 논리 상태를 Disconnected/invalid로 표시하며 속도를 0으로 만든다. */
    void Disconnect() noexcept override;

    /** @brief 현재 software 연결 여부를 반환한다. */
    [[nodiscard]] bool IsConnected() const noexcept override;

    /**
     * @brief J1..Jn 목표각을 받아 다음 Update부터 관절을 움직인다.
     * @param command 사양과 같은 순서의 목표각 [rad]와 속도·가속도 비율이다.
     * @return 연결되지 않았거나 관절 수, 값, 비율, 범위가 잘못되면 오류를 반환한다.
     * @details 두 비율은 유한한 (0,1] 값이어야 한다.
     * velocityScale은 모델 최대 각속도에 곱해 속도 상한을 정한다.
     * accelerationScale은 보관만 하며 이 Simulation은 가속도 제한에 사용하지 않는다.
     * 새 명령은 현재 목표를 교체한다.
     * 성공은 목표를 받았다는 뜻이며 도달 여부는 GetState에서 확인한다.
     */
    Result MoveJoint(const JointMoveCommand& command) override;

    /**
     * @brief TCP 목표까지 직선 이동을 요청한다.
     * @param command 공통 API의 TCP 목표 위치·방향과 선속도·각속도 상한이다.
     * @return 연결되지 않았으면 NotConnected를, 연결 상태면 항상 Unsupported를 반환한다.
     * @details FK는 관절각에서 모델 ToolFrame 위치와 방향을 계산한다.
     * 이 Controller에는 TCP 목표에서 관절각을 찾는 IK와 직선 경로 실행이 없다.
     * 따라서 FK가 있어도 이 명령은 수행되지 않는다.
     */
    Result MoveLinear(const LinearMoveCommand& command) override;

    /**
     * @brief 현재 관절 위치에서 Simulation 동작을 멈춘다.
     * @return 연결되지 않았으면 NotConnected를 반환하고 연결 상태에서는 성공한다.
     * @details 현재 관절각을 새 목표로 저장하고 속도를 0, mode를 Stopped로 바꾼다.
     * 이는 Simulation 상태 변경이며 장비 보호 정지나 비상 정지가 아니다.
     */
    Result Stop() override;

    /**
     * @brief Controller 상태의 독립된 값 복사본을 반환한다.
     * @return 관절 위치 [rad], 속도 [rad/s], mode와 상태 유효성을 담는다.
     * @details Connect 뒤 관절 상태는 유효하지만 이 Controller는 TCP feedback을 계산하거나 보고하지 않는다.
     * FK로 구한 모델 ToolFrame은 Controller가 보고하는 실제 TCP feedback과 별도다.
     * 반환 vector는 내부 저장소를 빌리지 않는다.
     */
    [[nodiscard]] RobotState GetState() const override;

    /**
     * @brief 경과 시간 [s]만큼 각 관절을 목표각 쪽으로 움직인다.
     * @details 연결되고 Moving 상태이며 시간이 유한한 양수일 때만 적용한다.
     * 한 번의 각도 변화 한도는 모델 최대 각속도 × velocityScale × dtSeconds다.
     * 목표까지의 차이가 한도보다 크면 한도만큼 움직이고, 작으면 남은 차이만큼 움직인다.
     * 이번 각도 변화량을 시간으로 나눈 값을 관절 속도 [rad/s]로 기록한다.
     * 가속도 제한이 없어 새 명령을 받으면 속도가 즉시 바뀔 수 있다.
     * 목표에 도달하면 각도를 정확히 맞추고 속도를 0으로 만든 뒤 Idle로 바꾼다.
     * 잘못된 시간은 오류 없이 무시한다.
     */
    void Update(double dtSeconds) override;

    /** @brief 생성 때 빌린 RobotSpecification을 const 참조로 반환한다.
     * @return Controller 수명 동안 참조 가능한 사양. 소유권은 호출자에게 이전되지 않는다.
     */
    [[nodiscard]] const models::RobotSpecification& GetSpecification() const noexcept;

private:
    // 소유하지 않는 robot model specification 주소.
    const models::RobotSpecification* specification_ = nullptr;

    // 현재 simulated q/dq/mode/fault 등을 저장한다.
    RobotState state_{};

    // MoveJoint가 마지막으로 수락한 J1..Jn 절대 목표각 [rad]. 새 명령은 이 값을 교체한다.
    JointVector targetPositionRadians_;

    // 모델 max velocity에 곱하는 현재 속도 비율. 1.0이면 모델 최대속도.
    double velocityScale_ = 1.0;

    // Command 계약에 보존한다. Simulation은 가속도 제한을 적용하지 않는다.
    double accelerationScale_ = 1.0;

    // Simulation backend가 Connect되어 사용 가능한 상태인지 나타낸다.
    bool connected_ = false;
};

} // namespace grasplink::robotics::backends::simulation
