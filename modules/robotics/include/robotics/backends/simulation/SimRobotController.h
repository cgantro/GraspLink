#pragma once

#include "robotics/core/IRobotController.h"
#include "robotics/models/RobotSpecification.h"

namespace grasplink::robotics::backends::simulation
{

/**
 * @brief RobotSpecification 기반으로 joint state를 시간에 따라 추종시키는 Simulation용 Robot Controller.
 *
 * @details
 * 이 클래스는 실제 GLB/Flecs Entity를 직접 수정하지 않는다. 내부적으로 RobotState와 target joint angle만 관리하고,
 * Viewer에서는 RobotTransformAdapter가 GetState() 결과를 시각 transform으로 변환한다.
 *
 * 현재 구현 범위:
 * - Connect/Disconnect 상태 관리
 * - JointMoveCommand의 joint count/angle limit 검증
 * - specification의 maxVelocity [rad/s]와 velocityScale을 이용한 위치 추종
 * - Stop() 및 RobotState feedback
 *
 * 아직 구현하지 않은 범위:
 * - acceleration limit 값 자체가 specification에 없으므로 accelerationScale은 저장만 하고 적용하지 않음
 * - FK/IK가 연결되지 않아 MoveLinear()는 Unsupported
 * - TCP pose는 아직 계산하지 않으므로 tcpPoseValid=false
 * - Fixed Control Loop는 호출자가 일정한 dt로 Update()를 호출하는 계층에서 구성해야 함
 */
class SimRobotController final : public IRobotController
{
public:
    /**
     * @brief Simulation이 사용할 robot 정적 규격을 연결한다.
     * @param specification 프로그램 수명 동안 유효해야 하는 RobotSpecification.
     * @throws std::invalid_argument joint 배열이 null이거나 jointCount가 0인 경우.
     *
     * @note specification의 소유권을 가져오지 않고 pointer만 저장한다.
     */
    explicit SimRobotController(const models::RobotSpecification& specification);

    /**
     * @brief 관절 위치/속도를 0으로 초기화하고 Idle simulation state로 전환한다.
     * @return 정상 초기화 시 Result::Success().
     */
    Result Connect() override;

    /** @brief 연결 상태를 해제하고 velocity를 0으로 만든 뒤 RobotState를 invalid 처리한다. */
    void Disconnect() noexcept override;

    /** @return Connect 이후 Disconnect 전이면 true. */
    [[nodiscard]] bool IsConnected() const noexcept override;

    /**
     * @brief 절대 joint target을 검증하고 simulation target으로 설정한다.
     * @param command targetPositionRadians [rad], velocity/acceleration scale (0,1].
     * @return joint count 불일치, angle limit 초과, scale 오류를 InvalidCommand로 반환한다.
     */
    Result MoveJoint(const JointMoveCommand& command) override;

    /**
     * @brief Cartesian 직선 이동 요청.
     * @return 현재 FK/IK 계층이 없으므로 연결 상태에서는 ErrorCode::Unsupported.
     */
    Result MoveLinear(const LinearMoveCommand& command) override;

    /**
     * @brief 현재 위치를 새 target으로 고정하고 joint velocity를 0으로 만든다.
     * @return 연결되어 있지 않으면 NotConnected, 성공하면 Stopped mode로 전환한다.
     */
    Result Stop() override;

    /** @return 현재 Simulation RobotState의 값 복사본. 각도 [rad], 속도 [rad/s]. */
    [[nodiscard]] RobotState GetState() const override;

    /**
     * @brief 현재 위치를 target 방향으로 한 simulation step 진행한다.
     * @param dtSeconds step 시간 [s]. 양수 finite 값만 motion에 반영한다.
     *
     * 각 joint의 한 step 최대 이동량은
     * `maxStep = maxVelocityRadiansPerSecond * velocityScale * dtSeconds`다.
     * target을 지나치지 않도록 delta를 [-maxStep,+maxStep]로 clamp하고,
     * 실제 velocity는 `step / dtSeconds` [rad/s]로 기록한다.
     */
    void Update(double dtSeconds) override;

    /** @return 이 Controller가 참조 중인 정적 RobotSpecification. */
    [[nodiscard]] const models::RobotSpecification& GetSpecification() const noexcept;

private:
    /** @brief non-owning robot model 규격 pointer. 생성자에서 유효성 검증 후 설정한다. */
    const models::RobotSpecification* specification_ = nullptr;

    /** @brief 상위 계층에 반환할 현재 simulated feedback state. */
    RobotState state_{};

    /** @brief MoveJoint가 설정한 J1..Jn 절대 목표각 [rad]. */
    JointVector targetPositionRadians_;

    /** @brief 모델 maxVelocity에 곱하는 현재 command 비율, 유효 범위 (0,1]. */
    double velocityScale_ = 1.0;

    /**
     * @brief 향후 maxAcceleration에 곱할 현재 command 비율.
     * @note 현재 specification에 acceleration limit이 없어 저장만 하고 Update에는 아직 적용하지 않는다.
     */
    double accelerationScale_ = 1.0;

    /** @brief Connect/Disconnect로 관리되는 simulation backend 초기화 상태. */
    bool connected_ = false;
};

} // namespace grasplink::robotics::backends::simulation
