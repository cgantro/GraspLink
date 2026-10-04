#pragma once

#include "robotics/core/IRobotController.h"
#include "robotics/models/RobotSpecification.h"

namespace grasplink::robotics::backends::simulation
{

/**
 * @brief 실제 모터 대신 메모리 속 RobotState를 시간에 따라 움직이는 Simulation용 Controller.
 *
 * @details
 * 이 클래스는 "화면의 GLB를 직접 회전시키는 클래스"가 아니다.
 * 먼저 로봇의 논리 상태(q, dq)를 계산하고, Viewer의 RobotTransformAdapter가 그 상태를 화면에 반영한다.
 *
 * 용어:
 * - q: 관절 위치/각도. 보통 joint position을 수식에서 q로 표기한다. 단위 [rad].
 * - dq 또는 q-dot: 관절 각속도. 단위 [rad/s].
 * - Target: Controller가 도달하려는 목표 관절각.
 * - State: 현재 시뮬레이션이 가지고 있는 관절각/속도/모드 등의 상태.
 * - Integration step: dt만큼 시간이 흘렀다고 보고 현재 상태를 조금 전진시키는 계산.
 * - Clamp: 어떤 값이 허용 범위를 넘지 않도록 최소/최대 사이로 제한하는 것.
 * - Fixed Control Loop: 렌더링 FPS와 관계없이 일정한 dt로 제어 계산을 반복하는 구조.
 * - FK: 현재 q에서 TCP pose를 계산하는 것.
 * - IK: 목표 TCP pose에서 필요한 q를 계산하는 것.
 *
 * 현재 구현은 목표각까지 "최대속도를 넘지 않게 조금씩 이동"하는 가장 단순한 joint-space simulator다.
 * 물리 엔진, 토크, 충돌, 실제 모터 동역학은 아직 포함하지 않는다.
 */
class SimRobotController final : public IRobotController
{
public:
    /**
     * @brief 이 Simulation이 어떤 robot model을 흉내낼지 정적 사양을 연결한다.
     * @param specification 관절 이름/축/limit/max velocity를 가진 RobotSpecification.
     * @throws std::invalid_argument joint 배열이 없거나 jointCount가 0이면 예외.
     *
     * @note specification 자체를 복사/소유하지 않고 주소만 저장하므로 프로그램 수명 동안 유효해야 한다.
     */
    explicit SimRobotController(const models::RobotSpecification& specification);

    /**
     * @brief 시뮬레이션 상태를 q=0, dq=0의 논리적 기준자세로 초기화한다.
     * @return 정상 초기화 시 Result::Success().
     *
     * @note 여기의 q=0은 실제 HCR 장비의 encoder zero/home calibration과 같은 의미가 아니다.
     */
    Result Connect() override;

    /** @brief Simulation을 연결 해제 상태로 만들고 속도 feedback을 0으로 초기화한다. */
    void Disconnect() noexcept override;

    /** @return Connect 후 Disconnect 전이면 true. */
    [[nodiscard]] bool IsConnected() const noexcept override;

    /**
     * @brief J1..Jn의 절대 목표각을 설정한다.
     * @param command 목표각 [rad], velocityScale, accelerationScale.
     * @return 관절 개수/각도 limit/scale/연결 상태가 잘못되면 실패 Result.
     *
     * @details
     * 이 함수는 관절을 즉시 목표각으로 순간이동시키지 않는다.
     * target만 저장하고 실제 q 변화는 이후 Update(dt)에서 max velocity를 지키며 진행한다.
     */
    Result MoveJoint(const JointMoveCommand& command) override;

    /**
     * @brief TCP를 Cartesian 직선으로 이동시키는 명령 진입점.
     * @return 현재 FK/IK/trajectory가 연결되지 않아 ErrorCode::Unsupported.
     */
    Result MoveLinear(const LinearMoveCommand& command) override;

    /**
     * @brief 현재 q를 새 target으로 설정해 이후 이동을 멈춘다.
     * @return 연결되어 있지 않으면 NotConnected, 성공 시 RobotMode::Stopped.
     */
    Result Stop() override;

    /** @return 현재 q[rad], dq[rad/s], mode 등을 담은 RobotState 복사본. */
    [[nodiscard]] RobotState GetState() const override;

    /**
     * @brief dtSeconds 동안 관절이 이동했다고 가정해 현재 q/dq를 한 step 갱신한다.
     * @param dtSeconds 이번 제어 step의 시간 길이 [s]. 양수 finite 값만 사용한다.
     *
     * @details
     * 각 관절의 한 step 최대 이동각은 다음과 같다.
     *
     * `maxStep[rad] = maxVelocity[rad/s] * velocityScale[-] * dt[s]`
     *
     * 예를 들어 최대속도 2 rad/s, scale 0.5, dt 0.01 s라면
     * 한 step에서 최대 0.01 rad만 움직인다.
     */
    void Update(double dtSeconds) override;

    /** @return 현재 Controller가 참조 중인 robot model 사양. */
    [[nodiscard]] const models::RobotSpecification& GetSpecification() const noexcept;

private:
    /** @brief 이 Controller가 흉내내는 robot model 사양. 메모리를 소유하지 않는 pointer. */
    const models::RobotSpecification* specification_ = nullptr;

    /** @brief 현재 simulated q/dq/mode를 보관하고 GetState()로 외부에 전달하는 상태. */
    RobotState state_{};

    /** @brief MoveJoint()가 마지막으로 수락한 J1..Jn 절대 목표각 [rad]. */
    JointVector targetPositionRadians_;

    /** @brief 각 관절 모델 max velocity에 곱하는 현재 속도 비율. 유효 범위 `(0,1]`. */
    double velocityScale_ = 1.0;

    /**
     * @brief 향후 모델 max acceleration에 곱할 비율.
     * @note 현재 RobotSpecification에 검증된 acceleration limit이 없어 값만 보존하고 아직 Update에 적용하지 않는다.
     */
    double accelerationScale_ = 1.0;

    /** @brief Connect/Disconnect로 관리되는 Simulation backend 준비 상태. */
    bool connected_ = false;
};

} // namespace grasplink::robotics::backends::simulation
