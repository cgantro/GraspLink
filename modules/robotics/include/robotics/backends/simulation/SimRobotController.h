#pragma once

#include "robotics/core/IRobotController.h"
#include "robotics/models/RobotSpecification.h"

namespace grasplink::robotics::backends::simulation
{

/*
 * [추가 용어 설명]
 * - q: 관절 위치/각도 [rad]. 로보틱스 수식에서 joint position을 보통 q로 쓴다.
 * - dq(q-dot): 관절 각속도 [rad/s].
 * - Target: Controller가 도달하려고 저장한 목표 관절각.
 * - State: 현재 q/dq/mode/fault 등을 모은 상태.
 * - Integration step: dt만큼 시간이 흘렀다고 보고 현재 상태를 조금 전진시키는 계산.
 * - Clamp: 값이 최소/최대 범위를 넘지 않도록 제한하는 것.
 * - Fixed Control Loop: rendering FPS와 별개로 일정한 dt로 제어 계산을 반복하는 구조.
 * - FK: 현재 관절각 q로부터 TCP pose를 계산하는 것.
 * - IK: 목표 TCP pose로부터 필요한 관절각 q를 계산하는 것.
 *
 * 이 클래스는 GLB/Flecs를 직접 움직이지 않는다.
 * 논리 RobotState를 계산하고 Viewer의 RobotTransformAdapter가 그 상태를 화면 transform으로 바꾼다.
 */
class SimRobotController final : public IRobotController
{
public:
    /**
     * @brief Simulation에서 사용할 robot model 규격을 연결한다.
     * @param specification 관절 이름/limit/max velocity를 가진 정적 모델 정의.
     * @note 이 객체가 specification을 소유하지 않으므로 specification은 Controller보다 오래 살아 있어야 한다.
     */
    explicit SimRobotController(const models::RobotSpecification& specification);

    /** @brief q=0, dq=0의 논리적 기준자세로 Simulation state를 초기화한다. */
    Result Connect() override;

    /** @brief Simulation을 연결 해제 상태로 만들고 더 이상 motion을 진행하지 않는다. */
    void Disconnect() noexcept override;

    /** @return Connect 후 Disconnect 전이면 true. */
    [[nodiscard]] bool IsConnected() const noexcept override;

    /**
     * @brief 절대 목표 관절각을 검사한 뒤 target으로 저장한다.
     * @note 관절을 즉시 순간이동시키지 않는다. 실제 q 변화는 이후 Update(dt)에서 진행된다.
     */
    Result MoveJoint(const JointMoveCommand& command) override;

    /** @brief Cartesian 직선 이동 요청. 현재 FK/IK/trajectory가 없어 Unsupported를 반환한다. */
    Result MoveLinear(const LinearMoveCommand& command) override;

    /** @brief 현재 q를 새 target으로 고정하고 dq를 0으로 만들어 이동을 멈춘다. */
    Result Stop() override;

    /** @return 현재 q[rad], dq[rad/s], mode 등을 담은 RobotState 복사본. */
    [[nodiscard]] RobotState GetState() const override;

    /**
     * @brief dtSeconds 동안 시간이 흘렀다고 가정하고 현재 q/dq를 한 step 갱신한다.
     *
     * 한 step 최대 이동각:
     * maxStep[rad] = maxVelocity[rad/s] * velocityScale[-] * dt[s]
     *
     * 예: maxVelocity=2 rad/s, scale=0.5, dt=0.01 s라면 한 step 최대 0.01 rad 이동한다.
     */
    void Update(double dtSeconds) override;

    /** @return 이 Controller가 참조하는 정적 robot model specification. */
    [[nodiscard]] const models::RobotSpecification& GetSpecification() const noexcept;

private:
    // 소유하지 않는 robot model specification 주소.
    const models::RobotSpecification* specification_ = nullptr;

    // 현재 simulated q/dq/mode/fault 등을 저장한다.
    RobotState state_{};

    // MoveJoint가 마지막으로 수락한 J1..Jn 절대 목표각 [rad].
    JointVector targetPositionRadians_;

    // 모델 max velocity에 곱하는 현재 속도 비율. 1.0이면 모델 최대속도.
    double velocityScale_ = 1.0;

    // 향후 model max acceleration에 곱할 비율. 현재는 acceleration limit이 없어 실제 Update에 적용하지 않는다.
    double accelerationScale_ = 1.0;

    // Simulation backend가 Connect되어 사용 가능한 상태인지 나타낸다.
    bool connected_ = false;
};

} // namespace grasplink::robotics::backends::simulation
