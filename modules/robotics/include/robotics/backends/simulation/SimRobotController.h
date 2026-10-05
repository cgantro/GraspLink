#pragma once

#include "robotics/core/IRobotController.h"
#include "robotics/models/RobotSpecification.h"

namespace grasplink::robotics::backends::simulation
{

/**
 * @brief 고정 시간 간격마다 관절 상태를 목표각 쪽으로 진행하는 software controller.
 * @details
 * Controller는 RobotState만 갱신하며 GLB mesh, Flecs Entity, Physics body는 직접 움직이지 않는다.
 * Viewer는 상태를 RobotKinematics에 보내 FK pose를 만들고 adapter가 별도로 화면 계층에 반영한다.
 * 관절 이동은 모델의 최대 각속도와 velocityScale로 제한되며 가속도 제한이나 속도 ramp는 없다.
 * MoveLinear은 IK와 Cartesian trajectory 실행이 없어 Unsupported다. FK의 ToolFrame 결과도 이
 * Controller가 제공하는 TCP feedback이 아니므로 Connect 뒤 valid는 true, tcpPoseValid는 false다.
 * specification과 내부 name/array view는 빌려 쓰며 Controller보다 오래 살아야 한다.
 */
class SimRobotController final : public IRobotController
{
public:
    /**
     * @brief 정적 로봇 사양을 참조해 controller를 만든다.
     * @param specification 관절 제한과 최대 속도를 제공하는 사양. 배열과 문자열 저장소도 수명 동안 유효해야 한다.
     * @throws std::invalid_argument 관절 배열이 비었거나, q=0이 제한 밖이거나, 제한/최대 속도가 유한한 유효값이 아닐 때.
     * @details 시작 상태가 항상 q=0이므로 0을 포함하지 않는 모델은 이 구현의 초기화 계약과 맞지 않는다.
     */
    explicit SimRobotController(const models::RobotSpecification& specification);

    /** @brief q=0, dq=0의 유효한 논리 상태로 Simulation 연결을 초기화한다.
     * @return 항상 성공. TCP pose는 여기서 계산하지 않으므로 tcpPoseValid는 false다.
     * @details target도 현재 q로 설정되고 속도·가속도 비율은 1.0으로 돌아간다.
     */
    Result Connect() override;

    /** @brief 연결을 해제하고 논리 상태를 Disconnected/invalid로 표시하며 속도를 0으로 만든다. */
    void Disconnect() noexcept override;

    /** @brief 현재 software 연결 여부를 반환한다. */
    [[nodiscard]] bool IsConnected() const noexcept override;

    /**
     * @brief J1..Jn 절대 목표각을 받아 다음 Update부터 관절 이동을 시작한다.
     * @param command 관절 수와 순서가 specification과 같은 목표각 [rad], 속도·가속도 비율.
     * @return NotConnected 또는 InvalidCommand(관절 수, 비율, 유한성, 관절 범위 오류); 유효한 요청은 성공.
     * @details 두 비율은 모두 유한한 (0,1]이어야 한다. velocityScale은 각 모델 최대 각속도에 곱해진다.
     * accelerationScale은 계약상 저장되지만 여기서는 사용하지 않는다. Moving 중 새 명령은 기존 목표를 교체하며,
     * 성공은 목표 수락이지 목표 도달이 아니다. 완료 여부는 GetState의 mode와 관절값으로 확인한다.
     */
    Result MoveJoint(const JointMoveCommand& command) override;

    /**
     * @brief Cartesian 직선 이동을 요청한다.
     * @param command 공통 API의 TCP 목표 pose와 선속도·각속도 상한.
     * @return 미연결이면 NotConnected, 연결되어 있으면 항상 Unsupported.
     * @details RobotKinematics의 FK는 관절각에서 pose를 구하는 계산이다. 목표 pose를 관절각으로 바꾸는 IK와
     * 경로 실행이 이 backend에 연결되지 않았으므로 FK가 있어도 MoveLinear을 수행할 수 없다.
     */
    Result MoveLinear(const LinearMoveCommand& command) override;

    /**
     * @brief 현재 위치에서 software 동작을 정지시킨다.
     * @return 미연결이면 NotConnected, 연결 상태에서는 성공.
     * @details 현재 q를 새 target으로 하고 dq를 0, mode를 Stopped로 설정한다. 이는 이 Simulation의 상태 변경이며
     * protective stop 또는 hardware E-Stop을 뜻하지 않는다. 성공은 software 요청 처리 완료를 뜻한다.
     */
    Result Stop() override;

    /**
     * @brief 현재 논리 상태의 독립된 값 복사본을 반환한다.
     * @return 관절 위치 [rad], 속도 [rad/s], mode, valid 및 TCP 유효성 정보를 복사한 snapshot.
     * @details Connect 직후 valid=true, tcpPoseValid=false다. TCP pose는 FK의 ToolFrame과 별개의 controller feedback이므로
     * 이 구현은 이를 채우지 않는다. Disconnect 뒤에는 valid=false다. 반환된 vector는 controller 내부 저장소를 빌리지 않는다.
     */
    [[nodiscard]] RobotState GetState() const override;

    /**
     * @brief 유효한 양의 경과 시간만큼 각 관절을 목표 쪽으로 진행한다.
     * @param dtSeconds 경과 시간 [s]. 연결, Moving 상태, 유한한 양수일 때만 적용된다.
     * @details 각 관절에서 maxStep = maxVelocityRadiansPerSecond * velocityScale * dtSeconds를 계산하고,
     * 목표까지의 각도 차이를 그 범위로 잘라 q를 갱신한다. dq는 이번 step/dtSeconds다. 가속도 제한이 없어
     * 명령 시작 시 속도가 즉시 바뀔 수 있다. 목표에 도달하면 q를 정확히 맞추고 dq=0, mode=Idle로 바꾼다.
     * 잘못된 dt는 오류 상태를 만들지 않고 해당 호출을 무시한다.
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
