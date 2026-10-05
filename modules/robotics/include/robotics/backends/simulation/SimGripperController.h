#pragma once

#include "robotics/core/IGripperController.h"
#include "robotics/models/GripperSpecification.h"

namespace grasplink::robotics::backends::simulation
{

/**
 * @brief 자유공간 개폐 위치를 고정 시간 간격으로 진행하는 Simulation Controller.
 * @details
 * GripperCommand raw 위치는 specification의 양 끝 범위를 열린 0과 닫힌 1로 선형 대응한다.
 * 내부 계산은 연속 closureFraction을 사용하고, GripperState.actualPosition만 raw 정수로 반올림한다.
 * raw speed는 SimGripperMotionSettings의 master 각속도 [rad/s] 범위에 대응하며 제조사 속도 사양이 아니다.
 * 위치·속도·힘 요청 범위 검사는 수행하지만 힘, 전류, 접촉, 물체 검출이나 파지 적응은 계산하지 않는다.
 * GripperSpecification은 빌려 쓰므로 배열과 문자열을 포함한 사양 저장소가 이 객체보다 오래 살아야 한다.
 */
struct SimGripperMotionSettings
{
    /// 시뮬레이션에서 raw speed 최솟값에 대응하는 master 각속도 [rad/s].
    double minimumMasterVelocityRadiansPerSecond = 0.1;

    /// 시뮬레이션에서 raw speed 최댓값에 대응하는 master 각속도 [rad/s].
    double maximumMasterVelocityRadiansPerSecond = 1.0;
};

/**
 * @brief 자유공간 linkage 위치를 시간에 따라 갱신하는 Gripper backend.
 * @details GetState는 값 복사본을 돌려준다. GUI나 기구학 계층이 연속 위치를 사용할 때는
 * valid와 closureFractionValid를 모두 확인해야 한다. 목표 도달은 자유공간 목표에 도달했다는 뜻이며 접촉이나 grasp를 뜻하지 않는다.
 */
class SimGripperController final : public IGripperController
{
public:
    /**
     * @brief 사양을 빌리고 시뮬레이션 속도 설정을 복사해 Controller를 만든다.
     * @param specification raw 범위, nominal closed master 각도와 joint 배열을 제공하는 모델 사양.
     * @param settings raw speed를 대응시킬 양의 master 각속도 범위 [rad/s].
     * @throws std::invalid_argument 사양 범위가 비었거나 분모가 0이고, nominal 각도 또는 설정이 유효하지 않은 경우.
     * @details position/speed raw 범위는 min < max, force 범위는 min <= max여야 한다.
     * nominalMasterClosedRadians와 설정 각속도는 유한한 양수이며 설정 최솟값은 최댓값보다 클 수 없다.
     */
    explicit SimGripperController(
        const models::GripperSpecification& specification,
        SimGripperMotionSettings settings = {});

    /** @brief 연결 전이면 열린 위치의 유효한 Inactive snapshot을 만들고, 연결 상태면 현재 상태를 보존한다. */
    Result Connect() override;

    /** @brief 연결을 끊고 snapshot을 무효화한다. 다음 Connect는 열린 위치에서 새 상태를 시작한다. */
    void Disconnect() noexcept override;

    /** @brief 연결되어 유효한 시뮬레이션 상태인지 반환한다. */
    [[nodiscard]] bool IsConnected() const noexcept override;

    /**
     * @brief 연결된 상태를 즉시 활성화하며, 이미 활성화된 경우 위치와 동작을 보존한다.
     * @details Reset 뒤에는 mode가 Inactive이므로 Activate만으로 이전 목표를 재개하지 않는다. 새 Command가 필요하다.
     */
    Result Activate() override;

    /**
     * @brief 현재 위치를 보존하고 이동을 중단한 뒤 비활성 상태로 전환한다.
     * @details 마지막 요청 echo와 미도달 분류는 보존한다. 다시 활성화해도 이전 요청은 자동 재개되지 않는다.
     */
    Result Reset() override;

    /**
     * @brief 활성화된 상태에서 raw 위치·속도·힘 범위가 유효한 요청을 수락한다.
     * @param command specification 범위 내 raw 요청. 물리 거리·속도·힘 단위가 아니다.
     * @return 연결/활성 상태 오류 또는 범위 오류, 아니면 목표 수락 결과.
     * @details 잘못된 요청은 기존 상태와 목표를 바꾸지 않는다. 유효한 새 요청은 현재 목표를 교체한다.
     */
    Result Command(const GripperCommand& command) override;

    /**
     * @brief 현재 위치를 보존하고 software 이동을 멈춘다.
     * @details 마지막 요청 echo를 유지하고, 미도달 위치는 objectStatus=Moving으로 분류한다.
     * 실제 E-Stop이나 접촉 정지는 아니다.
     */
    Result Stop() override;

    /** @brief 독립된 GripperState 복사본을 반환한다. 연결/closure 유효성 flag를 함께 확인한다. */
    [[nodiscard]] GripperState GetState() const override;

    /**
     * @brief 유효한 양의 시간 동안 연속 개폐 위치를 목표 방향으로 진행한다.
     * @param dtSeconds 경과 시간 [s]. 유한한 양수일 때만 반영한다.
     * @details 목표에 가까워지면 정확히 목표에서 멈추고 raw 위치는 표시용으로만 반올림한다.
     */
    void Update(double dtSeconds) override;

private:
    bool IsAtTarget() const noexcept;
    void UpdateRawPosition() noexcept;

    // specification의 배열/string_view 소유권은 호출자에게 있다.
    const models::GripperSpecification* specification_ = nullptr;

    // 모델 사양과 분리된 프로젝트 Simulation 가정값.
    SimGripperMotionSettings settings_{};
    GripperState state_{};
    bool connected_ = false;

    // raw 8-bit position으로 되돌리지 않고 계산하는 무차원 연속 위치와 목표.
    double closureFraction_ = 0.0;
    double targetClosureFraction_ = 0.0;

    // 현재 명령 speed가 정하는 자유공간 master 각속도 [rad/s].
    double masterVelocityRadiansPerSecond_ = 0.0;
};

} // namespace grasplink::robotics::backends::simulation
