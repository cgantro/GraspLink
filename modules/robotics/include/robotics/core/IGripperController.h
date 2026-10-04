#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics
{

/**
 * @brief 실제 Gripper와 Simulation Gripper를 같은 방식으로 제어하기 위한 공통 인터페이스.
 *
 * @details
 * 상위 코드는 Robotiq의 Modbus register, Serial 통신, GLB 손가락 계층을 직접 알 필요가 없다.
 * Hardware/Simulation backend가 이 인터페이스를 구현하고, 상위 코드는 공통 함수만 호출한다.
 *
 * 용어:
 * - Gripper: 물체를 잡기 위해 열리고 닫히는 말단장치.
 * - Activation: 장치를 명령 가능한 상태로 초기화하는 과정.
 * - Position request: 얼마나 열고/닫을지 요청하는 값.
 * - Speed request: 얼마나 빠르게 움직일지 요청하는 값.
 * - Force request: 어느 정도 힘으로 파지할지 요청하는 값.
 * - Raw value: mm/N/rad 같은 물리 단위가 아니라 장치 protocol이 사용하는 정수 값.
 * - Backend: 이 인터페이스 뒤에서 실제 장비 통신 또는 시뮬레이션을 수행하는 구현체.
 *
 * 현재 공통 GripperCommand는 0..255 position/speed/force 값을 사용한다.
 * Robotiq 2F-85에서는 각각 rPR/rSP/rFR에 대응한다.
 */
class IGripperController
{
public:
    /** @brief C++ 다형성으로 파생 Controller를 안전하게 삭제하기 위한 virtual destructor. */
    virtual ~IGripperController() = default;

    /**
     * @brief Gripper backend를 사용할 수 있도록 연결/초기화한다.
     * @return 성공 여부와 실패 원인을 담은 Result.
     *
     * Simulation에서는 내부 상태를 준비하고, Hardware에서는 serial/fieldbus 연결을 준비할 수 있다.
     */
    virtual Result Connect() = 0;

    /** @brief 연결/통신/Simulation 상태를 정리하고 명령을 받지 않는 상태로 만든다. */
    virtual void Disconnect() noexcept = 0;

    /** @return 현재 Gripper backend가 연결되어 명령을 처리할 수 있으면 true. */
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief Gripper를 정상 명령 가능한 활성 상태로 만든다.
     * @return activation 요청 결과.
     *
     * @details Robotiq 2F-85 Hardware backend에서는 rACT activation sequence와 대응한다.
     */
    virtual Result Activate() = 0;

    /**
     * @brief Gripper 제어 상태를 초기 상태로 되돌린다.
     * @return reset 요청 결과.
     *
     * Hardware에서는 제조사 protocol의 reset 의미를 따르고,
     * Simulation에서는 내부 command/state를 초기화한다.
     */
    virtual Result Reset() = 0;

    /**
     * @brief 목표 위치/속도/힘 값을 전달하고 gripper motion을 요청한다.
     * @param command 0..255 범위의 raw position/speed/force 값.
     * @return 명령 유효성, 연결 상태, fault 등을 반영한 Result.
     *
     * @note positionRequest는 손가락 간 거리[mm]나 관절각[rad] 자체가 아니다.
     *       Simulation에서는 model-specific mapping을 거쳐 opening/master linkage angle로 변환해야 한다.
     */
    virtual Result Command(const GripperCommand& command) = 0;

    /**
     * @brief 현재 진행 중인 gripper motion을 소프트웨어적으로 정지한다.
     * @return stop 요청 결과.
     */
    virtual Result Stop() = 0;

    /**
     * @brief 현재 gripper 상태를 값 복사본(snapshot)으로 가져온다.
     * @return activation, 접촉/목표도달, fault, position, motor current 정보를 포함한 상태.
     */
    [[nodiscard]] virtual GripperState GetState() const = 0;

    /**
     * @brief 시간이 dtSeconds만큼 진행됐다고 보고 backend 내부 상태를 한 단계 갱신한다.
     * @param dtSeconds 이전 Update 이후 경과 시간 [s].
     *
     * Hardware에서는 register polling/timeout, Simulation에서는 손가락 기구학/physics 상태 갱신에 사용할 수 있다.
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
