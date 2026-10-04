#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics
{

/**
 * @brief Hardware/Simulation gripper backend가 공통으로 구현하는 제어 인터페이스.
 *
 * @details
 * 상위 계층은 Robotiq Modbus register packing, Serial transport, GLB finger joint hierarchy를 직접 다루지 않는다.
 * 현재 공통 command는 0..255 position/speed/force 요청으로 정의되어 있으며 Robotiq 2F-85에서는
 * 각각 rPR/rSP/rFR에 대응한다. 실제 finger linkage angle/폭 변환은 model/backend 내부 책임이다.
 */
class IGripperController
{
public:
    /** @brief Interface를 통한 다형 삭제를 위한 virtual destructor. */
    virtual ~IGripperController() = default;

    /**
     * @brief Gripper backend를 사용 가능한 상태로 연결/초기화한다.
     * @return 성공 여부와 오류 원인을 담은 Result.
     */
    virtual Result Connect() = 0;

    /** @brief Transport/Simulation state를 정리한다. 예외를 외부로 던지지 않는다. */
    virtual void Disconnect() noexcept = 0;

    /** @return backend가 연결/초기화되어 command를 처리할 수 있으면 true. */
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief 장치 activation sequence를 시작/수행한다.
     * @return activation 요청 결과.
     * @note Robotiq 2F-85 hardware backend에서는 rACT sequence에 대응한다.
     */
    virtual Result Activate() = 0;

    /**
     * @brief Gripper controller를 reset 상태로 전환한다.
     * @return reset 요청 결과.
     * @note Hardware backend의 reset은 제조사 protocol 의미를 따르고, Simulation은 내부 상태를 초기화한다.
     */
    virtual Result Reset() = 0;

    /**
     * @brief 목표 position/speed/force를 설정하고 이동을 요청한다.
     * @param command 0..255 범위의 정규화된 위치/속도/힘 요청.
     * @return 명령 유효성/연결 상태/장치 fault를 반영한 Result.
     *
     * @note positionRequest는 mm나 rad가 아니다. model-specific mapping을 거쳐 실제 opening/linkage angle로 변환한다.
     */
    virtual Result Command(const GripperCommand& command) = 0;

    /**
     * @brief 현재 gripper motion을 소프트웨어적으로 정지한다.
     * @return stop 요청 결과.
     */
    virtual Result Stop() = 0;

    /**
     * @brief 현재 gripper 상태 snapshot을 반환한다.
     * @return activation/object/fault/position/current 정보를 포함한 값 복사본.
     */
    [[nodiscard]] virtual GripperState GetState() const = 0;

    /**
     * @brief Backend 내부 상태를 한 제어 주기 갱신한다.
     * @param dtSeconds 이전 update 이후 경과 시간 [s].
     *
     * Hardware는 register polling/timeout, Simulation은 free-space linkage 또는 향후 physics 상태 갱신에 사용한다.
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
