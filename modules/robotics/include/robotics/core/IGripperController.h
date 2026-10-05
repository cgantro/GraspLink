#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics
{

/**
 * @brief Hardware와 Simulation backend가 공유하는 Gripper 제어 계약.
 * @details
 * protocol register와 모델 joint의 연결은 backend 내부 책임이며 이 순수 domain 인터페이스는
 * Flecs나 GLM을 직접 다루지 않는다. 공통 계약은 요청/상태 형식만 제공한다. raw 요청의 물리량
 * 매핑, adaptive/mimic 동작, contact 판정은 backend별로 정의되며 여기서 장치 동작을 가정하지 않는다.
 * 성공 결과는 요청 수락일 수 있고 실제 완료는 GetState()로 확인한다. Stop은 software 요청이다.
 */
class IGripperController
{
public:
    virtual ~IGripperController() = default;

    /**
     * @brief backend 연결 또는 초기화를 요청한다.
     * @return 성공 또는 실패 분류와 진단.
     */
    virtual Result Connect() = 0;

    /** @brief backend 자원을 정리하고 연결을 해제한다. */
    virtual void Disconnect() noexcept = 0;

    /** @brief backend가 요청을 처리할 연결 상태인지 반환한다. */
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief backend가 제공하는 Gripper 활성화를 요청한다.
     * @return 요청 처리 결과. 기능이 없으면 Unsupported일 수 있다.
     * @details 구체적인 절차는 backend와 장치 계약에 따른다. 특정 protocol 절차는 이 인터페이스가
     * 보장하지 않는다.
     */
    virtual Result Activate() = 0;

    /**
     * @brief backend의 Gripper 상태 초기화 또는 reset을 요청한다.
     * @return 요청 처리 결과. 초기화 범위는 backend별로 다르다.
     */
    virtual Result Reset() = 0;

    /**
     * @brief raw 위치·속도·힘 요청을 전달한다.
     * @param command 각 0..255 raw code. 위치, 속도, 힘의 SI 단위값이 아니며 공통 환산식은 없다.
     * @return 요청 수락 여부와 실패 분류. 성공은 요청 위치 도달이나 grasp 성공을 뜻하지 않는다.
     * @details raw-to-physical 매핑, adaptive/mimic 동작과 허용 조건은 backend 책임이다.
     */
    virtual Result Command(const GripperCommand& command) = 0;

    /**
     * @brief Gripper 동작의 software 정지를 요청한다.
     * @return 요청 처리 결과.
     * @details 실제 E-Stop 또는 protective stop 동작을 보장하거나 대체하지 않는다.
     */
    virtual Result Stop() = 0;

    /**
     * @brief 현재 Gripper feedback snapshot을 값으로 반환한다.
     * @return activation, 상태 분류, fault, raw 위치와 전류 복사본.
     * @details valid가 false이면 현재 유효 feedback으로 간주할 수 없다. raw 값은 mm, rad, N 등으로
     * 변환되지 않으며 fault/contact code의 해석은 backend 계약에 따른다.
     */
    [[nodiscard]] virtual GripperState GetState() const = 0;

    /**
     * @brief backend 상태를 경과 시간만큼 진행하거나 갱신한다.
     * @param dtSeconds 경과 시간 [s]. 유효 범위와 처리 방식은 backend별로 다르다.
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
