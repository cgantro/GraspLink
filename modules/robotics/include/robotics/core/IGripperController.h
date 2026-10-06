#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics
{

/**
 * @brief 실제 Gripper와 Simulation에 공통 명령을 보내고 상태를 읽는 인터페이스다.
 * @details 장치 register는 통신으로 읽고 쓰는 값이며 이를 모델 관절에 연결하는 방식은 구현이 정한다.
 * 위치·속도·힘 code는 장치별 0..255 정수값이며 mm, mm/s, N으로 바꾸는 공통 환산식이 없다.
 * 여러 손가락의 회전 관계와 접촉 판정도 구현별로 다르다.
 * 성공은 요청을 받았다는 뜻일 수 있으므로 완료 여부는 GetState로 확인한다.
 * Stop은 소프트웨어 요청이며 장비 보호 정지나 비상 정지를 대신하지 않는다.
 */
class IGripperController
{
public:
    virtual ~IGripperController() = default;

    /**
     * @brief 구현 연결 또는 초기화를 요청한다.
     * @return 성공 또는 실패 분류와 진단.
     */
    virtual Result Connect() = 0;

    /** @brief 구현이 사용한 자원을 정리하고 장치 또는 Simulation 연결을 해제한다. */
    virtual void Disconnect() noexcept = 0;

    /** @brief 구현이 요청을 처리할 연결 상태인지 반환한다. */
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

        /**
     * @brief Gripper 구동부를 사용할 수 있도록 활성화를 요청한다.
     * @return 요청 처리 결과이며 구현에 기능이 없으면 Unsupported일 수 있다.
     * @details 활성화 순서와 대기 시간은 장치마다 다르므로 공통 인터페이스가 통신 절차를 정하지 않는다.
     */
    virtual Result Activate() = 0;

    /**
     * @brief 구현에 Gripper 상태를 초기화하거나 reset하도록 요청한다.
     * @return 요청 처리 결과. 초기화 범위는 구현별로 다르다.
     */
    virtual Result Reset() = 0;

    /**
     * @brief 단위가 정해지지 않은 위치·속도·힘 code를 구현에 전달한다.
     * @param command 각 0..255 원본 code. 위치, 속도, 힘의 SI 단위값이 아니며 공통 환산식은 없다.
     * @return 요청 수락 여부와 실패 분류. 성공은 요청 위치 도달이나 파지 성공을 뜻하지 않는다.
     * @details 원본 code를 물리량으로 바꾸는 방법, 여러 관절의 연동 회전, 유효 범위는 구현이 정한다.
     */
    virtual Result Command(const GripperCommand& command) = 0;

    /**
     * @brief 구현 안에서 Gripper 동작을 소프트웨어 방식으로 정지하도록 요청한다.
     * @return 요청 처리 결과.
     * @details 실제 비상 정지(E-Stop)나 장비 보호 정지를 보장하거나 대신 실행하지 않는다.
     */
    virtual Result Stop() = 0;

        /**
     * @brief 현재 Gripper 상태의 독립된 복사본을 반환한다.
     * @return 활성화, 동작, 오류, 위치 code와 제공되는 경우 연속 위치·전류를 담은 값이다.
     * @details valid가 false이면 현재 feedback으로 사용하지 않는다.
     * 원본 code는 mm, rad, N 같은 물리 단위가 아니며 오류·접촉 code 의미도 구현마다 다르다.
     * 연속 개폐 비율이나 전류를 쓸 때는 각각 closureFractionValid와 currentValid를 확인한다.
     * Simulation에서 목표에 도달했다는 뜻은 자유공간 위치에 왔다는 뜻이며 물체를 잡았다는 뜻은 아니다.
     */
    [[nodiscard]] virtual GripperState GetState() const = 0;

    /**
     * @brief 지정한 경과 시간만큼 Simulation 상태를 진행하거나 실제 장치의 최신 상태를 읽는다.
     * @param dtSeconds 경과 시간 [s]. 유효 범위와 처리 방식은 구현별로 다르다.
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
