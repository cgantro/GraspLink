#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics
{

/**
 * @brief 실제 장치와 Simulation에 공통 관절·공구 목표를 요청하는 인터페이스다.
 * @details RobotSpecification은 관절 이름, 순서, 허용각을 제공한다.
 * TCP는 공구 끝에서 위치와 방향을 지정하는 작업 기준점이다.
 * Joint 명령은 각 관절각을 직접 요청하고 Cartesian 명령은 TCP 위치와 방향을 요청한다.
 * TCP 목표를 관절 움직임으로 바꾸려면 역기구학(IK)이 필요하다.
 * IK는 원하는 공구 위치를 만드는 관절각을 찾는 계산이며 정방향 기구학(FK)의 반대다.
 * 기능 지원, 연결, 오류와 새 명령 처리 방식은 구현마다 다르다.
 * 성공은 요청을 받았다는 뜻일 수 있으므로 목표 도달은 GetState로 확인한다.
 * GetState는 내부 저장소와 분리된 시점별 값 복사본을 반환한다.
 * Stop은 소프트웨어 명령이며 장비 보호 정지나 비상 정지를 대신하지 않는다.
 */
class IRobotController
{
public:
    virtual ~IRobotController() = default;

    /**
     * @brief 구현 연결 또는 초기화를 요청한다.
     * @return 성공 또는 구체적인 실패 분류와 진단. 성공만으로 물리 장비의 안전 상태를 보장하지 않는다.
     */
    virtual Result Connect() = 0;

    /** @brief 구현이 사용한 자원을 정리하고 장치 또는 Simulation 연결을 해제한다. */
    virtual void Disconnect() noexcept = 0;

    /** @brief 새 제어 요청을 처리할 수 있는 연결 상태인지 알려준다. */
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief 각 관절을 지정한 절대 목표각으로 움직이도록 요청한다.
     * @param command J1..Jn 목표각 [rad]과 구현별 속도·가속도 비율.
     * @return 요청을 받아들였는지 나타낸다. 성공은 목표 도달이나 동작 완료가 아니다.
     * @details 관절 개수·범위와 Busy 처리, 실행 중 명령 교체 여부는 구현 계약에 따른다.
     */
    virtual Result MoveJoint(const JointMoveCommand& command) = 0;

    /**
     * @brief TCP가 목표 위치와 방향에 이르도록 직선 경로 이동을 요청한다.
     * @param command TCP 위치 [m], quaternion [x,y,z,w], 선속도 [m/s], 각속도 [rad/s], 선가속도 [m/s²], 각가속도 [rad/s²] 상한이다.
     * @return 요청 수락 여부와 실패 분류다. 기능이 없으면 Unsupported를 반환할 수 있다.
     * @details TCP는 공구 끝의 작업 기준점이다.
     * TCP 목표를 만드는 관절각을 찾는 역기구학(IK)과 경로 생성 기능이 있어야 실행할 수 있다.
     * FK는 관절각에서 위치를 계산하므로 FK만 구현한 Controller는 이 명령을 실행할 수 없다.
     * 목표 좌표계는 공통 자료형에서 정하지 않는다.
     */
    virtual Result MoveLinear(const LinearMoveCommand& command) = 0;

    /**
     * @brief 여러 TCP 자세를 한 경로로 실행하도록 요청한다.
     * @details 구현은 이 기능을 지원하지 않으면 Unsupported를 반환한다. Simulation처럼 지원하는 구현은 전체 경로에 하나의 속도 프로파일을 적용할 수 있다.
     */
    virtual Result MoveLinearPath(const LinearPathMoveCommand& command)
    {
        if (command.targetPoses.empty())
            return {ErrorCode::InvalidCommand, "IRobotController: linear path requires at least one target pose"};
        return {ErrorCode::Unsupported, "IRobotController: continuous multi-waypoint paths are not supported"};
    }

    /**
     * @brief 구현 안에서 실행 중인 동작을 소프트웨어 방식으로 정지하도록 요청한다.
     * @return 요청 처리 결과. 성공은 구현 수준에서 요청을 처리했다는 뜻이다.
     * @details 물리 E-Stop이나 protective stop 회로의 동작을 보장하거나 대체하지 않는다.
     */
    virtual Result Stop() = 0;

    /**
     * @brief Controller가 보고한 상태의 독립된 복사본을 반환한다.
     * @return 관절 위치 [rad], 속도 [rad/s], 동작 상태, 오류와 TCP 위치 [m]를 담는다.
     * @details valid는 관절을 포함한 전체 상태의 유효성을 나타낸다.
     * tcpPoseValid는 해당 구현의 TCP 위치·방향 feedback만 유효한지 나타낸다. Simulation은 관절 FK와 설정된 공구 offset으로 계산할 수 있다.
     * 오류 code의 뜻은 장치나 Simulation 구현에 따라 다르다.
     * 실제 TCP feedback과 모델 ToolFrame은 같은 위치라고 가정할 수 없다.
     */
    [[nodiscard]] virtual RobotState GetState() const = 0;

    /**
     * @brief Simulation 상태를 경과 시간만큼 진행하거나 실제 장치의 최신 상태를 갱신한다.
     * @param dtSeconds 경과 시간 [s]다. 유효 범위와 처리 방식은 구현이 정한다.
     * @details 실제 장치는 자체 제어 주기로 움직일 수 있다.
     * 따라서 이 호출은 고정된 관절 이동량이나 동작 완료를 보장하지 않는다.
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
