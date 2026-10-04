#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

/**
 * @file ControlTypes.h
 * @brief Hardware/Simulation backend가 공유하는 robotics 제어 데이터 타입.
 *
 * @details
 * 이 파일은 Viewer/Flecs/GLM/제조사 통신 protocol에 의존하지 않는 제어 계층의 공통 계약이다.
 * 단위는 다음 규칙으로 고정한다.
 *
 * - 관절 각도: radian [rad]
 * - 관절 각속도: radian/second [rad/s]
 * - Cartesian 위치/거리: meter [m]
 * - 선속도: meter/second [m/s]
 * - 각속도: radian/second [rad/s]
 * - 시간: second [s]
 * - Quaternion 배열 순서: [x, y, z, w]
 *
 * 실제 HCR-12A controller가 degree/mm를 사용하거나 다른 robot이 다른 단위를 사용하더라도
 * Hardware backend 경계에서 위 SI 단위로 변환한 뒤 상위 계층에 노출해야 한다.
 */
namespace grasplink::robotics
{

/**
 * @brief Robot의 관절별 scalar 값을 J1..Jn 순서로 저장하는 가변 길이 배열.
 *
 * @details
 * 원소 개수와 순서는 사용 중인 models::RobotSpecification::joints와 반드시 일치한다.
 * 6축에 고정하지 않았기 때문에 이후 4축/7축 robot도 같은 controller contract를 재사용할 수 있다.
 */
using JointVector = std::vector<double>;

/** @brief Controller API 호출이 실패한 이유를 backend-neutral하게 분류한다. */
enum class ErrorCode
{
    /** 요청이 정상 처리되었다. */
    None,
    /** Hardware transport 또는 Simulation backend가 아직 연결/초기화되지 않았다. */
    NotConnected,
    /** 관절 개수, 값 범위, NaN 등 command 자체가 유효하지 않다. */
    InvalidCommand,
    /** 현재 동작/상태 때문에 새 command를 즉시 수락할 수 없다. */
    Busy,
    /** Controller/backend가 fault 상태다. 세부 원인은 state의 faultCode 또는 backend log에서 확인한다. */
    Fault,
    /** 현재 backend에서 해당 기능을 구현하지 않았거나 지원하지 않는다. */
    Unsupported,
    /** Socket/Serial/Modbus 등 transport 계층에서 통신 오류가 발생했다. */
    TransportError
};

/**
 * @brief Controller 명령의 성공/실패와 사람이 읽을 수 있는 진단 문자열을 함께 반환하는 값 타입.
 */
struct Result
{
    /** @brief 성공이면 ErrorCode::None, 실패면 원인을 나타내는 공통 오류 코드. */
    ErrorCode code = ErrorCode::None;

    /** @brief 실패 원인/진단 정보. 성공 시 비어 있을 수 있다. */
    std::string message;

    /** @return code가 ErrorCode::None이면 true. */
    [[nodiscard]] bool Ok() const noexcept { return code == ErrorCode::None; }

    /** @brief `if (result)` 형태로 성공 여부를 검사하기 위한 명시적 bool 변환. */
    explicit operator bool() const noexcept { return Ok(); }

    /** @return 성공 상태의 Result. */
    static Result Success() { return {}; }
};

/**
 * @brief Robot TCP(Tool Center Point)의 Cartesian pose.
 *
 * @details
 * positionMeters는 공통 robotics world/base frame에서의 위치 [m]이고,
 * orientationXyzw는 같은 frame에서의 단위 quaternion [x,y,z,w]다.
 * 현재 SimRobotController는 FK가 아직 연결되지 않아 tcpPoseValid=false로 유지할 수 있다.
 */
struct CartesianPose
{
    /** @brief 위치 [x,y,z], 단위 meter [m]. */
    std::array<double, 3> positionMeters{};

    /** @brief 방향 quaternion [x,y,z,w]. 기본값은 identity rotation. */
    std::array<double, 4> orientationXyzw{0.0, 0.0, 0.0, 1.0};
};

/**
 * @brief Joint-space 절대 위치 이동 요청.
 *
 * @details
 * targetPositionRadians는 각 관절의 절대 목표각 [rad]이며 RobotSpecification 순서를 따른다.
 * velocityScale/accelerationScale은 모델의 물리적 제한 자체를 바꾸는 값이 아니라
 * 해당 제한에 곱하는 0~1 비율이다. 현재 SimRobotController는 velocityScale만 실제 추종에 사용하고,
 * acceleration limit 값이 specification에 확정되기 전까지 accelerationScale은 계약만 보존한다.
 */
struct JointMoveCommand
{
    /** @brief J1..Jn 절대 목표각 [rad]. */
    JointVector targetPositionRadians;

    /** @brief 각 관절 maxVelocity에 곱할 비율. 현재 구현의 유효 범위는 (0, 1]. */
    double velocityScale = 1.0;

    /** @brief 향후 maxAcceleration에 곱할 비율. 현재 구현의 유효 범위는 (0, 1]. */
    double accelerationScale = 1.0;
};

/**
 * @brief TCP를 Cartesian 직선 경로로 이동시키기 위한 고수준 요청.
 *
 * @note 현재 SimRobotController는 FK/IK/trajectory 계층이 연결되지 않아 Unsupported를 반환한다.
 */
struct LinearMoveCommand
{
    /** @brief 목표 TCP pose. 위치 [m], 방향 quaternion [x,y,z,w]. */
    CartesianPose targetPose{};

    /** @brief 허용 최대 TCP 선속도 [m/s]. */
    double maxLinearVelocityMetersPerSecond = 0.25;

    /** @brief 허용 최대 TCP 각속도 [rad/s]. */
    double maxAngularVelocityRadiansPerSecond = 0.5;
};

/** @brief 공통 Robot Controller의 실행 상태. */
enum class RobotMode
{
    /** backend 연결/초기화 전 또는 Disconnect 이후. */
    Disconnected,
    /** 연결되어 있고 현재 목표를 추종하지 않는 대기 상태. */
    Idle,
    /** Joint/Cartesian motion을 수행 중인 상태. */
    Moving,
    /** Stop() 요청으로 소프트웨어 motion이 정지된 상태. */
    Stopped,
    /** backend fault가 발생해 정상 motion command를 수행할 수 없는 상태. */
    Fault
};

/**
 * @brief Hardware/Simulation backend가 상위 계층에 노출하는 Robot 상태 snapshot.
 *
 * @details
 * jointPositionRadians/jointVelocityRadiansPerSecond의 개수와 순서는 현재 RobotSpecification과 같다.
 * `valid=false`이면 나머지 필드는 최신 상태라고 가정하면 안 된다.
 */
struct RobotState
{
    /** @brief 현재 관절 절대 위치 J1..Jn [rad]. */
    JointVector jointPositionRadians;

    /** @brief 현재 관절 속도 J1..Jn [rad/s]. */
    JointVector jointVelocityRadiansPerSecond;

    /** @brief FK 또는 Hardware feedback으로 계산/수신한 TCP pose. */
    CartesianPose tcpPose{};

    /** @brief 현재 controller 실행 상태. */
    RobotMode mode = RobotMode::Disconnected;

    /** @brief backend-native fault code. 0의 구체적 의미는 backend 규약을 따른다. */
    std::uint32_t faultCode = 0;

    /** @brief true일 때만 tcpPose가 유효한 최신 값이다. */
    bool tcpPoseValid = false;

    /** @brief 전체 snapshot이 사용 가능한지 나타낸다. Disconnect/fault 초기화 과정에서는 false일 수 있다. */
    bool valid = false;
};

/**
 * @brief Gripper position/speed/force의 공통 8-bit 요청값.
 *
 * @details
 * 현재 Robotiq 2F-85 backend에서는 다음과 같이 1:1 대응한다.
 * - positionRequest -> rPR: 0=fully open, 255=fully closed
 * - speedRequest    -> rSP: 0=min, 255=max
 * - forceRequest    -> rFR: 0=min, 255=max
 *
 * positionRequest는 finger 폭(mm)이나 motor angle(rad)을 직접 뜻하지 않는다.
 * Simulation에서는 model-specific mapping을 통해 master linkage angle q로 변환해야 한다.
 */
struct GripperCommand
{
    /** @brief 정규화된 위치 요청 0..255. 2F-85에서는 0=open, 255=closed. */
    std::uint8_t positionRequest = 0;

    /** @brief 정규화된 이동 속도 요청 0..255. */
    std::uint8_t speedRequest = 255;

    /** @brief 정규화된 파지 힘 요청 0..255. 실제 힘 단위(N)가 아니다. */
    std::uint8_t forceRequest = 128;
};

/**
 * @brief Gripper의 물체 접촉/목표 도달 상태.
 *
 * @details 현재 값은 Robotiq 2F-85의 gOBJ 의미와 맞춰 두었다.
 */
enum class GripperObjectStatus : std::uint8_t
{
    /** Finger가 목표 위치를 향해 이동 중이며 접촉으로 정지하지 않았다. */
    Moving = 0,
    /** Opening 방향 이동 중 물체 접촉으로 정지했다. */
    ContactWhileOpening = 1,
    /** Closing 방향 이동 중 물체 접촉으로 정지했다. */
    ContactWhileClosing = 2,
    /** 요청 위치에 도달했다. */
    AtRequestedPosition = 3
};

/**
 * @brief Hardware/Simulation gripper backend의 상태 snapshot.
 *
 * @details
 * 현재 필드들은 Robotiq 2F-85 상태 register를 손실 없이 표현할 수 있도록 구성되어 있다.
 * 다른 gripper backend는 가능한 경우 동일한 공통 의미로 변환하고, 제조사 고유 세부 정보는 backend 내부에 보존한다.
 */
struct GripperState
{
    /** @brief 장치가 activation 완료 상태인지 여부. 2F-85의 gACT/gSTA를 해석해 채울 수 있다. */
    bool activated = false;

    /** @brief 현재 Go-To 위치 명령이 활성 상태인지 여부. 2F-85의 gGTO 의미에 대응한다. */
    bool goToActive = false;

    /** @brief 제조사 activation/state raw code. 2F-85 backend에서는 gSTA를 보존한다. */
    std::uint8_t activationStatus = 0;

    /** @brief 이동/접촉/목표 도달 상태. */
    GripperObjectStatus objectStatus = GripperObjectStatus::AtRequestedPosition;

    /** @brief backend-native fault raw code. 2F-85에서는 gFLT에 대응한다. */
    std::uint8_t faultCode = 0;

    /** @brief 마지막으로 장치가 수락/반영한 위치 요청 echo. 2F-85에서는 gPR, 범위 0..255. */
    std::uint8_t requestedPositionEcho = 0;

    /** @brief 현재 실제 gripper position raw 값. 2F-85에서는 gPO, 범위 0..255. */
    std::uint8_t actualPosition = 0;

    /** @brief motor current raw 값. 2F-85에서는 gCU이며 물리 전류[A]로 변환 전의 register 값이다. */
    std::uint8_t currentRaw = 0;

    /** @brief true일 때만 이 snapshot을 최신 backend 상태로 사용한다. */
    bool valid = false;
};

} // namespace grasplink::robotics
