#pragma once

#include <array>
#include <cstdint>
#include <string>

/**
 * @file ControlTypes.h
 * @brief 실제 HCR-12A와 시뮬레이터가 공유하는 Controller 계층의 공통 데이터 타입.
 *
 * @details
 * 이 계층은 Viewer/Flecs/GLM/통신 프로토콜에 의존하지 않는다.
 * 모든 각도는 radian, 위치는 meter, 시간은 second를 사용한다.
 */

namespace control
{

constexpr std::size_t kRobotJointCount = 6;
using JointArray = std::array<double, kRobotJointCount>;

/** @brief Controller 호출 결과를 분류하는 공통 오류 코드. */
enum class ErrorCode
{
    None,
    NotConnected,
    InvalidCommand,
    Busy,
    Fault,
    Unsupported,
    TransportError
};

/** @brief Hardware/Simulation 구현이 같은 방식으로 결과를 반환하기 위한 값 타입. */
struct Result
{
    ErrorCode code = ErrorCode::None;
    std::string message;

    [[nodiscard]] bool Ok() const noexcept { return code == ErrorCode::None; }
    explicit operator bool() const noexcept { return Ok(); }

    static Result Success() { return {}; }
};

/** @brief TCP pose. position은 meter, quaternion은 xyzw 순서다. */
struct CartesianPose
{
    std::array<double, 3> positionMeters{};
    std::array<double, 4> orientationXyzw{0.0, 0.0, 0.0, 1.0};
};

/** @brief 관절 공간 절대 목표 명령. */
struct JointMoveCommand
{
    JointArray targetPositionRadians{};
    double velocityScale = 1.0;
    double accelerationScale = 1.0;
};

/** @brief Cartesian 직선 이동 목표 명령. */
struct LinearMoveCommand
{
    CartesianPose targetPose{};
    double maxLinearVelocityMetersPerSecond = 0.25;
    double maxAngularVelocityRadiansPerSecond = 0.5;
};

/** @brief 공통 Robot Controller 상태. 제조사별 세부 fault는 backend 내부에서 보존한다. */
enum class RobotMode
{
    Disconnected,
    Idle,
    Moving,
    Stopped,
    Fault
};

struct RobotState
{
    JointArray jointPositionRadians{};
    JointArray jointVelocityRadiansPerSecond{};
    CartesianPose tcpPose{};
    RobotMode mode = RobotMode::Disconnected;
    std::uint32_t faultCode = 0;
    bool valid = false;
};

/**
 * @brief Robotiq 2F-85의 한 번의 Go-To 명령에 대응하는 요청 값.
 *
 * positionRequest: rPR, 0=open, 255=closed
 * speedRequest: rSP, 0=min, 255=max
 * forceRequest: rFR, 0=min, 255=max
 */
struct GripperCommand
{
    std::uint8_t positionRequest = 0;
    std::uint8_t speedRequest = 255;
    std::uint8_t forceRequest = 128;
};

/** @brief Robotiq gOBJ 값과 1:1 대응한다. */
enum class GripperObjectStatus : std::uint8_t
{
    Moving = 0,
    ContactWhileOpening = 1,
    ContactWhileClosing = 2,
    AtRequestedPosition = 3
};

/**
 * @brief Hardware와 Simulation이 동일하게 노출할 2F-85 상태.
 *
 * actualPosition은 gPO(0~255), currentRaw는 gCU(약 10 mA/count),
 * requestedPositionEcho는 gPR이다.
 */
struct GripperState
{
    bool activated = false;
    bool goToActive = false;
    std::uint8_t activationStatus = 0;
    GripperObjectStatus objectStatus = GripperObjectStatus::AtRequestedPosition;
    std::uint8_t faultCode = 0;
    std::uint8_t requestedPositionEcho = 0;
    std::uint8_t actualPosition = 0;
    std::uint8_t currentRaw = 0;
    bool valid = false;
};

} // namespace control
