#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

/**
 * @file ControlTypes.h
 * @brief Hardware/Simulation backend가 공유하는 robotics 제어 타입.
 *
 * Viewer/Flecs/GLM/제조사 protocol에 의존하지 않는다.
 * 각도는 radian, 위치는 meter, 시간은 second를 사용한다.
 */
namespace grasplink::robotics
{

using JointVector = std::vector<double>;

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

struct Result
{
    ErrorCode code = ErrorCode::None;
    std::string message;

    [[nodiscard]] bool Ok() const noexcept { return code == ErrorCode::None; }
    explicit operator bool() const noexcept { return Ok(); }

    static Result Success() { return {}; }
};

struct CartesianPose
{
    std::array<double, 3> positionMeters{};
    std::array<double, 4> orientationXyzw{0.0, 0.0, 0.0, 1.0};
};

struct JointMoveCommand
{
    JointVector targetPositionRadians;
    double velocityScale = 1.0;
    double accelerationScale = 1.0;
};

struct LinearMoveCommand
{
    CartesianPose targetPose{};
    double maxLinearVelocityMetersPerSecond = 0.25;
    double maxAngularVelocityRadiansPerSecond = 0.5;
};

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
    JointVector jointPositionRadians;
    JointVector jointVelocityRadiansPerSecond;
    CartesianPose tcpPose{};
    RobotMode mode = RobotMode::Disconnected;
    std::uint32_t faultCode = 0;
    bool tcpPoseValid = false;
    bool valid = false;
};

/**
 * @brief normalized gripper position/speed/force request.
 *
 * 현재 2F-85는 이 값을 rPR/rSP/rFR에 1:1 매핑한다.
 * 다른 gripper backend는 같은 0~255 의미로 변환한다.
 */
struct GripperCommand
{
    std::uint8_t positionRequest = 0;
    std::uint8_t speedRequest = 255;
    std::uint8_t forceRequest = 128;
};

enum class GripperObjectStatus : std::uint8_t
{
    Moving = 0,
    ContactWhileOpening = 1,
    ContactWhileClosing = 2,
    AtRequestedPosition = 3
};

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

} // namespace grasplink::robotics
