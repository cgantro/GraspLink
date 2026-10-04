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

/*
 * [추가 용어 설명]
 * - Joint(관절): 로봇의 두 링크 사이에서 회전/이동이 일어나는 연결부.
 * - Joint-space: TCP 위치가 아니라 각 관절의 각도 q1, q2 ... 로 로봇 자세를 표현하는 방식.
 * - Cartesian pose: 3차원 위치(x,y,z)와 방향(orientation)을 합친 말단장치 자세.
 * - TCP(Tool Center Point): 로봇 끝단에서 작업 기준으로 사용하는 대표 점.
 * - Quaternion: 3차원 회전을 표현하는 4개 값. 여기서는 [x,y,z,w] 순서다.
 * - Backend: 같은 Controller API 뒤에서 실제 장비 또는 Simulation을 수행하는 구현체.
 * - Snapshot: 특정 시점의 상태를 복사해 전달한 값. 이후 Controller 내부 상태가 바뀌어도 복사본은 그대로다.
 *
 * 이 파일의 공통 단위:
 * - joint angle [rad]
 * - joint velocity [rad/s]
 * - Cartesian position [m]
 * - linear velocity [m/s]
 * - angular velocity [rad/s]
 * - time [s]
 */

/**
 * @brief J1부터 Jn까지 관절별 scalar 값을 순서대로 저장하는 배열.
 * @note 원소 개수와 순서는 사용 중인 RobotSpecification의 joints 배열과 같아야 한다.
 */
using JointVector = std::vector<double>;

enum class ErrorCode
{
    None,           // 성공. 오류가 없음.
    NotConnected,   // backend가 아직 Connect되지 않았거나 연결이 끊어짐.
    InvalidCommand, // 관절 개수/범위/NaN 등 명령 값 자체가 잘못됨.
    Busy,           // 현재 동작 중이라 새 명령을 즉시 받을 수 없음.
    Fault,          // 장비 또는 Simulation이 fault 상태임.
    Unsupported,    // 해당 backend에서 아직 지원하지 않는 기능임.
    TransportError  // TCP/Serial/Modbus 등 통신 계층 오류.
};

struct Result
{
    // 호출 결과의 공통 오류 코드. ErrorCode::None이면 성공이다.
    ErrorCode code = ErrorCode::None;

    // 사람이 읽을 수 있는 추가 진단 문자열. 성공 시 비어 있을 수 있다.
    std::string message;

    [[nodiscard]] bool Ok() const noexcept { return code == ErrorCode::None; }
    explicit operator bool() const noexcept { return Ok(); }

    static Result Success() { return {}; }
};

struct CartesianPose
{
    // TCP 위치 [x,y,z], 단위 meter [m].
    std::array<double, 3> positionMeters{};

    // TCP 방향 Quaternion [x,y,z,w]. 기본값 (0,0,0,1)은 회전이 없는 identity orientation이다.
    std::array<double, 4> orientationXyzw{0.0, 0.0, 0.0, 1.0};
};

struct JointMoveCommand
{
    // J1..Jn의 절대 목표 관절각 [rad]. 현재 각도에서 더할 delta 값이 아니다.
    JointVector targetPositionRadians;

    // 모델의 최대 관절속도에 곱하는 비율. 1.0이면 모델 최대속도, 0.5이면 절반 속도다.
    double velocityScale = 1.0;

    // 모델의 최대 관절가속도에 곱할 비율. 현재 Simulation은 가속도 limit이 없어 계약만 보존한다.
    double accelerationScale = 1.0;
};

struct LinearMoveCommand
{
    // 직선 이동 후 도달하려는 TCP의 위치/방향.
    CartesianPose targetPose{};

    // TCP가 직선 경로를 따라 이동할 때 허용할 최대 선속도 [m/s].
    double maxLinearVelocityMetersPerSecond = 0.25;

    // TCP 방향이 변할 때 허용할 최대 각속도 [rad/s].
    double maxAngularVelocityRadiansPerSecond = 0.5;
};

enum class RobotMode
{
    Disconnected, // backend가 연결/초기화되지 않은 상태.
    Idle,         // 연결되어 있지만 현재 이동 명령을 수행하지 않는 대기 상태.
    Moving,       // 목표 관절각/pose를 향해 이동 중인 상태.
    Stopped,      // Stop() 요청으로 software motion이 멈춘 상태.
    Fault         // 정상 제어를 계속할 수 없는 오류 상태.
};

struct RobotState
{
    // 현재 J1..Jn의 절대 관절각 [rad].
    JointVector jointPositionRadians;

    // 현재 J1..Jn의 관절 각속도 [rad/s].
    JointVector jointVelocityRadiansPerSecond;

    // FK 또는 실제 장비 feedback으로 얻은 현재 TCP pose.
    CartesianPose tcpPose{};

    // Controller의 현재 실행 상태.
    RobotMode mode = RobotMode::Disconnected;

    // backend/제조사 고유 fault code. 세부 해석은 해당 backend가 담당한다.
    std::uint32_t faultCode = 0;

    // true일 때만 tcpPose 값을 유효한 최신 값으로 사용한다.
    bool tcpPoseValid = false;

    // true일 때만 이 RobotState 전체를 현재 backend의 유효한 상태 snapshot으로 사용한다.
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
    // 위치 요청 raw 값. 2F-85에서는 0=fully open, 255=fully closed. mm/rad 자체가 아니다.
    std::uint8_t positionRequest = 0;

    // 이동 속도 요청 raw 값 0..255. 실제 mm/s로 직접 해석하지 않는다.
    std::uint8_t speedRequest = 255;

    // 파지 힘 요청 raw 값 0..255. 실제 Newton[N] 값 자체가 아니다.
    std::uint8_t forceRequest = 128;
};

enum class GripperObjectStatus : std::uint8_t
{
    Moving = 0,             // 접촉 없이 요청 위치를 향해 이동 중.
    ContactWhileOpening = 1,// 여는 방향 이동 중 물체 접촉으로 정지.
    ContactWhileClosing = 2,// 닫는 방향 이동 중 물체 접촉으로 정지.
    AtRequestedPosition = 3 // 요청 위치에 도달.
};

struct GripperState
{
    // 장치 activation이 완료되어 정상 명령을 받을 준비가 되었는지 여부.
    bool activated = false;

    // 현재 위치로 이동하는 Go-To 명령이 활성 상태인지 여부.
    bool goToActive = false;

    // 제조사 protocol의 activation/state raw code. 2F-85에서는 gSTA 계열 의미를 보존한다.
    std::uint8_t activationStatus = 0;

    // 이동 중/접촉/도달 중 어느 상태인지 나타낸다.
    GripperObjectStatus objectStatus = GripperObjectStatus::AtRequestedPosition;

    // backend 또는 제조사 고유 fault raw code.
    std::uint8_t faultCode = 0;

    // 장치가 마지막으로 받아들인 위치 요청값 echo. 2F-85에서는 gPR에 대응한다.
    std::uint8_t requestedPositionEcho = 0;

    // 현재 그리퍼 위치를 나타내는 raw feedback. 2F-85에서는 gPO에 대응한다.
    std::uint8_t actualPosition = 0;

    // 모터 전류 관련 raw feedback. 물리 전류[A]로 변환하기 전 값이다.
    std::uint8_t currentRaw = 0;

    // true일 때만 이 GripperState를 유효한 최신 상태로 사용한다.
    bool valid = false;
};

} // namespace grasplink::robotics
