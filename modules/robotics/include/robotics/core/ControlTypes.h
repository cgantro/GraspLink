#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief 로봇 제어 인터페이스에서 주고받는 값과 결과 형식.
 * @details 관절 위치 [rad], 관절 속도 [rad/s], TCP 위치 [m], 선속도 [m/s], 시간 [s]를 사용한다.
 * 방향 quaternion 순서는 [x,y,z,w]다. TCP feedback은 Controller 상태이며 FK의 ToolFrame 출력과
 * 별개다. 이 형식은 단위와 데이터 모양을 정할 뿐 하드웨어 연결이나 물리량 변환을 구현하지 않는다.
 */
namespace grasplink::robotics
{

/** @brief J1..Jn 순서로 저장하는 관절 값 목록. 순서는 RobotSpecification을 따른다. */
using JointVector = std::vector<double>;

/**
 * @brief 제어 요청 또는 연결 작업의 결과 분류.
 * @details None은 성공, NotConnected는 사용할 연결이 없음, InvalidCommand는 입력값 불량,
 * Busy는 backend가 현재 요청을 받을 수 없음, Fault는 backend/장치 fault, Unsupported는 기능 미지원,
 * TransportError는 통신 실패를 뜻한다. Fault code만으로 protective stop 상태를 판정할 수는 없다.
 */
enum class ErrorCode
{
    None,           // 성공.
    NotConnected,   // 아직 연결되지 않았거나 연결이 끊김.
    InvalidCommand, // 개수, 범위, NaN 등 명령 값이 잘못됨.
    Busy,           // 기존 동작을 처리 중이라 명령을 받을 수 없음.
    Fault,          // 장비 또는 Simulation 오류 상태.
    Unsupported,    // 해당 backend에서 지원하지 않는 기능.
    TransportError  // TCP/Serial/Modbus 통신 오류.
};

/**
 * @brief 호출의 성공 여부와 선택적 실패 진단.
 * @details 명령의 성공은 backend가 요청을 받아들였다는 뜻일 수 있으며 목표 도달이나 동작 완료를
 * 보장하지 않는다. 보호 정지나 실제 E-Stop도 이 결과만으로 보장되지 않는다.
 */
struct Result
{
    /// None이면 성공이며 message는 추가 진단용이다.
    ErrorCode code = ErrorCode::None;

    /// 사람이 읽을 수 있는 선택적 진단 문자열.
    std::string message;

    [[nodiscard]] bool Ok() const noexcept { return code == ErrorCode::None; }
    explicit operator bool() const noexcept { return Ok(); }

    static Result Success() { return {}; }
};

/** @brief TCP 위치 [m]와 방향 quaternion [x,y,z,w]. 좌표계와 유효성은 이 값에 포함하지 않는다. */
struct CartesianPose
{
    std::array<double, 3> positionMeters{};

    /// [x,y,z,w]; (0,0,0,1)은 회전 없음.
    std::array<double, 4> orientationXyzw{0.0, 0.0, 0.0, 1.0};
};

/** @brief J1..Jn 절대 목표각 [rad]과 backend가 해석하는 속도·가속도 비율. */
struct JointMoveCommand
{
    JointVector targetPositionRadians;

    /// 적용 범위와 방식은 backend별로 다르다. Simulation은 (0,1] 비율을 속도 상한에 적용한다.
    double velocityScale = 1.0;

    /// 적용 방식은 backend별로 다르다. Simulation은 가속도 제한을 적용하지 않는다.
    double accelerationScale = 1.0;
};

// TCP 직선 이동은 backend의 IK와 trajectory 지원이 필요하다.
/** @brief TCP 목표 pose와 직선 이동의 속도 상한 요청. 실행에는 backend의 IK와 경로 실행 기능이 필요하다. */
struct LinearMoveCommand
{
    /// 목표 위치 [m]와 방향 quaternion [x,y,z,w].
    CartesianPose targetPose{};

    /// 요청 선속도 상한 [m/s].
    double maxLinearVelocityMetersPerSecond = 0.25;

    /// 요청 각속도 상한 [rad/s].
    double maxAngularVelocityRadiansPerSecond = 0.5;
};

/** @brief Controller가 보고하는 논리 동작 상태. */
enum class RobotMode
{
    Disconnected, // backend 연결 또는 초기화 전.
    Idle,         // 연결되어 있고 동작 명령이 없음.
    Moving,       // 목표 자세로 이동 중.
    Stopped,      // Stop 요청으로 software 동작 정지.
    Fault         // 정상 제어 불가.
};

/**
 * @brief 한 시점의 Controller feedback 복사본.
 * @details 관절 위치 [rad], 속도 [rad/s], TCP 위치 [m]를 담는다. valid는 전체 snapshot,
 * tcpPoseValid는 TCP pose의 유효성이다. TCP가 유효하지 않아도 관절 상태는 유효할 수 있다.
 * faultCode 해석과 보호 정지 의미는 backend 또는 장치 계약에 달려 있다.
 */
struct RobotState
{
    /// J1..Jn 현재 관절각 [rad].
    JointVector jointPositionRadians;

    /// J1..Jn 현재 관절 속도 [rad/s].
    JointVector jointVelocityRadiansPerSecond;

    /// Controller가 보고한 TCP pose; 유효성은 tcpPoseValid로 확인한다.
    CartesianPose tcpPose{};

    RobotMode mode = RobotMode::Disconnected;

    // 상세 해석은 backend/제조사 책임이다.
    /// backend 또는 장치가 정의하는 fault code.
    std::uint32_t faultCode = 0;

    /// TCP pose feedback만의 유효성.
    bool tcpPoseValid = false;

    /// 전체 snapshot의 유효성. TCP 유효성과 별개다.
    bool valid = false;
};

/**
 * @brief Gripper backend에 전달하는 raw 요청값.
 * @details 각 필드는 0..255 code이며 SI 단위가 아니다. 공통적인 mm, rad, mm/s 또는 N 변환은
 * 정의되어 있지 않다. 장치별 매핑과 adaptive/mimic 동작은 backend 책임이다.
 */
struct GripperCommand
{
    /// 요청 위치 raw code [0,255]; 물리 거리 단위가 아니다.
    std::uint8_t positionRequest = 0;

    /// 요청 속도 raw code [0,255]; [mm/s]가 아니다.
    std::uint8_t speedRequest = 255;

    /// 요청 힘 raw code [0,255]; [N]이 아니다.
    std::uint8_t forceRequest = 128;
};

/** @brief Gripper backend가 분류한 물체 상태 code. */
enum class GripperObjectStatus : std::uint8_t
{
    Moving = 0,              // 접촉·목표 도달 확인 없음. 실제 이동 여부는 mode와 goToActive로 확인한다.
    ContactWhileOpening = 1, // 열다가 물체에 닿아 정지.
    ContactWhileClosing = 2, // 닫다가 물체에 닿아 정지.
    AtRequestedPosition = 3  // 요청 위치에 도달.
};

/**
 * @brief Gripper backend가 제공하는 논리 동작 상태.
 * @details objectStatus는 접촉·목표 도달 분류이고 mode는 연결·활성화·이동·정지를 구분한다.
 * 중간 위치에서 Stop한 상태를 목표 도달로 오인하지 않도록 Stopped를 별도로 둔다.
 */
enum class GripperMode : std::uint8_t
{
    Disconnected,
    Inactive,
    Idle,
    Moving,
    Stopped,
    Fault
};

/**
 * @brief 한 시점의 Gripper 상태 복사본.
 * @details protocol 상태와 위치·전류 raw 값을 보존한다. actualPosition과 currentRaw는 SI 단위가
 * 아니다. 선택적 closureFraction은 0=열림, 1=닫힘인 무차원 위치이며 raw 값과 별도로 유효성을 확인한다.
 * 연속 위치를 지원하는 backend는 이를 제공해 raw 8-bit 반올림이 관절 움직임으로 전파되는 것을 피한다.
 * valid가 false이면 feedback을 유효한 현재 상태로 간주하지 않는다. 접촉 판정, 보호 정지,
 * adaptive 또는 mimic 동작의 의미는 backend별로 정의된다.
 */
struct GripperState
{
    /// 연결·활성화·동작의 논리 상태. raw objectStatus와 별개다.
    GripperMode mode = GripperMode::Disconnected;

    /// backend 또는 장치가 보고한 activation 상태.
    bool activated = false;

    /// backend 또는 장치가 보고한 go-to 동작 상태.
    bool goToActive = false;

    /// protocol activation status raw code.
    std::uint8_t activationStatus = 0;

    /// 이동/접촉/목표 위치 상태 분류.
    GripperObjectStatus objectStatus = GripperObjectStatus::AtRequestedPosition;

    /// 장치 또는 backend fault raw code.
    std::uint8_t faultCode = 0;

    /// 요청 위치 echo raw code.
    std::uint8_t requestedPositionEcho = 0;

    /// 실제 위치 feedback raw code.
    std::uint8_t actualPosition = 0;

    /// 전류 feedback raw code.
    std::uint8_t currentRaw = 0;

    /// currentRaw가 실제 제공된 전류 feedback인지 여부. Simulation은 전류를 계산하지 않는다.
    bool currentValid = false;

    /// 연속 개폐 위치 [0,1]. 0은 열린 기준, 1은 닫힌 기준이며 물리 거리·힘 단위가 아니다.
    double closureFraction = 0.0;

    /// closureFraction이 유한한 [0,1] 위치로 제공되었는지 여부.
    bool closureFractionValid = false;

    /// 전체 snapshot의 유효성.
    bool valid = false;
};

}
