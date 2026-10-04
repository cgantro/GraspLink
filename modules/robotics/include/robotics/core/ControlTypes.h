#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

/**
 * @file ControlTypes.h
 * @brief Hardware/Simulation backend가 공통으로 사용하는 로봇 제어 데이터 타입.
 *
 * @details
 * 로보틱스를 처음 보는 사람이 이 파일을 읽을 때 필요한 용어를 먼저 정리한다.
 *
 * - Joint(관절): 로봇 팔이 회전하는 축. HCR-12A에서는 J1~J6가 각각 하나의 Joint다.
 * - Joint angle: 각 관절이 기준자세에서 얼마나 회전했는지 나타내는 각도.
 * - Joint-space: 로봇 자세를 `J1 각도, J2 각도, ...`처럼 관절각 목록으로 표현하는 방식.
 * - Cartesian: 작업공간을 X/Y/Z 위치와 방향으로 표현하는 방식.
 * - Pose: 위치(position)와 방향(orientation)을 합친 값.
 * - TCP(Tool Center Point): 로봇 끝단에서 "공구의 대표점"으로 사용하는 기준점.
 * - Quaternion: 3D 회전을 4개 값[x,y,z,w]으로 표현하는 방법. Euler angle보다 회전 합성에 안정적이다.
 * - FK(Forward Kinematics): 관절각 -> TCP 위치/방향을 계산하는 것.
 * - IK(Inverse Kinematics): 원하는 TCP 위치/방향 -> 필요한 관절각을 계산하는 것.
 * - Backend: 같은 인터페이스 뒤에서 실제 장비 또는 시뮬레이션을 구현하는 코드.
 * - Snapshot: 특정 시점의 상태를 복사한 값. 이후 내부 상태가 바뀌어도 반환된 값 자체는 그대로다.
 * - Fault: 정상 제어를 계속할 수 없는 오류 상태.
 *
 * 공통 단위는 다음으로 고정한다.
 * - 관절 각도: radian [rad]
 * - 관절 각속도: radian/second [rad/s]
 * - Cartesian 위치/거리: meter [m]
 * - 선속도: meter/second [m/s]
 * - 각속도: radian/second [rad/s]
 * - 시간: second [s]
 * - Quaternion 배열 순서: [x, y, z, w]
 *
 * 실제 HCR controller가 degree/mm를 쓰더라도 Hardware backend 경계에서 위 SI 단위로 변환한 뒤
 * 상위 계층에 전달하는 것을 계약으로 한다.
 */
namespace grasplink::robotics
{

/**
 * @brief J1..Jn 순서의 관절값 목록.
 *
 * @details
 * 예를 들어 HCR-12A라면 원소 6개가 `[J1,J2,J3,J4,J5,J6]` 순서로 들어간다.
 * 각 원소의 의미/단위는 사용하는 필드가 결정한다. position이면 [rad], velocity면 [rad/s]다.
 */
using JointVector = std::vector<double>;

/** @brief Controller 명령 실패 원인을 실제 장비/시뮬레이션 공통 의미로 분류한 오류 코드. */
enum class ErrorCode
{
    /** 정상 처리됨. */
    None,
    /** Controller/backend가 아직 연결 또는 초기화되지 않음. */
    NotConnected,
    /** 관절 개수/각도 범위/NaN 등 명령 자체가 잘못됨. */
    InvalidCommand,
    /** 현재 다른 동작 중이라 새 명령을 바로 받을 수 없음. */
    Busy,
    /** 장치 또는 backend가 fault 상태라 정상 제어가 불가능함. */
    Fault,
    /** 현재 구현에서는 해당 기능을 지원하지 않음. */
    Unsupported,
    /** Socket/Serial/Modbus 같은 통신 경로에서 오류 발생. */
    TransportError
};

/** @brief 성공/실패 코드와 사람이 읽을 수 있는 설명을 함께 담는 함수 반환값. */
struct Result
{
    /** @brief 성공이면 None, 실패면 원인을 나타내는 ErrorCode. */
    ErrorCode code = ErrorCode::None;

    /** @brief 실패 이유를 설명하는 문자열. 성공 시 비어 있을 수 있다. */
    std::string message;

    /** @return 성공(ErrorCode::None)이면 true. */
    [[nodiscard]] bool Ok() const noexcept { return code == ErrorCode::None; }

    /** @brief `if (result)`처럼 Result를 성공 여부로 검사하기 위한 변환. */
    explicit operator bool() const noexcept { return Ok(); }

    /** @return 성공 상태 Result. */
    static Result Success() { return {}; }
};

/**
 * @brief TCP의 위치와 방향을 함께 표현하는 Cartesian pose.
 *
 * @details
 * Pose는 "어디에 있는가(position) + 어느 방향을 보는가(orientation)"다.
 * TCP는 그리퍼/공구 끝에서 작업 기준으로 삼는 대표점이다.
 */
struct CartesianPose
{
    /** @brief TCP 위치 `[x,y,z]`, 단위 meter [m]. */
    std::array<double, 3> positionMeters{};

    /**
     * @brief TCP 방향을 나타내는 quaternion `[x,y,z,w]`.
     * @details 기본값 `{0,0,0,1}`은 회전이 없는 identity orientation이다.
     */
    std::array<double, 4> orientationXyzw{0.0, 0.0, 0.0, 1.0};
};

/**
 * @brief 관절각 목록으로 로봇을 움직이는 Joint-space 절대 위치 명령.
 *
 * @details
 * "J1을 30도, J2를 -10도..."처럼 각 관절의 목표각을 직접 지정하는 방식이다.
 * `targetPositionRadians`는 현재 각도에서 얼마나 더 움직일지를 뜻하는 delta가 아니라 최종 절대 목표각이다.
 */
struct JointMoveCommand
{
    /** @brief J1..Jn 절대 목표각 [rad]. RobotSpecification의 joint 순서와 같아야 한다. */
    JointVector targetPositionRadians;

    /**
     * @brief 모델 최대속도에 곱하는 무차원 비율. 현재 유효 범위 `(0,1]`.
     * @details 0.5면 해당 모델 최대속도의 50%까지만 사용한다.
     */
    double velocityScale = 1.0;

    /**
     * @brief 향후 최대가속도에 곱할 무차원 비율. 현재 유효 범위 `(0,1]`.
     * @note 아직 검증된 모델 가속도 limit이 없어 현재 SimRobotController에서는 실제 제한에 적용하지 않는다.
     */
    double accelerationScale = 1.0;
};

/**
 * @brief TCP(Tool Center Point, 로봇 말단 공구 중심점)를 Cartesian 직선 경로로 이동시키기 위한 고수준 요청.
 *
 * @details
 * 관절각을 직접 주는 대신 "공구 끝을 이 위치/방향으로 이동"이라고 명령하는 방식이다.
 * 실제로 움직이려면 IK와 trajectory 계산이 필요하다.
 */
struct LinearMoveCommand
{
    /** @brief 최종 TCP 목표 자세. 위치 [m], 방향 quaternion [x,y,z,w]. */
    CartesianPose targetPose{};

    /** @brief TCP가 직선으로 이동할 때 허용하는 최대 선속도 [m/s]. */
    double maxLinearVelocityMetersPerSecond = 0.25;

    /** @brief TCP 방향이 회전할 때 허용하는 최대 각속도 [rad/s]. */
    double maxAngularVelocityRadiansPerSecond = 0.5;
};

/** @brief Robot Controller가 현재 어떤 실행 단계에 있는지 나타내는 상태. */
enum class RobotMode
{
    /** 연결/초기화 전 또는 Disconnect 이후. */
    Disconnected,
    /** 연결되어 있지만 현재 이동 명령을 수행하지 않는 대기 상태. */
    Idle,
    /** 목표 관절각/자세를 향해 움직이는 중. */
    Moving,
    /** Stop() 명령으로 소프트웨어 이동이 중단된 상태. */
    Stopped,
    /** 오류 때문에 정상 이동 명령을 수행할 수 없는 상태. */
    Fault
};

/**
 * @brief 특정 시점의 로봇 상태를 복사해 담은 snapshot.
 *
 * @details
 * 현재 관절각/속도, TCP pose, 실행상태, fault 정보를 한 번에 전달한다.
 * `valid=false`이면 나머지 필드를 신뢰하면 안 된다.
 */
struct RobotState
{
    /** @brief 현재 관절 절대각 J1..Jn [rad]. */
    JointVector jointPositionRadians;

    /** @brief 현재 관절 각속도 J1..Jn [rad/s]. 양/음 부호는 회전 방향을 나타낸다. */
    JointVector jointVelocityRadiansPerSecond;

    /** @brief FK 계산 또는 실제 장비 feedback으로 얻은 TCP 위치/방향. */
    CartesianPose tcpPose{};

    /** @brief Controller의 현재 실행 상태. */
    RobotMode mode = RobotMode::Disconnected;

    /** @brief 실제 backend가 제공하는 원시 fault code. 0의 의미도 backend 규약을 따른다. */
    std::uint32_t faultCode = 0;

    /** @brief true일 때만 tcpPose가 현재 로봇 상태를 반영하는 유효한 값이다. */
    bool tcpPoseValid = false;

    /** @brief 전체 snapshot이 사용 가능한 상태인지 표시한다. */
    bool valid = false;
};

/**
 * @brief Gripper의 위치/속도/힘을 8-bit raw 값으로 요청하는 공통 명령.
 *
 * @details
 * 현재 2F-85에서는 다음 제조사 register 의미와 대응한다.
 * - positionRequest -> rPR: 0=open, 255=closed
 * - speedRequest -> rSP: 0=min, 255=max
 * - forceRequest -> rFR: 0=min, 255=max
 *
 * 이 숫자는 물리 단위 자체가 아니다. 예를 들어 positionRequest=128이 곧 42.5 mm라는 뜻은 아니다.
 * Simulation backend가 모델별 mapping을 통해 실제 opening/linkage angle로 변환해야 한다.
 */
struct GripperCommand
{
    /** @brief 위치 raw 요청값 0..255. 2F-85에서는 0=open, 255=closed. */
    std::uint8_t positionRequest = 0;

    /** @brief 이동속도 raw 요청값 0..255. 실제 mm/s 값 자체가 아니다. */
    std::uint8_t speedRequest = 255;

    /** @brief 파지힘 raw 요청값 0..255. 실제 Newton[N] 값 자체가 아니다. */
    std::uint8_t forceRequest = 128;
};

/**
 * @brief Gripper가 이동 중인지, 물체를 만나 멈췄는지, 목표 위치에 도달했는지를 나타내는 상태.
 * @details 현재 값은 Robotiq 2F-85의 gOBJ 의미와 맞춘다.
 */
enum class GripperObjectStatus : std::uint8_t
{
    /** 목표 위치를 향해 이동 중이며 아직 접촉으로 멈추지 않음. */
    Moving = 0,
    /** 손가락을 여는 중 물체와 접촉해 멈춤. */
    ContactWhileOpening = 1,
    /** 손가락을 닫는 중 물체와 접촉해 멈춤. */
    ContactWhileClosing = 2,
    /** 요청 위치에 도달함. */
    AtRequestedPosition = 3
};

/**
 * @brief 특정 시점의 gripper 상태 snapshot.
 *
 * @details
 * Robotiq 용어를 모르는 경우:
 * - activation: 장치를 명령 가능한 초기화 상태로 만드는 과정.
 * - Go-To: 지정한 위치로 이동하라는 동작 요청.
 * - echo: 장치가 마지막으로 받아들인 요청값을 다시 돌려주는 값.
 * - current: 모터가 얼마나 전류를 쓰는지에 관한 feedback. 현재 필드는 물리 A가 아니라 raw register 값이다.
 */
struct GripperState
{
    /** @brief 장치 초기화/activation이 완료되어 정상 명령을 받을 준비가 되었는지 여부. */
    bool activated = false;

    /** @brief 현재 목표 위치로 이동하는 Go-To 동작이 활성화되어 있는지 여부. */
    bool goToActive = false;

    /** @brief 제조사 activation/state raw code. 2F-85 backend에서는 gSTA에 해당한다. */
    std::uint8_t activationStatus = 0;

    /** @brief 이동/접촉/목표 도달 상태. */
    GripperObjectStatus objectStatus = GripperObjectStatus::AtRequestedPosition;

    /** @brief 제조사 fault raw code. 2F-85에서는 gFLT에 해당한다. */
    std::uint8_t faultCode = 0;

    /** @brief 장치가 마지막으로 반영한 위치 요청 echo. 2F-85에서는 gPR, 범위 0..255. */
    std::uint8_t requestedPositionEcho = 0;

    /** @brief 실제 위치 feedback raw 값. 2F-85에서는 gPO, 범위 0..255. */
    std::uint8_t actualPosition = 0;

    /** @brief 모터 전류 feedback의 raw register 값. 물리 전류[A]로 변환되기 전 값. */
    std::uint8_t currentRaw = 0;

    /** @brief true일 때만 이 snapshot의 나머지 필드를 현재 상태로 사용한다. */
    bool valid = false;
};

} // namespace grasplink::robotics
