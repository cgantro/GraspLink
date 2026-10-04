#pragma once

#include "robotics/models/ModelTypes.h"

#include <cstddef>
#include <string_view>

namespace grasplink::robotics::models
{

/**
 * @file RobotSpecification.h
 * @brief Robot 모델의 "변하지 않는 기계/제어 사양"을 표현하는 타입.
 *
 * @details
 * 로보틱스 용어를 모르는 경우 아래처럼 읽으면 된다.
 *
 * - Joint(관절): 로봇 팔이 꺾이거나 도는 회전 부분. HCR-12A의 J1~J6가 각각 하나의 Joint다.
 * - Link(링크): Joint와 Joint 사이의 단단한 팔 부품.
 * - Revolute joint: 직선으로 미끄러지는 대신 축을 중심으로 회전하는 관절.
 * - DOF(Degree of Freedom, 자유도): 독립적으로 움직일 수 있는 축의 개수. HCR-12A는 6개의 회전관절이라 6-DOF다.
 * - Pivot: 관절이 실제로 회전하는 중심점.
 * - Axis: Pivot을 지나며, 어떤 방향을 중심으로 회전하는지 나타내는 축.
 * - Joint limit: 관절이 기계적으로/제어상 움직일 수 있는 최소·최대 각도 범위.
 * - Bind pose: 모델을 처음 불러왔을 때의 기준 자세.
 *
 * 이 파일의 Specification은 "현재 몇 도 움직였는지"가 아니라
 * "이 로봇은 원래 어디를 중심으로 어느 축으로 얼마나 움직일 수 있는지"를 저장한다.
 */

/**
 * @brief 특정 robot model의 회전관절 하나에 대한 고정 사양.
 *
 * @details
 * 예를 들어 HCR-12A J2라면 이 구조체 하나가 J2의 이름, 회전 중심, 회전축,
 * 최소/최대 각도, 최대 회전속도를 정의한다.
 */
struct JointSpecification
{
    /**
     * @brief 관절 이름.
     * @details 예: "J1", "J2". Viewer는 같은 이름의 GLB Joint Node를 찾아 모델 사양과 화면상의 관절을 연결한다.
     */
    std::string_view name;

    /**
     * @brief Bind pose에서의 관절 회전 중심 [x,y,z], 단위 meter [m].
     * @details Mesh의 중앙점이 아니라 실제 기계 관절의 중심점이다. 현재 GLB hierarchy에도 이 pivot translation이 이미 반영돼 있다.
     */
    Vec3 bindPivotMeters;

    /**
     * @brief Joint 자신의 local 좌표계에서 본 회전축.
     * @details 예: `{0,1,0}`이면 local +Y축을 중심으로 회전한다. 방향값이므로 물리 단위는 없다.
     */
    Axis3 axis;

    /** @brief 허용되는 최소 절대 관절각 [rad]. 이 값보다 작은 command는 거부한다. */
    double minPositionRadians = 0.0;

    /** @brief 허용되는 최대 절대 관절각 [rad]. 이 값보다 큰 command는 거부한다. */
    double maxPositionRadians = 0.0;

    /**
     * @brief 이 관절이 허용하는 최대 회전속도 크기 [rad/s].
     * @details `rad/s`는 1초 동안 몇 radian까지 회전할 수 있는지를 뜻한다.
     */
    double maxVelocityRadiansPerSecond = 0.0;
};

/**
 * @brief 특정 robot model 전체의 고정 사양을 묶어 제공하는 non-owning view.
 *
 * @details
 * `RobotSpecification` 하나가 "이 로봇은 어느 제조사의 무슨 모델이고, 관절이 몇 개이며,
 * 각 관절 사양은 무엇인가"를 나타낸다.
 *
 * `non-owning`은 joints 배열의 메모리를 직접 소유하지 않고 주소만 보관한다는 뜻이다.
 * 따라서 joints가 가리키는 constexpr/static 배열은 이 Specification보다 오래 살아 있어야 한다.
 *
 * 새 4축/7축 로봇을 추가해도 IRobotController 인터페이스는 그대로 두고
 * 이 Specification만 새로 정의하는 것을 목표로 한다.
 */
struct RobotSpecification
{
    /** @brief 제조사 표시 이름. 예: "Hanwha Robotics". */
    std::string_view manufacturer;

    /** @brief 제품 모델명. 예: "HCR-12A". */
    std::string_view model;

    /**
     * @brief J1..Jn 순서로 저장된 JointSpecification 배열의 첫 원소 주소.
     * @note 소유권은 없다. JointMoveCommand/RobotState의 배열 순서도 반드시 이 순서와 같아야 한다.
     */
    const JointSpecification* joints = nullptr;

    /**
     * @brief joints 배열의 원소 개수, 즉 이 모델에서 제어하는 관절 수.
     * @details HCR-12A는 6이며, 다른 모델은 4/7 등 다른 값을 가질 수 있다.
     */
    std::size_t jointCount = 0;
};

} // namespace grasplink::robotics::models
