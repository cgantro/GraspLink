#pragma once

#include "robotics/models/ModelTypes.h"

#include <cstddef>
#include <string_view>

namespace grasplink::robotics::models
{

/**
 * @brief 특정 robot model의 하나의 revolute joint에 대한 정적 규격.
 *
 * @details
 * 이 구조체는 현재 joint state가 아니라 "모델이 원래 어떻게 정의되어 있는가"를 보관한다.
 * 따라서 position/velocity가 매 frame 변하지 않고, Controller와 Viewer adapter가 공통 기준으로 참조한다.
 *
 * 좌표/단위 규칙:
 * - name: GLB/Flecs Joint Entity 이름과 대응하는 논리 이름
 * - bindPivotMeters: controller-ready asset의 bind pose에서 관절 회전 중심 [m]
 * - axis: Joint local frame 기준 회전축, 무차원 unit direction
 * - min/maxPositionRadians: 허용 절대 joint angle [rad]
 * - maxVelocityRadiansPerSecond: 허용 최대 각속도 [rad/s]
 *
 * @note bindPivotMeters는 현재 RobotTransformAdapter가 매 frame translation에 더하는 값이 아니다.
 *       Pivot translation/hierarchy는 GLB Node 자체에 이미 들어 있으며, 이 값은 모델 규격/검증/FK 구현의 기준값이다.
 */
struct JointSpecification
{
    /** @brief Joint 논리 이름. 예: "J1". Viewer에서는 같은 이름의 GLB Node를 찾는 key로 사용한다. */
    std::string_view name;

    /** @brief Bind pose 기준 관절 회전 중심 [x,y,z], 단위 meter [m]. */
    Vec3 bindPivotMeters;

    /** @brief Joint local frame 기준 회전축. 예: J1의 +Y는 {0,1,0}. */
    Axis3 axis;

    /** @brief 허용 최소 절대 관절각 [rad]. */
    double minPositionRadians = 0.0;

    /** @brief 허용 최대 절대 관절각 [rad]. */
    double maxPositionRadians = 0.0;

    /** @brief 허용 최대 관절 각속도 크기 [rad/s]. */
    double maxVelocityRadiansPerSecond = 0.0;
};

/**
 * @brief 특정 robot model의 정적 규격 배열을 가리키는 non-owning view.
 *
 * @details
 * 제조사/모델명과 JointSpecification 배열을 묶어 Simulation/Hardware/Viewer가 같은 모델 정의를 공유하게 한다.
 * 새로운 4축/7축 robot을 추가할 때 Controller 인터페이스를 바꾸지 않고 새 specification만 정의할 수 있다.
 *
 * @note joints는 프로그램 수명 동안 유효한 constexpr/static 배열을 가리켜야 한다.
 */
struct RobotSpecification
{
    /** @brief 제조사 표시 이름. 예: "Hanwha Robotics". */
    std::string_view manufacturer;

    /** @brief 모델 표시 이름. 예: "HCR-12A". */
    std::string_view model;

    /** @brief J1..Jn 순서의 정적 joint 규격 배열 시작 주소. 소유권은 없다. */
    const JointSpecification* joints = nullptr;

    /** @brief joints 배열 원소 개수. RobotState/JointMoveCommand vector 길이의 기준이 된다. */
    std::size_t jointCount = 0;
};

} // namespace grasplink::robotics::models
