#pragma once

#include "robotics/models/ModelTypes.h"

#include <cstddef>
#include <string_view>

namespace grasplink::robotics::models
{

// 로봇 모델의 회전 관절 하나를 정의한다.
struct JointSpecification
{
    // GLB/Flecs에서 관절을 찾는 논리 이름. 예: "J1".
    std::string_view name;

    // Bind pose의 robot-base 기준 회전 중심 [m]. HCR pivot은 CAD 유래 값이다.
    Vec3 bindPivotMeters;

    // 관절 local frame 기준 회전축 방향. 단위 없음.
    Axis3 axis;

    // 허용 각도 범위 [rad].
    double minPositionRadians = 0.0;
    double maxPositionRadians = 0.0;

    // 최대 각속도 [rad/s].
    double maxVelocityRadiansPerSecond = 0.0;
};

struct LinkSpecification
{
    // 이 항목이 나타내는 Link 이름.
    std::string_view name;
    // joints 배열에서 대응하는 관절의 index.
    std::size_t jointIndex = 0;
};

// 로봇 모델의 문자열과 상수 배열을 가리키는 view. 데이터는 소유하지 않는다.
struct RobotSpecification
{
    // string_view와 배열 포인터의 대상은 RobotSpecification보다 오래 살아야 한다.

    // 제조사 표시 이름.
    std::string_view manufacturer;

    // 제품 모델명.
    std::string_view model;

    // J1..Jn 순서의 관절 배열.
    const JointSpecification* joints = nullptr;

    // 관절 수. RobotState와 JointMoveCommand 벡터 길이의 기준.
    std::size_t jointCount = 0;

    const LinkSpecification* links = nullptr;
    std::size_t linkCount = 0;

    // 마지막 관절에서 ToolFrame까지의 고정 변환. 장착 공구의 TCP는 별도다.
    Pose3 toolFrameInLastJoint{};
    bool hasToolFrame = false;
};

} // namespace grasplink::robotics::models
