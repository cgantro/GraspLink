#pragma once

namespace grasplink::robotics::models
{

/**
 * @brief Robotics model specification에서 사용하는 backend-neutral 3차원 실수 벡터.
 *
 * @details
 * 이 타입 자체는 단위를 강제하지 않는다. 실제 단위/좌표계는 이 값을 포함하는 필드 이름과 문서가 결정한다.
 * 예를 들어 `bindPivotMeters`에 사용되면 각 성분은 meter [m], `axis`에 사용되면 무차원 방향 성분이다.
 * GLM에 의존하지 않아 robotics core/model 계층이 Viewer/Graphics 계층과 분리되도록 한다.
 */
struct Vec3
{
    /** @brief X 성분. 단위는 사용 문맥을 따른다. */
    double x = 0.0;
    /** @brief Y 성분. 단위는 사용 문맥을 따른다. */
    double y = 0.0;
    /** @brief Z 성분. 단위는 사용 문맥을 따른다. */
    double z = 0.0;
};

/**
 * @brief Joint local frame의 회전축을 표현하는 Vec3 alias.
 *
 * @note JointSpecification/GripperJointSpecification에서는 단위 길이(unit-length) 방향벡터를 전제로 한다.
 * RobotTransformAdapter는 실제 적용 시 GLM vec3로 변환한 뒤 normalize한다.
 */
using Axis3 = Vec3;

} // namespace grasplink::robotics::models
