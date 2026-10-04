#include "viewer/robotics/RobotTransformAdapter.h"

#include <glm/gtx/quaternion.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace grasplink::viewer::robotics
{
namespace
{
/** @brief Joint axis가 사실상 0 vector인지 검사하기 위한 squared-length 기준. */
constexpr float kAxisEpsilon = 1e-8F;

/**
 * @brief controller-ready GLB Joint bind Euler가 identity인지 검사하는 허용 오차 [rad].
 *
 * GLB loader가 quaternion/matrix를 Euler로 변환하므로 부동소수점의 아주 작은 잔차는 허용한다.
 */
constexpr float kBindRotationEpsilon = 1e-4F;
}

RobotTransformAdapter::RobotTransformAdapter(
    const Entity& robotRoot,
    const ::grasplink::robotics::models::RobotSpecification& specification)
{
    if (!robotRoot)
        throw std::runtime_error("RobotTransformAdapter: invalid robot root");

    if (specification.joints == nullptr || specification.jointCount == 0)
        throw std::invalid_argument("RobotTransformAdapter: empty robot specification");

    joints_.reserve(specification.jointCount);

    for (std::size_t i = 0; i < specification.jointCount; ++i)
    {
        const auto& jointSpec = specification.joints[i];

        /*
         * Model specification의 `name`과 GLB Node/Flecs Entity 이름을 binding key로 사용한다.
         * HCR-12A의 경우 J1..J6가 이에 해당한다. 따라서 asset 이름을 변경하면 model specification도
         * 같이 바꾸거나 별도의 name mapping 계층을 도입해야 한다.
         */
        Entity joint = robotRoot.FindChildByNameRecursive(std::string(jointSpec.name));
        if (!joint)
            throw std::runtime_error(
                "RobotTransformAdapter: joint not found: " + std::string(jointSpec.name));

        /*
         * controller-ready GLB에서는 moving Joint Node의 bind rotation을 identity로 정규화했다.
         * 과거 asset처럼 bind quaternion 보정이 남아 있으면 `bind * delta` 규칙이 다시 필요하고,
         * model axis와 시각 axis가 어긋날 수 있으므로 여기서 즉시 실패시킨다.
         */
        const glm::vec3 bindEuler = joint.GetLocalRotation();
        if (glm::dot(bindEuler, bindEuler) >
            kBindRotationEpsilon * kBindRotationEpsilon)
        {
            throw std::runtime_error(
                "RobotTransformAdapter: controller-ready GLB requires identity bind rotation: " +
                std::string(jointSpec.name));
        }

        /*
         * RobotSpecification은 GLM에 의존하지 않으므로 Axis3(double)를 사용한다.
         * Viewer 경계에서만 glm::vec3(float)로 변환한다. axis는 방향값이라 물리 단위가 없고,
         * 실제 회전 계산 전 normalize된다.
         */
        const glm::vec3 axis{
            static_cast<float>(jointSpec.axis.x),
            static_cast<float>(jointSpec.axis.y),
            static_cast<float>(jointSpec.axis.z)};

        if (glm::dot(axis, axis) <= kAxisEpsilon)
            throw std::runtime_error(
                "RobotTransformAdapter: invalid joint axis: " + std::string(jointSpec.name));

        joints_.push_back({joint, jointSpec.axis});
    }
}

void RobotTransformAdapter::Apply(const ::grasplink::robotics::RobotState& state)
{
    if (!state.valid)
        return;

    if (state.jointPositionRadians.size() != joints_.size())
        throw std::invalid_argument("RobotTransformAdapter: RobotState joint count mismatch");

    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        const double position = state.jointPositionRadians[i];
        if (!std::isfinite(position))
            throw std::invalid_argument("RobotTransformAdapter: non-finite joint position");

        auto& binding = joints_[i];

        glm::vec3 axis{
            static_cast<float>(binding.axis.x),
            static_cast<float>(binding.axis.y),
            static_cast<float>(binding.axis.z)};
        axis = glm::normalize(axis);

        /*
         * 제어값 -> 그래픽 회전 변환:
         *
         * 1. Controller angle: position [rad]
         * 2. Model axis: joint-local unit vector
         * 3. angleAxis(position, axis) -> quaternion
         * 4. 현재 ECS Rotation이 vec3 Euler(rad)이므로 eulerAngles()로 저장 형식에 맞춤
         * 5. TransformSystemModule이 다음 ECS update에서 Euler -> quaternion -> 4x4 rotation matrix로 변환
         * 6. GLB Node translation이 pivot을 이미 표현하므로 rotation만 바꿔도 자식 Link가 실제 관절 중심에서 회전
         *
         * 즉 JointSpecification::bindPivotMeters를 여기서 position에 더하지 않는다.
         */
        const glm::quat rotation = glm::angleAxis(static_cast<float>(position), axis);
        binding.entity.SetLocalRotation(glm::eulerAngles(rotation));
    }
}

} // namespace grasplink::viewer::robotics
