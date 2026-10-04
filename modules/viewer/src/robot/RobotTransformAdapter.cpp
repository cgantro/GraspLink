#include "robot/RobotTransformAdapter.h"

#include <glm/gtx/quaternion.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace
{
constexpr float kAxisEpsilon = 1e-8F;
constexpr float kBindRotationEpsilon = 1e-4F;
}

RobotTransformAdapter::RobotTransformAdapter(
    const Entity& robotRoot,
    const control::specs::RobotSpecification& specification)
{
    if (!robotRoot)
        throw std::runtime_error("RobotTransformAdapter: invalid robot root");

    if (specification.joints == nullptr || specification.jointCount == 0)
        throw std::invalid_argument("RobotTransformAdapter: empty robot specification");

    joints_.reserve(specification.jointCount);

    for (std::size_t i = 0; i < specification.jointCount; ++i)
    {
        const auto& jointSpec = specification.joints[i];
        Entity joint = robotRoot.FindChildByNameRecursive(std::string(jointSpec.name));
        if (!joint)
            throw std::runtime_error(
                "RobotTransformAdapter: joint not found: " + std::string(jointSpec.name));

        const glm::vec3 bindEuler = joint.GetLocalRotation();
        if (glm::dot(bindEuler, bindEuler) >
            kBindRotationEpsilon * kBindRotationEpsilon)
        {
            throw std::runtime_error(
                "RobotTransformAdapter: controller-ready GLB requires identity bind rotation: " +
                std::string(jointSpec.name));
        }

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

void RobotTransformAdapter::Apply(const control::RobotState& state)
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

        const auto& binding = joints_[i];
        glm::vec3 axis{
            static_cast<float>(binding.axis.x),
            static_cast<float>(binding.axis.y),
            static_cast<float>(binding.axis.z)};
        axis = glm::normalize(axis);

        const glm::quat rotation = glm::angleAxis(static_cast<float>(position), axis);
        binding.entity.SetLocalRotation(glm::eulerAngles(rotation));
    }
}
