#include "viewer/robotics/GripperTransformAdapter.h"

#include "components/TransformComponents.h"

#include <glm/gtx/quaternion.hpp>

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace grasplink::viewer::robotics
{
namespace
{
constexpr float kUnitScaleTolerance = 1.0e-4F;

bool IsFinite(const glm::vec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool IsUnitScale(const glm::vec3& value)
{
    return std::abs(value.x - 1.0F) <= kUnitScaleTolerance &&
        std::abs(value.y - 1.0F) <= kUnitScaleTolerance &&
        std::abs(value.z - 1.0F) <= kUnitScaleTolerance;
}

bool IsFinite(const ::grasplink::robotics::models::QuaternionWxyz& value)
{
    return std::isfinite(value.w) && std::isfinite(value.x) &&
        std::isfinite(value.y) && std::isfinite(value.z);
}

glm::dquat ToDoubleQuaternion(const ::grasplink::robotics::models::QuaternionWxyz& value)
{
    return glm::dquat{value.w, value.x, value.y, value.z};
}

::grasplink::robotics::models::QuaternionWxyz ToModelQuaternion(const glm::dquat& value)
{
    return {value.w, value.x, value.y, value.z};
}

void ValidateLocalTransform(const Entity& entity)
{
    const glm::vec3 position = entity.GetLocalPosition();
    const glm::vec3 rotation = entity.GetLocalRotation();
    const glm::vec3 scale = entity.GetLocalScale();
    if (!IsFinite(position) || !IsFinite(rotation) || !IsFinite(scale))
        throw std::invalid_argument("GripperTransformAdapter: non-finite bind transform");
    if (!IsUnitScale(scale))
        throw std::invalid_argument("GripperTransformAdapter: gripper hierarchy requires unit scale");
}

bool IsSameHierarchy(const Entity& joint, const std::vector<Entity>& ancestry)
{
    if (ancestry.empty() || !joint.IsValid())
        return false;
    if (ancestry.front() != joint)
        return false;

    for (std::size_t i = 0; i < ancestry.size(); ++i)
    {
        if (!ancestry[i].IsValid())
            return false;
        if (i + 1 < ancestry.size() && ancestry[i].GetParent() != ancestry[i + 1])
            return false;
    }
    return true;
}
}

GripperTransformAdapter::GripperTransformAdapter(
    const Entity& gripperRoot,
    const ::grasplink::robotics::models::GripperSpecification& specification)
    : gripperRoot_(gripperRoot)
{
    if (!gripperRoot_)
        throw std::runtime_error("GripperTransformAdapter: invalid Gripper root");
    if (specification.joints == nullptr || specification.jointCount == 0)
        throw std::invalid_argument("GripperTransformAdapter: empty joint specification");

    ValidateLocalTransform(gripperRoot_);

    // 사양에 적힌 이름만 모은다. 그 외 authored mesh/link node는 원본 변환을 그대로 유지한다.
    std::unordered_set<std::string> wantedNames;
    wantedNames.reserve(specification.jointCount);
    for (std::size_t i = 0; i < specification.jointCount; ++i)
    {
        const std::string name(specification.joints[i].name);
        if (name.empty() || !wantedNames.emplace(name).second)
            throw std::invalid_argument("GripperTransformAdapter: empty or duplicate specification joint name");
    }

    std::unordered_map<std::string, Entity> foundJoints;
    std::function<void(const Entity&)> collect = [&](const Entity& parent)
    {
        for (const Entity& child : parent.GetChildren())
        {
            const char* rawName = child.GetHandle().name().c_str();
            const std::string name = rawName == nullptr ? std::string{} : std::string(rawName);
            if (wantedNames.count(name) != 0 && !foundJoints.emplace(name, child).second)
                throw std::runtime_error("GripperTransformAdapter: duplicate GLB joint name: " + name);
            collect(child);
        }
    };
    collect(gripperRoot_);

    joints_.reserve(specification.jointCount);
    for (std::size_t i = 0; i < specification.jointCount; ++i)
    {
        const std::string name(specification.joints[i].name);
        const auto found = foundJoints.find(name);
        if (found == foundJoints.end())
            throw std::runtime_error("GripperTransformAdapter: joint not found: " + name);

        JointBinding binding;
        binding.entity = found->second;
        for (Entity current = binding.entity; current; current = current.GetParent())
        {
            ValidateLocalTransform(current);
            binding.ancestry.push_back(current);
            if (current == gripperRoot_)
                break;
        }
        if (binding.ancestry.empty() || binding.ancestry.back() != gripperRoot_)
            throw std::runtime_error("GripperTransformAdapter: joint is outside the Gripper root: " + name);

        // Node 22 Gripper의 non-identity matrix는 root Local TRS에 남긴다. 관절 bind 회전만 별도로 저장한다.
        const glm::dvec3 bindEuler(binding.entity.GetLocalRotation());
        const glm::dquat rawBindRotation{bindEuler};
        const double rawBindLengthSquared = glm::dot(rawBindRotation, rawBindRotation);
        if (!std::isfinite(rawBindLengthSquared) || rawBindLengthSquared <= 0.0)
            throw std::invalid_argument("GripperTransformAdapter: invalid bind rotation: " + name);
        const glm::dquat bindRotation = glm::normalize(rawBindRotation);
        const double lengthSquared = glm::dot(bindRotation, bindRotation);
        if (!std::isfinite(lengthSquared) || lengthSquared <= 0.0)
            throw std::invalid_argument("GripperTransformAdapter: invalid bind rotation: " + name);
        binding.bindRotation = ToModelQuaternion(bindRotation);
        joints_.push_back(std::move(binding));
    }
}

void GripperTransformAdapter::Apply(
    const ::grasplink::robotics::kinematics::GripperKinematicState& state)
{
    if (!std::isfinite(state.masterAngleRadians) ||
        state.jointAnglesRadians.size() != joints_.size() ||
        state.jointLocalRotations.size() != joints_.size())
        throw std::invalid_argument("GripperTransformAdapter: joint pose count or master angle is invalid");
    if (!gripperRoot_.IsValid())
        throw std::runtime_error("GripperTransformAdapter: Gripper Scene has been removed");

    // 먼저 모든 handle과 부모 경로를 확인한다. 한 관절만 재부모화된 상태에서 일부 pose를 쓰지 않는다.
    for (const JointBinding& binding : joints_)
        if (!IsSameHierarchy(binding.entity, binding.ancestry))
            throw std::runtime_error("GripperTransformAdapter: Gripper hierarchy changed or was removed");

    std::vector<glm::vec3> localEulerRotations;
    localEulerRotations.reserve(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        if (!std::isfinite(state.jointAnglesRadians[i]) || !IsFinite(state.jointLocalRotations[i]))
            throw std::invalid_argument("GripperTransformAdapter: non-finite joint pose");

        const glm::dquat rawDelta = ToDoubleQuaternion(state.jointLocalRotations[i]);
        const double rawDeltaLengthSquared = glm::dot(rawDelta, rawDelta);
        if (!std::isfinite(rawDeltaLengthSquared) || rawDeltaLengthSquared <= 0.0)
            throw std::invalid_argument("GripperTransformAdapter: zero or invalid delta rotation");
        const glm::dquat delta = glm::normalize(rawDelta);

        // local pose = authored bind * joint-local delta. 곱의 오른쪽에 delta를 둬 bind 기준 축을 따른다.
        const glm::dquat bind = glm::normalize(ToDoubleQuaternion(joints_[i].bindRotation));
        const glm::dquat result = glm::normalize(bind * delta);
        const glm::dvec3 euler = glm::eulerAngles(result);
        const glm::vec3 output{
            static_cast<float>(euler.x),
            static_cast<float>(euler.y),
            static_cast<float>(euler.z)};
        if (!std::isfinite(euler.x) || !std::isfinite(euler.y) || !std::isfinite(euler.z) || !IsFinite(output))
            throw std::invalid_argument("GripperTransformAdapter: calculated non-finite Euler rotation");
        localEulerRotations.push_back(output);
    }

    // GLM Entity의 Euler 저장 형식으로 바꾸는 마지막 단계까지 계산을 끝낸 후 Local 회전만 쓴다.
    for (std::size_t i = 0; i < joints_.size(); ++i)
        joints_[i].entity.SetLocalRotation(localEulerRotations[i]);
}

} // namespace grasplink::viewer::robotics
