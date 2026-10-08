#include "simulation/robotics/GripperTransformAdapter.h"

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

namespace grasplink::simulation::robotics
{
namespace
{
constexpr float kUnitScaleTolerance = 1.0e-4F;

bool IsFinite(const glm::vec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool IsFinite(const glm::quat& value)
{
    return std::isfinite(value.w) && std::isfinite(value.x) &&
        std::isfinite(value.y) && std::isfinite(value.z);
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
    const glm::quat rotation = entity.GetLocalRotation();
    const glm::vec3 scale = entity.GetLocalScale();
    if (!IsFinite(position) || !IsFinite(rotation) || !IsFinite(scale))
        throw std::invalid_argument("GripperTransformAdapter: non-finite bind transform");
    // 일반 setter를 거치지 않고 ECS에서 직접 기록한 값도 검사한다. 영 quaternion은 방향을 나타낼 수 없으므로 bind 계층 연결을 거부한다.
    (void)Rotation{rotation};
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

    // 모델 사양에 있는 관절 이름만 검색 대상으로 삼는다. 메시와 link처럼 관절이 아닌 authored node는 저장된 변환을 그대로 유지한다.
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

        // 이 GLB에서는 Node 22 Gripper에 비항등 장착 변환이 있으므로 이를 root의 Local TRS에 둔다. 관절별 bind 회전만 따로 저장해 개폐 delta와 합성한다.
        const glm::dquat rawBindRotation{binding.entity.GetLocalRotation()};
        const double rawBindLengthSquared = glm::dot(rawBindRotation, rawBindRotation);
        if (!std::isfinite(rawBindLengthSquared) || rawBindLengthSquared <= 0.0)
            throw std::invalid_argument("GripperTransformAdapter: invalid bind rotation: " + name);
        const glm::dquat bindRotation = glm::normalize(rawBindRotation);
        binding.bindRotation = ToModelQuaternion(bindRotation);
        joints_.push_back(std::move(binding));
    }
    preparedRotations_.resize(joints_.size());
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

    // 먼저 모든 관절 handle과 생성 시 저장한 부모 경로가 유지되는지 확인한다. 하나라도 재부모화되었으면 일부 관절에만 새 자세를 쓰지 않고 전체 적용을 중단한다.
    for (const JointBinding& binding : joints_)
        if (!IsSameHierarchy(binding.entity, binding.ancestry))
            throw std::runtime_error("GripperTransformAdapter: Gripper hierarchy changed or was removed");

    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        if (!std::isfinite(state.jointAnglesRadians[i]) || !IsFinite(state.jointLocalRotations[i]))
            throw std::invalid_argument("GripperTransformAdapter: non-finite joint pose");

        const glm::dquat rawDelta = ToDoubleQuaternion(state.jointLocalRotations[i]);
        const double rawDeltaLengthSquared = glm::dot(rawDelta, rawDelta);
        if (!std::isfinite(rawDeltaLengthSquared) || rawDeltaLengthSquared <= 0.0)
            throw std::invalid_argument("GripperTransformAdapter: zero or invalid delta rotation");
        const glm::dquat delta = glm::normalize(rawDelta);

        // 최종 Local 회전은 GLB에 저장된 bind 회전에 관절 Local delta를 오른쪽으로 곱한 값이다. 이 순서는 delta 축을 bind 기준으로 해석한다.
        const glm::dquat bind = glm::normalize(ToDoubleQuaternion(joints_[i].bindRotation));
        const glm::dquat result = glm::normalize(bind * delta);
        preparedRotations_[i] = Rotation{glm::quat{result}};
    }

    // 모든 quaternion을 먼저 검증하고 정규화한 뒤에야 관절 Local 회전을 쓴다. Euler 각으로 바꾸지 않아 중간 변환에서 생길 수 있는 특이점도 피한다.
    for (std::size_t i = 0; i < joints_.size(); ++i)
        joints_[i].entity.SetLocalRotation(preparedRotations_[i]);
}

} // namespace grasplink::viewer::robotics
