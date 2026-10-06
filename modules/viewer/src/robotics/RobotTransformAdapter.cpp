#include "viewer/robotics/RobotTransformAdapter.h"
#include "systems/TransformSystemModule.h"
#include "components/TransformComponents.h"

#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace grasplink::viewer::robotics
{
namespace
{
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

    std::unordered_set<std::string> wantedJoints;
    // 모델 사양에 나열된 관절 이름을 모아 GLB Entity 계층에서 같은 이름의 노드를 찾는다.
    for (std::size_t i = 0; i < specification.jointCount; ++i)
        wantedJoints.emplace(specification.joints[i].name);

    std::unordered_map<std::string, Entity> jointEntities;
    std::function<void(const Entity&)> collectJoints = [&](const Entity& parent)
    {
        for (const Entity& child : parent.GetChildren())
        {
            const char* entityName = child.GetHandle().name().c_str();
            const std::string name = entityName ? entityName : "";
            if (wantedJoints.erase(name) != 0)
                jointEntities.emplace(name, child);
            if (!wantedJoints.empty())
                collectJoints(child);
        }
    };
    collectJoints(robotRoot);

    joints_.reserve(specification.jointCount);

    for (std::size_t i = 0; i < specification.jointCount; ++i)
    {
        const auto& jointSpec = specification.joints[i];
        const auto found = jointEntities.find(std::string(jointSpec.name));
        if (found == jointEntities.end())
            throw std::runtime_error(
                "RobotTransformAdapter: joint not found: " + std::string(jointSpec.name));
        const Entity& joint = found->second;

        // 관절의 Local 변환과 중간 부모 변환을 누적해 모델 base 기준 pivot을 구한다. Scene 안에서 로봇을 배치하는 robotRoot 변환은 이 bind 검증에 포함하지 않는다.
        glm::mat4 bindInBase = TransformSystemModule::ComposeLocalMatrix(
            joint.GetLocalPosition(), joint.GetLocalRotation(), joint.GetLocalScale());
        Entity parent = joint.GetParent();
        bool followsPreviousJoint = i == 0;
        while (parent && parent != robotRoot)
        {
            // 첫 관절은 robotRoot 아래에 있어야 하고 이후 관절은 직전 관절의 자손이어야 한다. GLB의 중간 link node는 경로에 있어도 된다.
            if (i > 0 && parent == joints_[i - 1].entity)
                followsPreviousJoint = true;
            bindInBase = TransformSystemModule::ComposeLocalMatrix(
                parent.GetLocalPosition(), parent.GetLocalRotation(), parent.GetLocalScale()) * bindInBase;
            parent = parent.GetParent();
        }
        const glm::vec3 expectedPivot{static_cast<float>(jointSpec.bindPivotMeters.x),
            static_cast<float>(jointSpec.bindPivotMeters.y), static_cast<float>(jointSpec.bindPivotMeters.z)};
        const glm::mat4 expectedBind = glm::translate(glm::mat4{1.0F}, expectedPivot);
        // FK는 모델 사양의 pivot 위치를 유지한 채 관절 회전만 바꾼다. 따라서 GLB의 bind 행렬도 그 위치와 항등 회전·크기를 가져야 한다.
        bool matchesBind = parent == robotRoot && followsPreviousJoint;
        for (int column = 0; column < 4; ++column)
            matchesBind = matchesBind && glm::all(glm::lessThanEqual(
                glm::abs(bindInBase[column] - expectedBind[column]), glm::vec4{1.0e-5F}));
        if (!matchesBind)
            throw std::invalid_argument("RobotTransformAdapter: GLB hierarchy or bind pivot differs from robot specification");

        // quaternion q와 -q는 같은 회전이므로 성분 전체를 직접 비교하지 않는다. 정규화한 quaternion의 벡터부가 0인지 확인해 항등 회전을 판정한다.
        const glm::quat bindRotation = Rotation{joint.GetLocalRotation()};
        const glm::vec3 bindVector{bindRotation.x, bindRotation.y, bindRotation.z};
        if (glm::dot(bindVector, bindVector) >
            kBindRotationEpsilon * kBindRotationEpsilon)
        {
            throw std::runtime_error(
                "RobotTransformAdapter: controller-ready GLB requires identity bind rotation: " +
                std::string(jointSpec.name));
        }

        joints_.push_back({joint});
    }
}

void RobotTransformAdapter::Apply(const ::grasplink::robotics::kinematics::RobotKinematicState& state)
{
    // FK 결과는 사양 순서로 나오므로 연결한 GLB 관절 수와 회전 결과 수가 같아야 한다.
    if (state.jointLocalRotations.size() != joints_.size())
        throw std::invalid_argument("RobotTransformAdapter: joint rotation count mismatch");

    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        // GLB가 저장한 관절 위치와 부모 관계는 그대로 두고, FK가 계산한 부모 기준 Local 회전만 적용한다.
        const auto& pose = state.jointLocalRotations[i];
        // 모델 quaternion의 성분 순서는 [w,x,y,z]이며 GLM 생성자도 같은 순서를 받는다. 계산은 double 정밀도로 전달되고 Entity 저장 때 float로 바뀐다.
        const glm::dquat rotation{pose.w, pose.x, pose.y, pose.z};
        auto& binding = joints_[i];
        if (!binding.entity)
            throw std::runtime_error("RobotTransformAdapter: robot Scene has been removed");
        binding.entity.SetLocalRotation(glm::quat{glm::normalize(rotation)});
    }
}

} // namespace grasplink::viewer::robotics
