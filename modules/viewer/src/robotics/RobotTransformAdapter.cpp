#include "viewer/robotics/RobotTransformAdapter.h"
#include "systems/TransformSystemModule.h"

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
    // RobotSpecification의 joint 이름을 GLB Entity 계층에서 찾아 연결한다.
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

        // 좌표: 관절 중심을 모델 root 기준으로 복원. Scene 배치용 root 변환은 제외.
        glm::mat4 bindInBase = TransformSystemModule::ComposeLocalMatrix(
            joint.GetLocalPosition(), joint.GetLocalRotation(), joint.GetLocalScale());
        Entity parent = joint.GetParent();
        bool followsPreviousJoint = i == 0;
        while (parent && parent != robotRoot)
        {
            // 제한: 관절은 직전 관절의 자손이어야 함. 사이의 link node는 허용.
            if (i > 0 && parent == joints_[i - 1].entity)
                followsPreviousJoint = true;
            bindInBase = TransformSystemModule::ComposeLocalMatrix(
                parent.GetLocalPosition(), parent.GetLocalRotation(), parent.GetLocalScale()) * bindInBase;
            parent = parent.GetParent();
        }
        const glm::vec3 expectedPivot{static_cast<float>(jointSpec.bindPivotMeters.x),
            static_cast<float>(jointSpec.bindPivotMeters.y), static_cast<float>(jointSpec.bindPivotMeters.z)};
        const glm::mat4 expectedBind = glm::translate(glm::mat4{1.0F}, expectedPivot);
        // root를 제외한 bind matrix가 base-frame pivot과 identity orientation/scale에 맞는지 확인한다.
        bool matchesBind = parent == robotRoot && followsPreviousJoint;
        for (int column = 0; column < 4; ++column)
            matchesBind = matchesBind && glm::all(glm::lessThanEqual(
                glm::abs(bindInBase[column] - expectedBind[column]), glm::vec4{1.0e-5F}));
        if (!matchesBind)
            throw std::invalid_argument("RobotTransformAdapter: GLB hierarchy or bind pivot differs from robot specification");

        const glm::vec3 bindEuler = joint.GetLocalRotation();
        if (glm::dot(bindEuler, bindEuler) >
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
    // FK의 joint 순서와 GLB에 연결한 Entity 수가 같아야 한다.
    if (state.jointLocalRotations.size() != joints_.size())
        throw std::invalid_argument("RobotTransformAdapter: joint rotation count mismatch");

    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        // 반영: Local 회전만 바꿔 GLB 계층에 저장된 초기 관절 위치를 유지.
        const auto& pose = state.jointLocalRotations[i];
        // 정밀도: 90도 부근 Euler 변환까지 double을 유지해 float 반올림 오차를 줄임.
        const glm::dquat rotation{pose.w, pose.x, pose.y, pose.z};
        auto& binding = joints_[i];
        if (!binding.entity)
            throw std::runtime_error("RobotTransformAdapter: robot Scene has been removed");
        binding.entity.SetLocalRotation(glm::vec3(glm::eulerAngles(glm::normalize(rotation))));
    }
}

} // namespace grasplink::viewer::robotics
