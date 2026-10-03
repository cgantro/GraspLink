#include "robot/RobotJointController.h"

#include <glm/gtx/quaternion.hpp>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace
{
constexpr float kAxisEpsilon = 0.000001F;

// 정규화된 runtime GLB에서 Controller가 찾는 논리 Joint Node 이름.
const std::array<const char*, RobotJointController::JointCount> kJointNames{
    "J1", "J2", "J3", "J4", "J5", "J6"};
} // namespace

RobotJointController::RobotJointController(const Entity& robotRoot)
{
    if (!robotRoot)
        throw std::runtime_error("RobotJointController: invalid robot root");

    /*
        Joint Node 자체는 Mesh가 없어도 된다. 이 Node가 pivot이고 Link/다음 Joint가 자식이므로
        Joint local transform을 변경하면 Flecs hierarchy를 통해 모든 자식 World Transform이 따라 바뀐다.
    */
    for (std::size_t i = 0; i < JointCount; ++i)
    {
        Entity joint = robotRoot.FindChildByNameRecursive(kJointNames[i]);
        if (!joint)
            throw std::runtime_error(
                std::string("RobotJointController: joint not found: ") + kJointNames[i]);

        joints_[i].entity = joint;

        /*
            ECS Rotation은 Euler radian이지만 회전 합성은 quaternion으로 한다.
            GLB bind rotation을 quaternion으로 보존해야 J1/J6 같은 non-identity 기준 자세를 잃지 않는다.
        */
        const glm::vec3 bindEuler = joint.GetLocalRotation();
        joints_[i].bindRotation = glm::normalize(glm::quat(bindEuler));
        joints_[i].position = 0.0F;
    }
}

void RobotJointController::SetJointPosition(
    std::size_t jointIndex,
    float positionRadians,
    const glm::vec3& localAxis)
{
    if (jointIndex >= JointCount)
        throw std::out_of_range("RobotJointController: invalid joint index");

    const float axisLengthSquared = glm::dot(localAxis, localAxis);
    if (axisLengthSquared < kAxisEpsilon)
        throw std::invalid_argument("RobotJointController: joint axis is zero");

    JointBinding& joint = joints_[jointIndex];
    const glm::vec3 axis = glm::normalize(localAxis);

    /*
        angleAxis(theta, axis)는 "axis 주위로 theta만큼 회전"하는 quaternion을 만든다.
        positionRadians를 delta 누적값이 아니라 bind pose 기준 절대 관절 위치로 취급한다.
    */
    const glm::quat jointRotation = glm::angleAxis(positionRadians, axis);
    const glm::quat localRotation = glm::normalize(joint.bindRotation * jointRotation);

    // TODO(FUTURE): 여기에서 HCR12A_LIMITS[jointIndex]로 position을 clamp/reject한다.
    // TODO(FUTURE): target을 즉시 적용하지 않고 Fixed Control Loop에서 max velocity/acceleration을 적용한다.
    // TODO(FUTURE): localAxis는 호출 인자가 아니라 검증된 JointDefinition에 고정한다.

    // 현재 Transform Component가 Euler를 저장하므로 최종 quaternion을 다시 Euler로 변환한다.
    const glm::vec3 localEuler = glm::eulerAngles(localRotation);
    joint.entity.SetLocalRotation(localEuler);
    joint.position = positionRadians;
}

void RobotJointController::ResetJoint(std::size_t jointIndex)
{
    if (jointIndex >= JointCount)
        throw std::out_of_range("RobotJointController: invalid joint index");

    JointBinding& joint = joints_[jointIndex];
    joint.entity.SetLocalRotation(glm::eulerAngles(joint.bindRotation));
    joint.position = 0.0F;
}

void RobotJointController::ResetAll()
{
    for (std::size_t i = 0; i < JointCount; ++i)
        ResetJoint(i);
}

Entity RobotJointController::GetJointEntity(std::size_t jointIndex) const
{
    if (jointIndex >= JointCount)
        throw std::out_of_range("RobotJointController: invalid joint index");

    return joints_[jointIndex].entity;
}

float RobotJointController::GetJointPosition(std::size_t jointIndex) const
{
    if (jointIndex >= JointCount)
        throw std::out_of_range("RobotJointController: invalid joint index");

    return joints_[jointIndex].position;
}
