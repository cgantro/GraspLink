#include "robot/RobotJointController.h"

#include <glm/gtx/quaternion.hpp>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace 
{

constexpr float kAxisEpsilon = 0.000001F;
/*
    GLB에 들어 있는 실제 Joint Node 이름.

    현재 모델 hierarchy:
        J1
        J2
        ...
        J6
*/
const std::array<const char*, RobotJointController::JointCount> kJointNames{"J1", "J2","J3", "J4","J5", "J6"};
} // namespace 

RobotJointController::RobotJointController(const Entity& robotRoot){
    if(!robotRoot) throw std::runtime_error("RobotJointController: invalid robot root");

    /* 
        계층 구조를 DFS로 탐색한다.
        Joint가 Mesh를 가질 필요는 없다.
        현재 GLB 파일에서는 Joint Node 자체가 Pivot 역할을 하고, Link Mesh가 자식으로 존재한다.
        따라서 Joint Node를 회전 시키면 하위 Link와 Joint도 계층 구조를 따라 회전한다.
    */
    for(std::size_t i = 0; i<JointCount; i++){
        Entity joint  = robotRoot.FindChildByNameRecursive(kJointNames[i]);
        if(!joint)  throw std::runtime_error(std::string("RobotJointController: joint not found: ")+kJointNames[i]);

        joints_[i].entity = joint;

        /*
            현재 Entity는 Rotation이 Euler radians

            쿼터니언으로 변환 후, bind Rotation으로 보존

            이후 SetJointPosition은 항상 이 bind Rotation을 기준으로 절대 각 계산
        */
        const glm::vec3 bindEuler = joint.GetLocalRotation();
        joints_[i].bindRotation = glm::normalize(glm::quat(bindEuler));

        joints_[i].position = 0.0F;
    }
}

void RobotJointController::SetJointPosition(std::size_t jointIndex, float positionRadians, const glm::vec3& localAxis){
    if(jointIndex >= JointCount) throw std::out_of_range("RobotJointController: invalid joint index");

    const float axisLengthSquard = glm::dot(localAxis, localAxis);

    if(axisLengthSquard < kAxisEpsilon) throw std::invalid_argument("RobotJointController: joint axis is zero");

    JointBinding& joint = joints_[jointIndex];

    // Axis는 반드시 단위 벡터
    const glm::vec3 axis = glm::normalize(localAxis);

    // Joint의 순수 회전량 
    // positionRadians = 30도일 때, angleAxis(30deg, axis) 
    const glm::quat jointRotation = glm::angleAxis(positionRadians,axis);

    /*
        원래 자세를 없애는 것이 아닌, 원래 자세 x Joint 회전으로 합성한다.

        axis는 Joint의 Local 좌표의 기준으로 해석한다.
        
        매번 현재 rotation에 angle을 더하지 않고 bindRotation에서 다시 계산 하는 이유
            rotation += delta 식으로 누적 -> 프레임마다 오차 누적
        positionRadians을 절대 Joint position으로 취급
    */
    const glm::quat localRotation = glm::normalize(joint.bindRotation * jointRotation);

    // ECS가 오일러 각을 사용하므로 다시 변환
    // 향후 Transform Component 자체를 쿼터니언으로 바꾼다면 없애도 됨
    const glm::vec3 localEuler = glm::eulerAngles(localRotation);

    joint.entity.SetLocalRotation(localEuler);
    joint.position = positionRadians;
}

void RobotJointController::ResetJoint(std::size_t jointIndex){
    if(jointIndex >= JointCount) throw std::out_of_range("RobotJointController: invalid joint index");

    JointBinding& joint = joints_[jointIndex];

    // 저장해둔 bind Rotation으로 복원
    joint.entity.SetLocalRotation(glm::eulerAngles(joint.bindRotation));
    joint.position = 0.0F;
}

void RobotJointController::ResetAll(){
    for(std::size_t i = 0; i < JointCount; i++){
        ResetJoint(i);
    }
}

Entity RobotJointController::GetJointEntity(std::size_t jointIndex) const{
    if(jointIndex >= JointCount) throw std::out_of_range("RobotJointController: invalid joint index");

    return joints_[jointIndex].entity;
}

float RobotJointController::GetJointPosition(std::size_t jointIndex) const{
    if(jointIndex >= JointCount) throw std::out_of_range("RobotJointController: invalid joint index");

    return joints_[jointIndex].position;
}