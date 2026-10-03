#pragma once

#include "Entity.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstddef>

/**
 * @brief  GLB로 생성된 HCR-12A의 J1 ~ J6 Entity를 찾아서 각 관절의 회전을 적용하는 Viewer-side Controller.
 * 
 * 1. GLB 관전 노드에는 이미 원래 Local Rotation이 존재 가능
 * 2. 따라서 관전 각을 SetLocalRotation({0,angle,0})처럼 덮어쓰면 자세가 깨질 수 이씅ㅁ
 * 3. 생성 시 원래 회전을 bindRotation으로 보관
 * 4. 실제 Joint 회전 : bindRotation * jointRotation
 *  이 클래스는 "로봇 상태 → 화면의 Joint Entity" 연결 역할만 담당한다.
 */
class RobotJointController{
public:
    static constexpr std::size_t JointCount = 6;
    explicit RobotJointController(const Entity& robotRoot);

    /*
        특정 관절을 절대 각도로 설정
        jointIndex
            0 -> J1
            1 -> J2 ...
        positionRadians : bind pose 기준 회전량(30도 = glm::radians(30.f))
        localAxis: 해당 Joint의 Local 회전 축
            아직은 정확히 모르므로 X/Y/Z 넣으면서 검증해봐야함
    */

    void SetJointPosition(std::size_t jointIndex, float positionRadians, const glm::vec3& localAxis);
    // Joint를 원래 자세로 돌린다.
    void ResetJoint(std::size_t jointIndex);
    // J1~J6 전체를 원래 bind Pose로 되돌린다.
    void ResetAll();
    // 디버깅/추후 FK 검증을 위해 실제 Flecs Joint Entity를 얻는다
    Entity GetJointEntity(std::size_t jointIndex) const;
    // 현재 Controller에 설정된 Joint Position. 단위는 라디안
    float GetJointPosition(std::size_t jointIndex) const;
private:
    struct JointBinding{
        Entity entity;

        /*
            GLB에서 처음 로드 되었을 때 Local Rotation
            Euler를 그대로 저장 X 쿼터니언으로 저장
            회전 합성은 쿼터니언이 안전하기 때문
        */
        glm::quat bindRotation{1.0F,0.0F,0.0F,0.0F};
        float position = 0.0F;
    };

    std::array<JointBinding,JointCount> joints_;
};