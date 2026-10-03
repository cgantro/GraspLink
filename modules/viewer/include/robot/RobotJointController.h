#pragma once

#include "Entity.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstddef>

/**
 * @brief HCR-12A J1~J6 Entity에 관절 각도를 시각적으로 적용하는 Viewer-side Controller.
 *
 * @details
 * GLB Joint Node의 local transform은 단순 초기 자세가 아니라 자식 Link 좌표계의 기준이 될 수 있다.
 * 따라서 joint angle을 Euler 값으로 덮어쓰지 않고 다음 순서로 적용한다.
 *
 * bindRotation * jointDeltaRotation
 *
 * 현재 클래스의 책임은 "관절 상태 -> Flecs Transform" 연결이다.
 * FK/IK 계산, 경로 계획, 물리 제어는 별도 계층에서 담당해야 한다.
 *
 * @warning 정규화된 HCR-12A asset에서 J1/J6는 non-identity bind rotation을 가진다.
 *          bind rotation을 제거하면 모델 기준축이 깨질 수 있다.
 * @todo [FUTURE] localAxis를 호출자가 매번 전달하지 않고 검증된 J1~J6 JointDefinition에 고정한다.
 * @todo [FUTURE] docs/HCR12A_2F85_simulation_specs.md의 angle/velocity limit을 적용한다.
 * @todo [FUTURE] Fixed Control Loop가 도입되면 target position과 actual position을 분리한다.
 */
class RobotJointController
{
public:
    static constexpr std::size_t JointCount = 6;

    /**
     * @brief robotRoot 아래에서 J1~J6를 찾아 bind rotation을 저장한다.
     * @param robotRoot PrefabFactory가 생성한 HCR-12A hierarchy의 root Entity.
     * @throws std::runtime_error root가 유효하지 않거나 J1~J6를 찾지 못한 경우.
     */
    explicit RobotJointController(const Entity& robotRoot);

    /**
     * @brief 하나의 Joint를 bind pose 기준 절대 각도로 설정한다.
     * @param jointIndex 0=J1, 1=J2, ... 5=J6.
     * @param positionRadians bind pose 기준 관절 각도(radian).
     * @param localAxis Joint local coordinate 기준 회전축.
     * @throws std::out_of_range jointIndex가 범위를 벗어난 경우.
     * @throws std::invalid_argument localAxis의 길이가 0에 가까운 경우.
     */
    void SetJointPosition(
        std::size_t jointIndex,
        float positionRadians,
        const glm::vec3& localAxis);

    /** @brief 지정 Joint를 GLB bind pose로 복원한다. */
    void ResetJoint(std::size_t jointIndex);

    /** @brief J1~J6를 모두 GLB bind pose로 복원한다. */
    void ResetAll();

    /** @brief 디버깅/FK 검증을 위해 실제 Flecs Joint Entity를 반환한다. */
    Entity GetJointEntity(std::size_t jointIndex) const;

    /** @brief Controller에 기록된 관절 위치를 radian으로 반환한다. */
    float GetJointPosition(std::size_t jointIndex) const;

private:
    /** @brief 하나의 Joint Entity와 GLB 기준 회전을 함께 보관하는 내부 binding. */
    struct JointBinding
    {
        Entity entity;
        glm::quat bindRotation{1.0F, 0.0F, 0.0F, 0.0F};
        float position = 0.0F;
    };

    std::array<JointBinding, JointCount> joints_;
};
