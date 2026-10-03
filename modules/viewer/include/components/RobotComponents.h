#pragma once

#include <glm/glm.hpp>

/**
 * @brief 회전 관절의 축, 제한, 현재 각도를 표현하는 ECS 데이터 모델.
 *
 * @note 각도 단위는 radian이다. 축은 joint local coordinate 기준을 사용해야 한다.
 * @todo [FUTURE] HCR-12A의 검증된 J1~J6 axis/limit/velocity를 JointDefinition으로 통합하고
 *       RobotJointController와 중복되는 상태 표현을 제거한다.
 */
struct RobotJoint
{
    glm::vec3 axis{0.0F, 0.0F, 1.0F};
    float minAngle = 0.0F;
    float maxAngle = 0.0F;
    float angle = 0.0F;
};

/** @brief 로봇의 TCP/말단장치 Entity를 식별하기 위한 tag Component. */
struct EndEffector
{
};
