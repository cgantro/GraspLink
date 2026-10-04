#pragma once

#include <glm/glm.hpp>

/**
 * @brief Viewer/ECS에 남아 있는 회전 관절 표현용 Component.
 *
 * @details
 * 이 구조체는 현재 graphics/ECS 편의를 위한 데이터 형식이며 HCR-12A의 authoritative model specification은 아니다.
 * Robot의 이름/축/angle limit/max velocity source of truth는
 * `robotics::models::RobotSpecification` 및 model별 header(Hcr12a.h 등)다.
 *
 * 단위/좌표 규칙:
 * - axis: Joint local frame 기준 무차원 회전축
 * - minAngle/maxAngle/angle: radian [rad]
 *
 * @todo [FUTURE] 실제 사용처가 없거나 RobotSpecification과 상태가 중복되면 이 Component를 제거하거나
 *       Viewer cache 역할로만 명확히 제한한다.
 */
struct RobotJoint
{
    /** @brief Joint local frame 기준 회전축. 기본값은 +Z. 실제 모델축은 RobotSpecification을 사용한다. */
    glm::vec3 axis{0.0F, 0.0F, 1.0F};

    /** @brief 허용 최소 각도 [rad]. */
    float minAngle = 0.0F;

    /** @brief 허용 최대 각도 [rad]. */
    float maxAngle = 0.0F;

    /** @brief 현재 표시/상태 각도 [rad]. */
    float angle = 0.0F;
};

/**
 * @brief Robot의 TCP/말단장치 Entity를 식별하기 위한 값 없는 ECS tag.
 *
 * @note 위치/방향 값 자체를 저장하지 않는다. 실제 pose는 Entity의 Transform 또는 robotics RobotState::tcpPose에 존재한다.
 */
struct EndEffector
{
};
