#pragma once

#include <glm/glm.hpp>

/**
 * @file RobotComponents.h
 * @brief Viewer/Flecs 안에서 로봇 관련 Entity를 표시하기 위한 간단한 ECS Component.
 *
 * @details
 * ECS(Entity Component System)는 객체의 데이터를 작은 Component로 나눠 Entity에 붙이는 구조다.
 * 여기의 Component는 화면 표현 편의를 위한 데이터이며, 로봇 모델 사양의 source of truth는 아니다.
 */

/**
 * @brief Viewer/ECS에 남아 있는 회전관절 표현용 Component.
 *
 * @details
 * 용어:
 * - Joint: 로봇 팔이 회전하는 관절.
 * - Axis: 어느 방향의 선을 중심으로 회전하는지 나타내는 방향벡터.
 * - Angle: 현재 관절이 기준자세에서 얼마나 회전했는지 나타내는 값.
 * - Joint limit: 관절이 허용되는 최소/최대 회전 범위.
 *
 * 이 구조체는 Viewer 쪽 보조 표현일 뿐 HCR-12A의 authoritative specification이 아니다.
 * 실제 이름/축/limit/max velocity는 `robotics::models::RobotSpecification`과 model별 header를 따른다.
 */
struct RobotJoint
{
    /** @brief Joint local frame 기준 회전축. 기본 +Z는 placeholder이며 실제 모델축은 RobotSpecification을 사용한다. */
    glm::vec3 axis{0.0F, 0.0F, 1.0F};

    /** @brief 허용 최소 관절각 [rad]. */
    float minAngle = 0.0F;

    /** @brief 허용 최대 관절각 [rad]. */
    float maxAngle = 0.0F;

    /** @brief 현재 화면/상태에서의 관절각 [rad]. */
    float angle = 0.0F;
};

/**
 * @brief Robot의 말단장치/공구 기준 Entity를 표시하는 값 없는 ECS tag.
 *
 * @details
 * End Effector는 로봇 팔의 가장 끝에서 실제 작업을 수행하는 부분을 뜻한다.
 * 예: gripper, 용접 torch, suction cup.
 *
 * 이 tag 자체는 위치/방향을 저장하지 않는다.
 * 실제 pose는 Entity Transform 또는 RobotState::tcpPose에 존재한다.
 */
struct EndEffector
{
};
