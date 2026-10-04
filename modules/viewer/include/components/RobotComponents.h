#pragma once

#include <glm/glm.hpp>

/**
 * @brief 회전 관절의 축, 제한, 현재 각도를 표현하는 ECS 데이터 모델.
 *
 * @note 각도 단위는 radian이다. 축은 joint local coordinate 기준을 사용해야 한다.
 * @todo [FUTURE] HCR-12A의 검증된 J1~J6 axis/limit/velocity를 JointDefinition으로 통합하고
 *       RobotJointController와 중복되는 상태 표현을 제거한다.
 */
/*
 * [추가 현재 구조/용어 설명]
 * - ECS Component: Entity에 붙는 작은 데이터 조각.
 * - Joint local coordinate: 해당 관절 자신을 기준으로 한 좌표계.
 * - Axis: 회전 중심선의 방향벡터. 예: (0,1,0)은 local +Y축.
 * - Joint limit: 관절이 허용되는 최소/최대 각도 범위.
 * - End Effector/TCP: 로봇 끝단에서 Tool/작업 기준으로 사용하는 말단 위치.
 *
 * [현재 구조 주의]
 * 위 기존 TODO의 RobotJointController는 이후 RobotTransformAdapter/SimRobotController 구조로 개편되었다.
 * 현재 HCR-12A의 authoritative axis/limit/max velocity는
 * robotics::models::RobotSpecification 및 models/hanwha/Hcr12a.h에 있다.
 * 이 RobotJoint Component는 Viewer/ECS 쪽 legacy 표현이므로 robotics model source-of-truth로 사용하지 않는다.
 */
struct RobotJoint
{
    // Joint local frame 기준 회전축. 무차원 방향벡터.
    glm::vec3 axis{0.0F, 0.0F, 1.0F};

    // 허용 최소 각도 [rad].
    float minAngle = 0.0F;

    // 허용 최대 각도 [rad].
    float maxAngle = 0.0F;

    // 현재 표시/상태 각도 [rad].
    float angle = 0.0F;
};

/** @brief 로봇의 TCP/말단장치 Entity를 식별하기 위한 tag Component. */
/* Tag Component는 값을 저장하지 않고 "이 Entity가 어떤 역할인지"만 표시한다. */
struct EndEffector
{
};
