#pragma once

/**
 * @brief 시각 Mesh와 별도로 만든 로봇 arm 충돌 proxy Entity를 식별하는 ECS marker.
 * @details 위치·회전은 RobotPhysicsAdapter가 FK 결과로 설정하고, RigidBody와 Colliders가 실제 물리 구성을 제공한다.
 * 이 marker 자체는 GLB mesh나 Jolt Body를 소유하지 않으며 충돌 형상을 만들거나 동작을 변경하지 않는다.
 */
struct RobotCollisionProxy {};
