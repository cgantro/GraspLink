#pragma once

/**
 * @brief 화면 모델 대신 충돌 계산에 사용할 proxy Entity임을 표시한다.
 * @details Proxy는 원래 시각 메시와 별도로 만든 대리 물체이며 충돌 모양을 가진다.
 * RobotPhysicsAdapter가 FK 결과로 이 Entity의 위치와 회전을 정한다.
 * 움직임 방식은 RigidBody에, 접촉 모양은 Colliders에 따로 기록한다.
 * 이 표식은 GLB 메시나 Jolt Body를 소유하지 않고 자체적으로 충돌 동작을 바꾸지 않는다.
 */
struct RobotCollisionProxy {};
