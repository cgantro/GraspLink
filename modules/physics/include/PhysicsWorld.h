#pragma once

#include "PhysicsTypes.h"

#include <memory>

namespace grasplink::physics
{

// Jolt World와 Body를 소유한다. Entity 연결과 Fixed Update 순서는 simulation 모듈 담당.
class PhysicsWorld final
{
public:
    PhysicsWorld();
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    PhysicsWorld(PhysicsWorld&&) = delete;
    PhysicsWorld& operator=(PhysicsWorld&&) = delete;

    // 단위: 고정 시간 간격 [s]. 유한한 양수가 아니면 계산하지 않는다.
    void Step(double fixedDeltaSeconds);

    // 단일 Box를 생성한다. 원점 자세는 World 기준, 크기는 반쪽 길이 [m].
    PhysicsBodyHandle CreateBox(
        const BoxBodyDescription& description);

    // Box/Cylinder/Sphere/ConvexHull을 하나의 Body로 묶는다.
    // Body 원점과 형상 COM의 차이는 Jolt 내부에서 처리한다.
    PhysicsBodyHandle CreateBody(
        const BodyDescription& description);

    // 다른 World 또는 삭제한 Body의 handle은 false.
    [[nodiscard]]
    bool IsBodyValid(
        PhysicsBodyHandle handle) const;

    // 출력: Body 원점의 World 자세. Dynamic 결과를 Entity에 반영할 때 사용한다.
    [[nodiscard]]
    Transform GetBodyTransform(
        PhysicsBodyHandle handle) const;

    // Body 원점의 World 자세를 즉시 바꾼다. Reset/Teleport에 사용한다.
    void SetBodyTransform(
        PhysicsBodyHandle handle,
        const Transform& transform);

    // 입력: Body 원점의 World 목표 자세와 고정 간격 [s]. Kinematic 전용.
    // Jolt가 이번 간격에 목표까지 이동할 속도를 계산한다.
    void MoveKinematic(
        PhysicsBodyHandle handle,
        const Transform& targetTransform,
        double fixedDeltaSeconds);

    // Body를 계산 목록에서 제거한 뒤 해제한다. 잘못된 handle은 무시한다.
    void DestroyBody(
        PhysicsBodyHandle handle);

private:
    // Jolt 타입과 자원은 Impl에서 관리한다.
    struct Impl;

    std::unique_ptr<Impl> impl_;
};

} // namespace grasplink::physics
