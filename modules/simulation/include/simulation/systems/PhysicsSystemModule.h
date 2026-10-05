#pragma once

#include <flecs.h>

#include <memory>

namespace grasplink::physics
{
class PhysicsWorld;
}

namespace grasplink::simulation
{

// Flecs 설정을 Jolt Body에 연결하고 Fixed Update에서 자세를 양방향으로 전달한다.
class PhysicsSystemModule final
{
public:
    // world와 physicsWorld는 이 객체보다 오래 살아야 한다. Body는 physicsWorld가 소유한다.
    PhysicsSystemModule(flecs::world& world, grasplink::physics::PhysicsWorld& physicsWorld);

    // Body 연결을 먼저 제거하고 this를 참조하는 observer를 해제한다.
    ~PhysicsSystemModule();

    PhysicsSystemModule(const PhysicsSystemModule&) = delete;
    PhysicsSystemModule& operator=(const PhysicsSystemModule&) = delete;

    // Controller와 World 자세 갱신 뒤 호출: 설정 반영 -> Kinematic -> Jolt -> Dynamic Local.
    // 간격은 s. Dynamic 조상은 미지원. Static Environment 자신의 시각 scale 외에는 단위 scale 필요.
    void Step(double fixedDeltaSeconds);

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

}
