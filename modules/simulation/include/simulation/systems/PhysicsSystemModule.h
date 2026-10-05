#pragma once

#include <flecs.h>

#include <memory>

namespace grasplink::physics
{
class PhysicsWorld;
}

namespace grasplink::simulation
{

/**
 * @brief ECS 물리 설정을 Jolt Body에 연결하고 Fixed Update에서 자세를 전달한다.
 * @details RigidBody와 Colliders를 모두 가진 Entity만 Query 대상이다. 생성 시 필요한 Local
 * Position/Rotation/Scale과 갱신된 World TransformMatrix도 있어야 Body를 만들 수 있다. 누락된
 * 위치·회전·크기 pair가 있는 grouping Entity는 Body 대상이 아니며 부모 좌표만 자식 World 자세에
 * 반영한다. 물리 Entity는 Dynamic 조상을 둘 수 없고, 물리 계층은 단위 scale을 사용한다.
 * Static Environment는 자기 Local scale만 렌더링 용도로 허용하며 collider 치수에는 적용하지 않는다.
 * Dynamic은 SceneRoot 또는 항등 grouping 조상만 허용해 World≈Local로 다룬다.
 * Physics가 정한 결과를 Local Position/Rotation에 직접 기록하고 기존 Scale은 유지한다.
 *
 * TwoF85 설정 helper가 만드는 본체와 여섯 관절의 Kinematic proxy도 이 모듈이 동기화한다.
 * 프록시는 원본 ECS 관절의 자식이므로 현재 관절 계층에서 계산된 World 자세를 사용한다.
 * 팔 FK에 따른 장착부 이동도 부모 계층으로 전파된다. 그리퍼 상태·관절 계산은 Robotics와 앱의 책임이며,
 * 이 모듈은 그 결과의 물리 동기화를 맡는다. 접촉 시 개폐 정지·파지는 구현하지 않는다.
 */
class PhysicsSystemModule final
{
public:
    /**
     * @brief Flecs World와 PhysicsWorld에 Body 연결 observer를 설치한다.
     * @param world 설정 Entity와 observer가 살아 있는 Flecs World.
     * @param physicsWorld Jolt Body를 실제로 소유하고 해제하는 World.
     * @details 두 참조 대상은 이 모듈보다 오래 살아야 한다. 이미 설정된 Entity도 생성 시 한 번
     * 탐색해 연결한다. 반환 후 첫 Step 전까지는 OnSet 설정이 Body 생성 예약으로 남는다.
     */
    PhysicsSystemModule(flecs::world& world, grasplink::physics::PhysicsWorld& physicsWorld);

    /** @brief 연결 Body를 observer가 유효할 때 제거한 뒤 this를 참조하는 observer를 해제한다. */
    ~PhysicsSystemModule();

    PhysicsSystemModule(const PhysicsSystemModule&) = delete;
    PhysicsSystemModule& operator=(const PhysicsSystemModule&) = delete;

    /**
     * @brief 한 고정 시간 간격의 ECS와 Jolt 자세를 동기화한다.
     * @param fixedDeltaSeconds 시뮬레이션 간격 [s]. 유한한 양수만 처리한다.
     * @details 호출 전 Controller/FK가 Local pose를 갱신하고 TransformSystem이 World 행렬을
     * 계산해야 한다. PrePhysicsSync가 예약 Body 재구성과 Scene-driven 자세 전달을 처리하고,
     * Jolt 1회 계산 뒤 PostPhysicsSync가 Physics-driven Dynamic 자세를 Local에 직접 반영한다. 호출자는
     * 이후 TransformSystem을 다시 실행해 다음 렌더링/고정 간격에 쓸 World 행렬을 갱신한다.
     * Static·Kinematic은 Scene, Dynamic은 Physics가 자세를 정한다. 같은 Scene 목표는 다시 전달하지
     * 않지만 Kinematic이 실제 목표에 도달할 때까지 이동을 계속하며 도달 후 잔류 속도를 한 번 제거한다.
     * Dynamic의 비항등 조상은 지원하지 않는다. collider에 scale이 전파되지 않으므로 Static Environment
     * 자신의 시각 scale 외에는 Entity와 모든 scale을 가진 조상에 단위 scale이 필요하다.
     */
    void Step(double fixedDeltaSeconds);

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

}
