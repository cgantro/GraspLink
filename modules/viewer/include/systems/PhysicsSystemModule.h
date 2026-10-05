#pragma once

#include <flecs.h>

#include <memory>

namespace grasplink::physics
{
class PhysicsWorld;
}

/**
 * @brief Entity의 물리 설정을 Jolt Body와 연결
 *
 * 설정: `RigidBody` + `BoxCollider`가 있을 때 Jolt Body 생성
 * handle: 내부 보관. Entity 생성 코드는 설정 Component만 사용
 *
 * 고정 Step 흐름:
 * - Kinematic: Controller가 바꾼 Entity 자세를 Jolt에 전달
 * - Jolt: 충돌과 Dynamic Body 이동 계산
 * - Dynamic: Jolt 결과를 Entity에 반영
 * - Static: 생성 때 자세를 읽고 고정
 *
 * 역할: Flecs 설정·좌표 변환·Body 수명 연결
 * PhysicsWorld: Jolt 계산과 Body 저장
 * 수명: Flecs World와 PhysicsWorld를 이 모듈보다 나중에 파괴
 */
class PhysicsSystemModule final
{
public:
    /**
     * @brief 물리 설정 감시 기능 등록, 기존 설정 Entity의 Body 생성
     * @param world Entity와 Component를 보관하는 Flecs World. 이 모듈보다 긴 수명
     * @param physicsWorld Jolt Body 생성·계산 담당. 이 모듈보다 긴 수명
     */
    PhysicsSystemModule(flecs::world& world, grasplink::physics::PhysicsWorld& physicsWorld);

    /** @brief Jolt Body와 설정 감시 기능 정리 */
    ~PhysicsSystemModule();

    PhysicsSystemModule(const PhysicsSystemModule&) = delete;
    PhysicsSystemModule& operator=(const PhysicsSystemModule&) = delete;

    /**
     * @brief 고정 간격으로 Physics 한 번 진행
     * @param fixedDeltaSeconds 계산 간격(초). FixedControlLoop가 전달하는 양수 값
     *
     * 호출 순서: Controller가 Entity 자세 변경 후 호출
     * 화면 FPS와 분리된 고정 시간 간격
     */
    void Step(double fixedDeltaSeconds);

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};
