#pragma once

#include "PhysicsTypes.h"

#include <memory>

namespace grasplink::physics
{

/*
 * Jolt Physics World를 GraspLink용 API로 감싸는 클래스.
 *
 * 외부에서는 JPH::PhysicsSystem, JPH::BodyID 같은
 * Jolt 타입을 직접 사용하지 않는다.
 *
 * 이 클래스가 담당하는 것:
 *
 * 1. Physics World 초기화
 * 2. Body 생성 / 삭제
 * 3. Physics simulation step
 * 4. Body Transform 조회 / 변경
 * 5. Kinematic Body 이동
 */
class PhysicsWorld final
{
public:
    PhysicsWorld();
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    PhysicsWorld(PhysicsWorld&&) = delete;
    PhysicsWorld& operator=(PhysicsWorld&&) = delete;

    /*
     * Physics simulation을 fixedDeltaSeconds만큼 진행한다.
     *
     * FixedControlLoop 내부에서 호출할 예정이다.
     *
     * 예:
     *
     * fixedDeltaSeconds = 0.004
     *
     * -> Physics World를 4ms 진행
     */
    void Step(double fixedDeltaSeconds);

    /*
     * Box 형태의 Physics Body를 생성한다.
     *
     * description.motionType에 따라
     *
     * Static
     * Kinematic
     * Dynamic
     *
     * 중 하나로 생성된다.
     */
    PhysicsBodyHandle CreateBox(
        const BoxBodyDescription& description);

    /*
     * Handle이 현재 Physics World에서
     * 실제 존재하는 Body를 가리키는지 확인한다.
     */
    [[nodiscard]]
    bool IsBodyValid(
        PhysicsBodyHandle handle) const;

    /*
     * Physics가 가지고 있는 Body의 현재 World Transform을 반환한다.
     *
     * Dynamic Body를 Entity에 반영할 때 사용한다.
     *
     * Physics
     *    ↓
     * GetBodyTransform()
     *    ↓
     * Entity Transform
     */
    [[nodiscard]]
    Transform GetBodyTransform(
        PhysicsBodyHandle handle) const;

    /*
     * Body의 위치와 회전을 즉시 변경한다.
     *
     * 이동 과정을 계산하는 함수가 아니라
     * 해당 위치로 바로 옮기는 함수다.
     *
     * 주 사용처:
     *
     * - 초기 위치 지정
     * - Simulation Reset
     * - 강제 Teleport
     */
    void SetBodyTransform(
        PhysicsBodyHandle handle,
        const Transform& transform);

    /*
     * Kinematic Body를 targetTransform 방향으로 이동시킨다.
     *
     * Robot Link처럼 Physics가 스스로 움직이는 게 아니라
     * Controller가 목표 위치를 정하는 Body에 사용한다.
     *
     * Jolt가 fixedDeltaSeconds 동안 목표 위치까지 이동하도록
     * 필요한 velocity를 계산한다.
     */
    void MoveKinematic(
        PhysicsBodyHandle handle,
        const Transform& targetTransform,
        double fixedDeltaSeconds);

    /*
     * Physics World에서 Body를 제거하고 해제한다.
     */
    void DestroyBody(
        PhysicsBodyHandle handle);

private:
    /*
     * Jolt 전용 타입은 모두 Impl 안에 숨긴다.
     *
     * 따라서 이 헤더를 include하는 쪽에서는
     * Jolt header가 필요하지 않다.
     */
    struct Impl;

    std::unique_ptr<Impl> impl_;
};

} // namespace grasplink::physics