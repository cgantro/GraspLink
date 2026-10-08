#pragma once

#include <flecs.h>
#include "physics/PhysicsTypes.h"

#include <memory>

namespace grasplink::physics
{
class PhysicsWorld;
}

namespace grasplink::diagnostics
{
class Logger;
}

namespace grasplink::simulation
{

/**
 * @brief ECS 설정을 Jolt 물리 Body로 만들고 매 고정 간격마다 장면과 물리 상태를 동기화한다.
 * @details Body는 Jolt가 위치, 속도, 충돌을 계산하는 물체다.
 * RigidBody와 Colliders가 함께 붙은 Entity만 Body를 만든다.
 * Collider는 접촉 여부를 계산하는 모양이고 화면에 보이는 Mesh와 별도로 지정한다.
 * Entity의 Local 위치와 회전은 바로 위 부모 기준이며 World 변환은 부모 계층을 합친 장면 전체 기준이다.
 * Body 생성에는 Entity 자신의 Local Position, Rotation, Scale과 부모까지 반영한 World TransformMatrix가 필요하다.
 * RigidBody 설정만 붙은 grouping Entity는 Body가 없지만 부모 위치와 회전은 자식의 World 변환에 반영된다.
 * 충돌 모양에 Entity scale을 곱하지 않는다.
 * Static Environment에만 화면 크기를 위한 Local scale을 허용하며 그 외 Entity와 scale을 지정한 조상은 단위 scale이어야 한다.
 * Dynamic Body는 계산한 World 위치·회전을 Entity Local 값에 바로 쓰므로 변환이 있는 조상 아래에서는 사용할 수 없다.
 * Dynamic 부모와 물리 계산되는 자식 Body를 함께 움직이는 규칙도 없다.
 * Static은 장면이 정한 자세에 고정한다.
 * Kinematic은 장면이 준 목표까지의 이동을 Jolt에 전달해 충돌 계산에 참여시킨다.
 * Dynamic은 중력과 충돌 결과로 Jolt가 정한 자세를 장면에 돌려준다.
 * TwoF85 본체와 여섯 관절의 Kinematic 충돌 Body도 각 원본 관절의 World 자세를 목표로 동기화한다.
 * Gripper 관절 상태와 각도 계산은 Robotics와 앱이 맡고 이 모듈은 이미 계산된 자세를 물리에 전달한다.
 * 접촉에 따른 개폐 정지와 파지 연결은 이 Step 뒤 실행되는 GripperGraspAdapter가 담당한다.
 */
class PhysicsSystemModule final
{
public:
        /**
     * @brief Flecs 설정 변화를 관찰해 Jolt Body의 생성과 해제를 연결한다.
     * @param world 설정 Entity와 observer가 살아 있는 Flecs World다.
     * @param physicsWorld 만든 Body를 실제로 소유하고 해제하는 PhysicsWorld다.
     * 두 World는 이 모듈보다 오래 살아야 한다.
     * @param logger 진단 기록을 남길 Logger다. 전달하면 이 모듈보다 오래 살아야 한다.
     * 생성 시 이미 붙어 있는 component도 찾아 연결한다.
     * 새 설정으로 Body를 만들거나 다시 만드는 일은 첫 Step에서 실행한다.
     */
    PhysicsSystemModule(flecs::world& world, grasplink::physics::PhysicsWorld& physicsWorld,
        grasplink::diagnostics::Logger* logger = nullptr);

    /** @brief 연결 Body를 observer가 유효할 때 제거한 뒤 this를 참조하는 observer를 해제한다. */
    ~PhysicsSystemModule();

    PhysicsSystemModule(const PhysicsSystemModule&) = delete;
    PhysicsSystemModule& operator=(const PhysicsSystemModule&) = delete;

        /**
     * @brief Scene이 정한 자세를 Jolt에 보내고 Dynamic 계산 결과를 Entity에 기록한다.
     * @param fixedDeltaSeconds 이번 물리 계산 간격 [s]다. 유한한 양수만 처리한다.
     * @details 호출 전에 Controller와 FK가 관절 자세를 갱신해야 한다.
     * TransformSystem은 부모 Local 변환을 합쳐 최신 World 행렬을 계산한다.
     * Step은 예약한 Body 설정을 적용하고 Static 위치와 Kinematic 목표를 Jolt에 전달한다.
     * 이어서 Jolt를 고정 간격 한 번 계산하고 Dynamic Body의 World 위치·회전을 Entity Local 값에 기록한다.
     * 다음 렌더링에서 이 결과를 사용하려면 호출자가 TransformSystem을 다시 실행해야 한다.
     * 같은 Kinematic 목표를 반복 전달하지 않으며 목표에 도달한 속도는 한 번 정지시킨다.
     * 충돌 모양에는 scale을 적용하지 않으므로 Static Environment의 화면용 scale을 제외하고 관련 Entity와 조상은 단위 scale이어야 한다.
     */
    void Step(double fixedDeltaSeconds);

    /**
     * @brief Scene Entity에 현재 연결된 물리 Body의 비소유 핸들을 반환한다.
     * @details Entity는 Flecs World가 소유한 장면 물체를 가리킨다. 아직 Body가 없거나 Entity가 삭제됐으면 무효 핸들을 반환한다.
     * 설정 변경으로 Body가 재생성될 수 있으므로 adapter는 각 Step에서 다시 조회해야 한다.
     */
    [[nodiscard]] grasplink::physics::PhysicsBodyHandle GetBodyHandle(flecs::entity entity) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

}
