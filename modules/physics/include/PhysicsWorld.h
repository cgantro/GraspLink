#pragma once

#include "PhysicsTypes.h"

#include <memory>

namespace grasplink::physics
{

/**
 * @brief Jolt 물리 World와 그 안의 Body 수명을 관리한다.
 * @details
 * Entity와의 연결 및 Fixed Update 호출 순서는 simulation 모듈 책임이다. 각 World는 자체 Jolt
 * PhysicsSystem, 임시 메모리 할당기, 실제 계산에 쓰이는 thread pool을 가진다. Jolt 전역 타입 등록은
 * 살아 있는 World 수를 세어 첫 World에서 준비하고 마지막 World가 끝날 때 해제한다.
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

    /**
     * @brief World를 주어진 시간만큼 한 번 계산한다.
     * @param fixedDeltaSeconds 계산 간격 [s]. 유한한 양수여야 한다.
     * @details 한 호출은 추가 분할 없이 Jolt Update 한 번으로 처리한다. 유효하지 않은 간격은 조용히 무시한다.
     */
    void Step(double fixedDeltaSeconds);

    /** @brief 단일 Box 형상으로 Body를 만든다.
     * @param description World 기준 Body 원점 자세, 반쪽 길이 [m], 이동 방식과 충돌 범주.
     * @return 이 World에서만 유효한 Body 핸들.
     * @throws std::invalid_argument 자세, layer 또는 치수가 유효하지 않은 경우.
     */
    PhysicsBodyHandle CreateBox(
        const BoxBodyDescription& description);

    /**
     * @brief Box/Cylinder/Sphere/ConvexHull 형상을 하나의 Body로 묶는다.
     * @param description 각 형상의 국소 배치와 치수, 그리고 World 기준 Body 원점 자세.
     * @return 이 World에서만 유효한 Body 핸들.
     * @throws std::invalid_argument 비어 있는 형상 목록, 유효하지 않은 자세·layer·치수인 경우.
     * @throws std::runtime_error Jolt가 형상 또는 Body를 만들지 못한 경우.
     * @details 입력 자세와 반환 자세는 모델 Body 원점 기준이다. Jolt는 비대칭 compound의 무게중심(COM)을
     * 내부 계산에 사용하지만, 이 API는 COM 위치 대신 Body 원점을 보고하고 설정한다.
     */
    PhysicsBodyHandle CreateBody(
        const BodyDescription& description);

    /** @brief 핸들이 현재 이 World에 추가된 Body를 가리키는지 확인한다. */
    [[nodiscard]]
    bool IsBodyValid(
        PhysicsBodyHandle handle) const;

    /**
     * @brief Body 원점의 현재 World 자세를 읽는다.
     * @param handle 이 World에서 생성한 유효한 Body 핸들.
     * @return 위치 [m]와 회전. Entity에 물리 계산 결과를 반영할 때 사용한다.
     * @throws std::invalid_argument 핸들이 무효이거나 다른 World 소유인 경우.
     * @details COM이 아니라 Body 원점을 반환하므로 비대칭 형상에서도 모델 기준점을 유지한다.
     */
    [[nodiscard]]
    Transform GetBodyTransform(
        PhysicsBodyHandle handle) const;

    /**
     * @brief Body 원점을 지정한 World 자세로 즉시 설정한다.
     * @param handle 이 World에서 생성한 유효한 Body 핸들.
     * @param transform 목표 위치 [m]와 회전.
     * @throws std::invalid_argument 핸들이 무효이거나 자세 성분이 유한하지 않고 회전이 0인 경우.
     * @details 경로를 따라가는 이동이 아니라 Reset/Teleport용 직접 배치다. 회전은 전달 전에 정규화된다.
     */
    void SetBodyTransform(
        PhysicsBodyHandle handle,
        const Transform& transform);

    /**
     * @brief Kinematic Body가 한 고정 간격 동안 목표 자세로 이동하도록 예약한다.
     * @param handle 이 World의 Kinematic Body 핸들.
     * @param targetTransform 목표 Body 원점의 World 자세, 위치 단위 [m].
     * @param fixedDeltaSeconds 이번 이동 간격 [s], 유한한 양수.
     * @throws std::invalid_argument 핸들, 목표 자세 또는 시간 간격이 유효하지 않은 경우.
     * @throws std::logic_error Body가 Kinematic이 아닌 경우.
     * @details Jolt는 현재 자세와 목표, 간격에서 이동 속도를 계산해 충돌 처리를 수행한다. 즉시 순간이동할
     * 때는 SetBodyTransform을 사용한다.
     */
    void MoveKinematic(
        PhysicsBodyHandle handle,
        const Transform& targetTransform,
        double fixedDeltaSeconds);

    /**
     * @brief 목표에 도달한 Kinematic Body의 선속도·각속도를 0으로 만든다.
     * @details MoveKinematic은 목표에서 이동 속도를 만들며 Step 이후에도 그 속도가 남는다.
     * 목표 전달을 생략하기 전에 한 번 호출해 정지시킨다. 위치·회전은 바꾸지 않는다.
     * @throws std::invalid_argument 핸들이 유효하지 않을 때.
     * @throws std::logic_error Body가 Kinematic이 아닐 때.
     */
    void StopKinematic(PhysicsBodyHandle handle);

    /** @brief Body를 Jolt 계산 목록에서 빼고 해제한다. 무효·타 World 핸들은 무시한다. */
    void DestroyBody(
        PhysicsBodyHandle handle);

private:
    // Jolt 타입과 자원은 Impl에서 관리한다.
    struct Impl;

    std::unique_ptr<Impl> impl_;
};

} // namespace grasplink::physics
