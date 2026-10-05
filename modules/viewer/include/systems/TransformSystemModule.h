#pragma once

#include <flecs.h>
#include <glm/glm.hpp>

/**
 * @brief Entity 변환 행렬 계산
 *
 * Local: 부모 기준 위치·회전·크기
 * World: 모든 부모 변환을 반영한 Scene 기준 위치·회전·크기
 * 예: 관절 회전 → 아래 링크의 World 위치·회전 변경
 *
 * Render: Flecs System이 행렬 갱신
 * Physics: 고정 업데이트에서 최신 Local 값으로 World 행렬 계산
 */
class TransformSystemModule
{
public:
    /**
     * @brief Local·World 행렬 계산 System 등록
     * @param world Entity와 Transform Component를 보관하는 Flecs World
     */
    explicit TransformSystemModule(flecs::world& world);

    /**
     * @brief Local 위치·회전·크기를 행렬로 조합
     * @param position 부모 기준 위치
     * @param rotationRadians 부모 기준 Euler 회전 (라디안)
     * @param scale 축별 크기 배율
     * @return 이동·회전·크기 순서의 Local 행렬
     *
     * Render와 Physics에서 같은 계산 순서 사용
     */
    static glm::mat4 ComposeLocalMatrix(
        const glm::vec3& position,
        const glm::vec3& rotationRadians,
        const glm::vec3& scale);

    /**
     * @brief Entity의 현재 Local 값으로 Local 행렬 계산
     * @param entity 변환을 계산할 Entity
     * @return Transform 값이 없거나 Entity가 사라진 경우 기본 행렬
     */
    static glm::mat4 CalculateLocalMatrix(flecs::entity entity);

    /**
     * @brief Entity와 모든 부모의 Local 값을 합쳐 World 행렬 계산
     * @param entity World 행렬을 구할 Entity
     * @return 부모 변환이 반영된 Scene 기준 행렬
     *
     * 이유: Fixed Update가 Render 행렬 갱신보다 먼저 실행될 수 있음
     * 방법: 캐시 대신 현재 Local 값을 부모부터 다시 계산
     */
    static glm::mat4 CalculateWorldMatrix(flecs::entity entity);

private:
    // 매 프레임 전체 계산. 변경 감시자 미사용
    void RegisterObserver(flecs::world& world);

    // Local 행렬 System + 부모를 반영한 World 행렬 System 등록
    void RegisterSystem(flecs::world& world);
};
