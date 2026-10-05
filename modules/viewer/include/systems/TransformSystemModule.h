#pragma once

#include <flecs.h>
#include <glm/glm.hpp>

// Local TRS에서 행렬을 계산한다. 위치 결정은 Controller/Physics/Scene logic의 책임이다.
class TransformSystemModule
{
public:
    // 모듈만 등록한다. 자동 실행 System은 없으며 ViewerApp이 fixed step 전후와 렌더 전에 직접 호출한다.
    explicit TransformSystemModule(flecs::world& world);

    // 입력: 부모 기준 위치 [m], Euler 회전 [rad], 크기 배율. 점에 크기 → 회전 → 이동 순서로 적용한다.
    static glm::mat4 ComposeLocalMatrix(
        const glm::vec3& position,
        const glm::vec3& rotationRadians,
        const glm::vec3& scale);

    // 완전한 Local TRS와 행렬 Component가 있는 Entity부터 부모를 따라 World 행렬을 갱신한다.
    // 방문한 grouping/SceneRoot도 World 행렬을 받지만 Local TRS는 추가하지 않는다.
    static void UpdateWorldTransforms(flecs::world& world);
};
