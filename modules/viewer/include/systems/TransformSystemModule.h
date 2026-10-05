#pragma once

#include <flecs.h>
#include <glm/glm.hpp>

/**
 * @brief Entity의 Local TRS를 Local/World 행렬로 계산하는 변환 모듈이다.
 * @details flecs 모듈만 등록하며 자체 자동 실행 System은 만들지 않는다. Local 값의 작성은 Controller,
 * Physics, Scene logic의 책임이다. 호출자는 해당 로직이 값을 바꾼 뒤 Physics가 읽기 전과 렌더 단계 전에
 * `UpdateWorldTransforms`를 명시적으로 호출해야 한다. World 행렬은 캐시된 현재 상태이지 자동 추적 값이 아니다.
 */
class TransformSystemModule
{
public:
    /** @brief flecs World에 모듈 표식을 등록한다. 변환 갱신은 호출자가 수행한다. */
    explicit TransformSystemModule(flecs::world& world);

    /**
     * @brief 부모 기준 TRS를 Local 행렬로 합성한다.
     * @param position 부모 좌표계 위치 [m].
     * @param rotationRadians GLM Euler X/Y/Z 회전 [rad].
     * @param scale 축별 크기 배율. 단위 없음.
     * @return 열 벡터 기준 `T * R * S` 행렬. 점에는 scale, rotation, translation 순서로 적용된다.
     */
    static glm::mat4 ComposeLocalMatrix(
        const glm::vec3& position,
        const glm::vec3& rotationRadians,
        const glm::vec3& scale);

    /**
     * @brief 완전한 Local TRS Entity에서 시작해 조상 순서로 World 행렬을 갱신한다.
     * @details 시작 Entity는 Position/Rotation/Scale의 Local pair와 TransformMatrix의 Local/World pair를
     * 모두 가져야 한다. 재귀로 방문한 grouping 부모는 완전한 TRS가 없으면 항등 Local로 조상 변환만 전달하며,
     * 부분 TRS도 합성하지 않는다. 방문한 부모와 SceneRoot에는 파생 World 행렬을 저장하지만 Local TRS를
     * 새로 붙이지 않는다. SceneRootTag는 Scene 소유권 표시이며 공간 변환의 입력이 아니다.
     * 호출 한 번 안에서만 부모 결과를 공유하는 캐시를 사용하므로 다음 호출은 변경된 Local 값을 다시 읽는다.
     * 이 함수는 캐시를 갱신할 뿐 Entity의 TRS를 움직이지 않는다.
     */
    static void UpdateWorldTransforms(flecs::world& world);
};
