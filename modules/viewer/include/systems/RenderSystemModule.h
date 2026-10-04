#pragma once

#include <flecs.h>

/**
 * @brief 렌더 가능한 Flecs Entity를 조회해 Renderer의 shadow/main pass로 전달하는 ECS module.
 *
 * @details
 * 이 System은 Component를 조합해 "무엇을 그릴지"만 결정하고 실제 OpenGL API 호출은 Renderer에 위임한다.
 * 주요 입력은 다음과 같다.
 *
 * - MeshFilter: GPU Mesh와 index range
 * - MeshRenderer: Shader/Material/visibility
 * - `(TransformMatrix, World)`: object local -> world matrix
 * - RenderContext: Renderer/Camera non-owning pointer
 *
 * Camera의 View/Projection matrix와 Entity의 World model matrix를 Renderer에 전달하므로 최종 vertex 변환은
 * 일반적으로 `Projection * View * Model * localPosition` 순서의 shader 연산으로 이어진다.
 * 현재 controller-ready HCR asset의 model/world translation은 meter [m]지만 RenderSystem 자체는 단위 변환을 하지 않는다.
 *
 * @todo [FUTURE] frustum culling, render queue, transparent sorting이 필요해지면 Entity 선택/정렬 단계를 이 계층에 추가한다.
 */
struct RenderSystemModule
{
public:
    /**
     * @brief World에 shadow/main rendering query/system을 등록한다.
     * @param world RenderContext와 render/transform Component가 존재하는 Flecs World.
     */
    explicit RenderSystemModule(flecs::world& world);

private:
    /**
     * @brief 실제 Flecs rendering query/callback을 World에 등록한다.
     * @param world system을 등록할 Flecs World.
     */
    void RegisterSystem(flecs::world& world);
};
