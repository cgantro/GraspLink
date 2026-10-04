#pragma once

#include <flecs.h>

/**
 * @brief 렌더 가능한 Entity를 조회해 Renderer의 shadow/main pass로 전달하는 Flecs module.
 *
 * @details System은 MeshFilter, MeshRenderer, World Transform 같은 Component를 조합해
 * "무엇을 그릴지" 선택한다. 실제 OpenGL 호출은 Renderer가 담당한다.
 *
 * @todo [FUTURE] frustum culling이나 render queue가 필요해지면 Entity 선택/정렬 단계를 이 계층에 추가한다.
 */
struct RenderSystemModule
{
public:
    /** @brief World에 렌더링 System을 등록한다. */
    explicit RenderSystemModule(flecs::world& world);

private:
    /** @brief main/shadow rendering query와 callback을 World에 등록한다. */
    void RegisterSystem(flecs::world& world);
};
