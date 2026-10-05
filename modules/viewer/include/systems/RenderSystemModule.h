#pragma once

#include <flecs.h>

/**
 * @brief ECS의 렌더 component를 OpenGL Renderer 호출로 연결한다.
 * @details World transform, MeshFilter, MeshRenderer를 가진 Entity 중 보이는 항목을 모아 같은 목록을 Shadow Pass와 Main Pass에 전달한다.
 * Entity 선택과 component 조회는 이 모듈이 맡고, GL framebuffer·shader·texture 상태와 draw 호출은 Renderer가 맡는다.
 */
struct RenderSystemModule
{
public:
    /**
     * @brief Flecs 렌더 system을 등록한다.
     * @param world RenderSystemModule과 RenderSystem을 등록할 Flecs world.
     */
    explicit RenderSystemModule(flecs::world& world);

private:
    void RegisterSystem(flecs::world& world);
};
