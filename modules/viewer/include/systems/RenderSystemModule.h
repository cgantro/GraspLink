#pragma once

#include <flecs.h>

// World 변환이 계산된 렌더 가능 엔티티를 선택해 Renderer에 전달한다.
struct RenderSystemModule
{
public:
    explicit RenderSystemModule(flecs::world& world);

private:
    void RegisterSystem(flecs::world& world);
};
