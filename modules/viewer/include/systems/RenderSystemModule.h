#pragma once

#include <flecs.h>

/*
    ============================================================================
    RenderSystemModule
    ============================================================================

    렌더링에 관련된 Flecs System을 등록하는 Module.


    전체 데이터 흐름:

        Entity
          ├─ MeshFilter
          ├─ MeshRenderer
          └─ TransformMatrix(World)

                    ↓

            RenderSystemModule

                    ↓

              RenderContext

              ├─ Renderer
              └─ Camera

                    ↓

                 OpenGL


    중요한 책임 분리:

    TransformSystemModule
        -> "어디에 있는가?" 계산

    RenderSystemModule
        -> "무엇을 그릴 것인가?" 결정

    Renderer
        -> "실제로 어떻게 OpenGL로 그릴 것인가?"


    따라서 RenderSystem에서는:

        ParentWorld * Local

    같은 Transform 계산을 절대 하지 않는다.

    이미 TransformSystem이 계산한

        (TransformMatrix, World)

    를 읽기만 한다.
*/
struct RenderSystemModule
{
public:
    explicit RenderSystemModule(flecs::world& world);

private:
    void RegisterSystem(flecs::world& world);
};