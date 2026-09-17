#pragma once

#include <cstdint>

/*
    ============================================================================
    RenderComponents
    ============================================================================

    Entity가 "무엇을 어떻게 렌더링할지" 표현하는 ECS 데이터.

    실제 OpenGL Draw Call은 Component가 수행하지 않는다.

        Component
            = 데이터

        RenderSystem
            = 어떤 Entity를 그릴지 결정

        Renderer
            = 실제 OpenGL 처리
*/


/*
    MeshFilter
    ============================================================================

    Entity가 사용할 Geometry Resource를 나타낸다.

    이후 GLB/AssetManager가 추가되면:

        meshID
            ↓
        AssetManager::GetMesh(meshID)
            ↓
        Mesh
            ├─ VAO
            ├─ VBO
            └─ IBO

    흐름이 된다.
*/
struct MeshFilter
{
    std::uint32_t meshID = 0;
};


/*
    MeshRenderer
    ============================================================================

    Mesh를 어떤 방식으로 그릴지에 관한 정보.

    현재는 최소 데이터만 둔다.

    GLB Material 시스템이 들어오면 materialID가 실제
    Material Resource를 가리키게 된다.
*/
struct MeshRenderer
{
    std::uint32_t materialID = 0;

    bool visible = true;
};