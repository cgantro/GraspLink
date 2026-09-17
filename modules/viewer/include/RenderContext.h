#pragma once

class Renderer;
class Camera;

/*
    ============================================================================
    RenderContext
    ============================================================================

    RenderSystem이 렌더링 작업을 수행하기 위해 필요한
    외부 객체들을 묶는다.

    이건 ECS Component라기보다는 "System Context"다.


    Component:

        Entity가 어떤 데이터를 가지고 있는가?

        MeshFilter
        MeshRenderer
        TransformMatrix


    Context:

        RenderSystem이 어떤 Renderer와 Camera를 사용할 것인가?


    Renderer와 Camera의 소유권은 RenderContext에 없다.

    따라서 unique_ptr/shared_ptr가 아니라 non-owning pointer만 보관한다.
*/
struct RenderContext
{
    Renderer* renderer = nullptr;
    Camera* camera = nullptr;
};