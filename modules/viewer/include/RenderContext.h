#pragma once

class Renderer;
class Camera;

/**
 * @brief RenderSystem이 Renderer와 Camera에 접근하기 위한 non-owning ECS context.
 *
 * @details
 * 일반 Component가 개별 Entity 상태를 나타내는 것과 달리 RenderContext는 렌더링 System 전체가 공유하는
 * 외부 서비스 객체를 Flecs World에 연결한다. Renderer/Camera의 실제 소유권은 ViewerApp에 있다.
 *
 * 수명 규칙:
 * `ViewerApp -> Renderer/Camera 생성 -> RenderContext 등록 -> RenderSystem 사용 -> World 정리 -> Renderer/Camera 파괴`
 * 순서를 지켜야 raw pointer가 dangling 되지 않는다.
 */
struct RenderContext
{
    /** @brief OpenGL frame/pass/draw command를 수행할 non-owning Renderer pointer. nullptr이면 렌더링 불가. */
    Renderer* renderer = nullptr;

    /**
     * @brief View/Projection matrix와 camera world position을 제공할 non-owning Camera pointer.
     * Camera position/target 공간 단위는 Scene/asset 단위를 따른다. 현재 HCR scene은 meter 기준이다.
     */
    Camera* camera = nullptr;
};
