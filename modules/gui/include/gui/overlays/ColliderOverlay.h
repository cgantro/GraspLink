#pragma once

#include <flecs.h>

#include <memory>

class Camera;

namespace grasplink::gui
{

/**
 * @brief ECS Collider 설정의 화면 투영선을 깊이 검사 없이 표시한다.
 * @details Jolt 내부 Shape를 조회하지 않는 근사 표시다. PIMPL이 World query와 화면 선 캐시를
 * 소유하므로 빌린 flecs::world보다 먼저 파괴해야 한다. ImGui context나 OpenGL backend는 소유하지 않는다.
 */
class ColliderOverlay
{
public:
    /**
     * @brief World의 Collider·World 변환 조회를 준비한다.
     * @param world Collider 설정과 World 변환을 읽을 World. Overlay보다 오래 살아 있어야 한다.
     */
    explicit ColliderOverlay(flecs::world& world);
    ~ColliderOverlay();

    ColliderOverlay(const ColliderOverlay&) = delete;
    ColliderOverlay& operator=(const ColliderOverlay&) = delete;

    /**
     * @brief 활성 ImGui 프레임 안에서 설정 Collider의 캐시된 투영선을 그린다.
     * @param camera 이번 호출에서만 빌릴 Camera. 참조를 저장하지 않는다.
     * @param visible false면 선 캐시를 비우고, 다음 첫 표시 때 즉시 갱신한다.
     * @details 첫 표시·화면 크기 변경 때와 100 ms마다 다시 투영한다. Camera가 이동해도 다음 갱신까지
     * 이전 화면 선을 사용한다. 전경 draw list를 사용해 가려진 형상도 보이며 OpenGL 호출은 직접 하지 않는다.
     */
    void Draw(const Camera& camera, bool visible);

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

}
