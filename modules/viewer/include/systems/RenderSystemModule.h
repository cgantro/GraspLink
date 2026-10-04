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
/*
 * [추가 그래픽스/ECS 용어 설명]
 * - Flecs Module: 관련 Component/System 등록 코드를 하나의 묶음으로 World에 import하는 구조.
 * - Query: 특정 Component 조합을 가진 Entity만 찾는 ECS 검색 조건.
 * - Shadow Pass: light 관점 depth를 먼저 그리는 단계.
 * - Main Pass: 실제 화면 색/조명을 그리는 단계.
 * - Frustum Culling: Camera 시야 밖의 객체를 draw 대상에서 제외하는 최적화.
 * - Render Queue: 투명도/Shader/거리 등 기준으로 draw 순서를 정렬한 목록.
 *
 * 책임 분리:
 * RenderSystemModule = 어떤 Entity를 그릴지 선택
 * Renderer           = 선택된 데이터를 실제 OpenGL 명령으로 그림
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
