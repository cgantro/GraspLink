#pragma once

class Renderer;
class Camera;

/**
 * @brief RenderSystem이 Renderer와 Camera에 접근하기 위한 non-owning context.
 *
 * @details
 * ECS Component는 보통 Entity가 가진 상태를 표현하지만 RenderContext는
 * System 전체가 공통으로 사용하는 외부 객체를 연결한다.
 *
 * Renderer와 Camera의 소유권은 ViewerApp에 있으므로 여기서는 raw pointer만 보관한다.
 * 따라서 RenderContext보다 Renderer/Camera의 수명이 반드시 길어야 한다.
 */
struct RenderContext
{
    Renderer* renderer = nullptr;
    Camera* camera = nullptr;
};
