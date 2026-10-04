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
/*
 * [추가 용어 설명]
 * - Context: 여러 함수/System이 공통으로 필요한 객체 묶음을 전달하기 위한 접근 지점.
 * - non-owning: 이 구조체가 Renderer/Camera를 생성하거나 삭제할 책임은 없다는 뜻.
 * - raw pointer: C++의 일반 포인터. 여기서는 소유권이 아니라 접근용 주소로만 사용한다.
 * - ECS Singleton/Context 용도: 특정 Entity 하나의 상태가 아니라 World 전체 System이 공유할 외부 서비스를 연결한다.
 *
 * renderer/camera가 nullptr이면 RenderSystem이 사용할 실제 객체가 연결되지 않은 상태다.
 */
struct RenderContext
{
    // 실제 OpenGL draw를 수행할 Renderer. 소유권은 ViewerApp에 있다.
    Renderer* renderer = nullptr;

    // 현재 frame의 View/Projection 정보를 제공할 Camera. 소유권은 ViewerApp에 있다.
    Camera* camera = nullptr;
};
