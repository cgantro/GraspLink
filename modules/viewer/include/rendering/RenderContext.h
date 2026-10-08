#pragma once

namespace grasplink::graphics
{
class Renderer;
class Camera;
}

namespace grasplink::rendering
{

/**
 * @brief 장면 그리기 시스템이 현재 프레임에 사용할 Renderer와 Camera를 찾는 전역 자료다.
 * @details
 * Renderer는 Mesh의 삼각형을 GPU에 그리라는 OpenGL 명령을 보내고 Camera는 화면에서 볼 위치와 원근을 정한다.
 * 이 포인터들은 객체를 빌려 가리킬 뿐 수명을 관리하지 않는다. ViewerApp이 두 객체를 소유하므로 장면과 이 자료를 먼저 없애야 한다.
 * 둘 중 하나라도 nullptr이면 장면 그리기 시스템은 해당 프레임의 그리기를 건너뛴다. 이 포인터를 소유 객체보다 오래 보관하지 않는다.
 */
struct RenderContext
{
    /// Entity의 Mesh를 GPU에 그리는 Renderer를 빌려 가리킨다. 이 포인터는 수명을 연장하지 않는다.
    ::grasplink::graphics::Renderer* renderer = nullptr;

    /// 장면 좌표를 카메라 좌표와 화면 투영 좌표로 바꾸는 Camera를 빌려 가리킨다. 장면 그리기 중에는 계속 살아 있어야 한다.
    ::grasplink::graphics::Camera* camera = nullptr;
};

} // namespace grasplink::rendering
