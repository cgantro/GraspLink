#pragma once

#include <flecs.h>
#include <memory>

class Window;
class Renderer;
class Camera;
class OrbitCameraController;
class SceneManager;
class AssetManager;

namespace grasplink::robotics
{
class IRobotController;
}

namespace grasplink::viewer::robotics
{
class RobotTransformAdapter;
}

/**
 * @brief Viewer 실행 객체를 조립하고 전체 생명주기를 관리하는 Composition Root.
 */
/*
 * [추가 구조/용어 설명]
 * - Composition Root: 프로그램 시작 지점에서 실제 구현 객체를 만들고 서로 연결하는 장소.
 * - Lifetime: 객체가 생성되어 사용되고 파괴될 때까지의 수명.
 * - Dependency: 어떤 객체가 동작하기 위해 다른 객체를 필요로 하는 관계.
 * - ECS World: Flecs의 Entity/Component/System 전체를 보관하는 컨테이너.
 *
 * 현재 주요 흐름:
 * Window/Input -> OrbitCameraController -> Camera
 * GLB -> GltfLoader -> AssetManager/PrefabFactory -> Flecs Entity
 * Sim/Hardware IRobotController -> RobotState -> RobotTransformAdapter -> Flecs Transform
 * Flecs RenderSystem -> Renderer -> OpenGL
 *
 * ViewerApp 자체가 FK/IK, robot limit, OpenGL shader 수학을 직접 구현하는 것이 아니라
 * 각 책임 객체를 올바른 순서로 생성하고 연결하는 역할을 한다.
 */
class ViewerApp
{
public:
    /** @brief 아직 runtime 자원을 만들지 않은 application 객체를 생성한다. */
    ViewerApp();

    /** @brief Shutdown을 통해 소유 자원을 의존성 역순으로 정리한다. */
    ~ViewerApp();

    /** @brief Init 후 Window가 닫힐 때까지 MainLoop를 실행한다. 정상 종료 시 0을 반환한다. */
    int Run();

private:
    /** @brief Window/OpenGL/ECS/Scene/Asset/Robot Controller/Adapter를 생성하고 연결한다. */
    bool Init();

    /** @brief 입력, 제어 상태, ECS, 렌더링을 frame마다 갱신한다. */
    void MainLoop();

    /** @brief 참조 관계가 남지 않도록 상위 객체부터 역순으로 자원을 정리한다. */
    void Shutdown();

    // GLFW Window와 OpenGL Context owner.
    std::unique_ptr<Window> m_Window;

    // 실제 OpenGL frame/draw pass를 수행하는 Renderer owner.
    std::unique_ptr<Renderer> m_Renderer;

    // View/Projection 상태 owner.
    std::unique_ptr<Camera> m_Camera;

    // Mouse 입력을 Orbit/Pan/Zoom camera 조작으로 변환하는 객체 owner.
    std::unique_ptr<OrbitCameraController> m_CameraController;

    // 현재/다음 Scene의 lifetime owner.
    std::unique_ptr<SceneManager> m_SceneManager;

    // GPU Mesh/Material/Texture cache owner.
    std::unique_ptr<AssetManager> m_AssetManager;

    // 현재 선택된 robot backend owner. 인터페이스로 보관해 Simulation/Hardware 교체가 가능하다.
    std::unique_ptr<grasplink::robotics::IRobotController> m_RobotController;

    // RobotState 관절각을 GLB/Flecs Joint Local Rotation으로 표시하는 viewer adapter owner.
    std::unique_ptr<grasplink::viewer::robotics::RobotTransformAdapter> m_RobotTransformAdapter;

    // 모든 Flecs Entity/Component/System을 소유하는 ECS World.
    flecs::world m_World;
};
