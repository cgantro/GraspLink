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
 *
 * @details
 * ViewerApp은 로봇 제어 수학이나 OpenGL 세부 구현을 직접 담당하지 않는다.
 * 각 하위 객체를 올바른 순서로 생성하고 다음 데이터 흐름을 연결한다.
 *
 * `Sim/Hardware IRobotController -> RobotState -> RobotTransformAdapter -> Flecs Transform -> Renderer`
 *
 * 현재 MainLoop의 dt는 rendering frame delta [s]이며 SimRobotController::Update()에 그대로 전달된다.
 * 향후 Fixed Control Loop를 도입하면 controller update를 accumulator 기반 고정 dt로 분리하고
 * rendering은 최신 RobotState만 소비하도록 변경해야 한다.
 */
class ViewerApp
{
public:
    /** @brief 아직 runtime 자원을 만들지 않은 빈 application 객체를 생성한다. */
    ViewerApp();

    /** @brief Shutdown()을 통해 Controller/Scene/GPU/Window 자원을 의존성 역순으로 정리한다. */
    ~ViewerApp();

    /**
     * @brief Application을 초기화하고 Window 종료까지 MainLoop를 실행한다.
     * @return 정상 종료 시 0, Init 실패 시 음수 값.
     */
    int Run();

private:
    /**
     * @brief Window/OpenGL, Flecs systems, Scene/Assets, Robot Controller와 Transform Adapter를 순서대로 구성한다.
     * @return 모든 필수 자원과 controller 연결이 성공하면 true.
     */
    bool Init();

    /**
     * @brief 입력 -> controller state update -> visual transform 반영 -> scene/ECS -> render 순서로 frame을 반복한다.
     * @note frame delta time 단위는 second [s]이며 debugger 등으로 큰 값이 들어오면 구현에서 clamp한다.
     */
    void MainLoop();

    /** @brief 서로 참조하는 상위 객체부터 해제해 dangling reference 없이 application 자원을 정리한다. */
    void Shutdown();

    /** @brief GLFW window/OpenGL context owner. 가장 늦게 파괴되어야 한다. */
    std::unique_ptr<Window> m_Window;

    /** @brief OpenGL rendering backend owner. Window context보다 먼저 파괴한다. */
    std::unique_ptr<Renderer> m_Renderer;

    /** @brief View/Projection camera state owner. */
    std::unique_ptr<Camera> m_Camera;

    /** @brief Window mouse input을 Camera pose로 변환하는 controller owner. */
    std::unique_ptr<OrbitCameraController> m_CameraController;

    /** @brief Active Scene의 생성/전환/lifetime manager. */
    std::unique_ptr<SceneManager> m_SceneManager;

    /** @brief CPU ModelResource를 GPU Mesh/Material/Texture로 업로드하고 캐시하는 owner. */
    std::unique_ptr<AssetManager> m_AssetManager;

    /**
     * @brief 현재 선택된 Robot backend의 다형 owner.
     * 현재 ViewerApp에서는 SimRobotController를 넣지만 향후 Hardware implementation으로 교체 가능하다.
     */
    std::unique_ptr<grasplink::robotics::IRobotController> m_RobotController;

    /** @brief RobotState [rad]를 GLB/Flecs Joint Local Rotation으로 변환하는 Viewer-side adapter owner. */
    std::unique_ptr<grasplink::viewer::robotics::RobotTransformAdapter> m_RobotTransformAdapter;

    /** @brief 모든 Entity/Component/System을 소유하는 Flecs World. */
    flecs::world m_World;
};
