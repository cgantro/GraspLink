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
 * @brief Viewer 실행에 필요한 객체를 만들고 서로 연결하는 최상위 Application 객체.
 *
 * @details
 * `Composition Root`는 프로그램 시작점에서 실제 구현 객체를 생성하고 의존관계를 조립하는 장소를 뜻한다.
 * ViewerApp은 로봇 제어 수학을 직접 구현하지 않고 다음 객체들을 연결한다.
 *
 * `Sim/Hardware Controller`
 * -> `RobotState`
 * -> `RobotTransformAdapter`
 * -> `Flecs Transform`
 * -> `Renderer`
 *
 * 로보틱스 용어:
 * - Controller: 목표 관절각을 받고 현재 관절 상태를 계산/제어하는 객체.
 * - RobotState: 특정 시점의 J1..Jn 각도/속도 등을 담은 상태 snapshot.
 * - Adapter: RobotState를 화면의 Entity transform 형식으로 바꾸는 연결 계층.
 * - Fixed Control Loop: 렌더링 FPS와 무관하게 일정한 dt로 Controller를 업데이트하는 구조.
 *
 * 현재 MainLoop는 rendering frame dt를 그대로 SimRobotController::Update()에 전달한다.
 * 향후에는 accumulator 기반 Fixed Control Loop로 분리하고 Renderer는 최신 RobotState만 소비하도록 변경할 예정이다.
 */
class ViewerApp
{
public:
    /** @brief 아직 Window/Scene/Controller를 만들지 않은 빈 application 객체를 생성한다. */
    ViewerApp();

    /** @brief Shutdown()을 호출해 Controller/Scene/GPU/Window 자원을 의존성 역순으로 정리한다. */
    ~ViewerApp();

    /**
     * @brief 전체 Application을 초기화하고 Window가 닫힐 때까지 main loop를 실행한다.
     * @return 정상 종료 시 0, Init 실패 시 음수 값.
     */
    int Run();

private:
    /**
     * @brief Window/OpenGL, ECS, Scene, Asset, Robot Controller, Transform Adapter를 순서대로 생성/연결한다.
     * @return 모든 필수 초기화와 Controller Connect가 성공하면 true.
     */
    bool Init();

    /**
     * @brief 입력 처리 -> Controller update -> RobotState 시각 반영 -> ECS update -> Render 순서로 frame을 반복한다.
     * @note 현재 dt는 rendering frame 간 경과시간 [s]다. 향후 Controller용 fixed dt와 분리한다.
     */
    void MainLoop();

    /** @brief 상위 의존 객체부터 역순으로 해제해 dangling reference 없이 자원을 정리한다. */
    void Shutdown();

    /** @brief GLFW Window와 OpenGL context를 소유한다. 그래픽 객체보다 늦게 파괴되어야 한다. */
    std::unique_ptr<Window> m_Window;

    /** @brief OpenGL 렌더링 기능을 소유한다. */
    std::unique_ptr<Renderer> m_Renderer;

    /** @brief 화면을 바라보는 View/Projection camera 상태를 소유한다. */
    std::unique_ptr<Camera> m_Camera;

    /** @brief 마우스 입력을 camera 회전/이동으로 바꾸는 Viewer용 controller. Robot Controller와는 별개다. */
    std::unique_ptr<OrbitCameraController> m_CameraController;

    /** @brief 현재 Scene의 생성/교체/lifetime을 관리한다. */
    std::unique_ptr<SceneManager> m_SceneManager;

    /** @brief CPU에서 읽은 GLB 자원을 GPU Mesh/Material/Texture로 올리고 캐시한다. */
    std::unique_ptr<AssetManager> m_AssetManager;

    /**
     * @brief 현재 선택된 Robot backend를 IRobotController 인터페이스로 소유한다.
     * @details 현재는 SimRobotController지만 나중에 실제 Hanwha Hardware backend로 바꿔도 상위 흐름은 유지한다.
     */
    std::unique_ptr<grasplink::robotics::IRobotController> m_RobotController;

    /**
     * @brief Controller의 RobotState 관절각[rad]을 GLB/Flecs Joint Local Rotation으로 변환하는 Viewer adapter.
     * @note 제어 limit/trajectory/IK를 계산하지 않는다.
     */
    std::unique_ptr<grasplink::viewer::robotics::RobotTransformAdapter> m_RobotTransformAdapter;

    /** @brief 모든 Flecs Entity/Component/System을 소유하는 ECS World. */
    flecs::world m_World;
};
