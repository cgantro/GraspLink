#pragma once

#include <flecs.h>
#include <memory>

class Window;
class Renderer;
class Camera;
class OrbitCameraController;
class SceneManager;
class AssetManager;
class RobotJointController;

/**
 * @brief GraspLink Viewer 실행 객체를 조립하고 전체 생명주기를 관리한다.
 *
 * @details
 * ViewerApp은 각 기능의 구현 세부사항을 직접 담당하지 않는다.
 * Window, Renderer, Camera, Scene, Asset, Robot Controller를 생성하고
 * 메인 루프에서 올바른 순서로 호출하는 Composition Root 역할을 한다.
 *
 * 초심자 관점에서 보면 "프로그램을 구성하는 부품들을 연결하는 곳"이다.
 * 실제 렌더링은 Renderer, 입력 해석은 OrbitCameraController,
 * ECS 갱신은 Flecs System이 담당한다.
 *
 * @note 멤버의 선언 순서는 파괴 순서와 관련된다. 참조를 보관하는 객체는
 *       자신이 참조하는 객체보다 먼저 파괴되어야 한다.
 * @todo [FUTURE] 기본 Scene 대신 SimulationScene을 도입하면 로봇/환경 생성 코드를
 *       ViewerApp에서 Scene 계층으로 이동한다.
 */
class ViewerApp
{
public:
    /** @brief 빈 ViewerApp 객체를 생성한다. 실제 자원 생성은 Init()에서 수행한다. */
    ViewerApp();

    /** @brief 생성된 Viewer 자원을 안전한 순서로 해제한다. */
    ~ViewerApp();

    /**
     * @brief Viewer를 초기화하고 메인 루프를 실행한다.
     * @return 정상 종료 시 0, 초기화 실패 시 음수 값.
     */
    int Run();

private:
    /**
     * @brief Window, Renderer, Camera, ECS, Scene, Asset을 생성하고 연결한다.
     * @return 초기화 성공 여부.
     */
    bool Init();

    /** @brief Window가 닫힐 때까지 입력, Scene, ECS, Render 순서로 프레임을 반복한다. */
    void MainLoop();

    /** @brief 서로 참조하는 객체들의 의존 관계를 고려해 자원을 해제한다. */
    void Shutdown();

    std::unique_ptr<Window> m_Window;
    std::unique_ptr<Renderer> m_Renderer;
    std::unique_ptr<Camera> m_Camera;
    std::unique_ptr<OrbitCameraController> m_CameraController;

    std::unique_ptr<SceneManager> m_SceneManager;
    std::unique_ptr<AssetManager> m_AssetManager;
    std::unique_ptr<RobotJointController> m_RobotJointController;

    /// ECS Entity, Component, System을 소유하는 Flecs World.
    flecs::world m_World;
};
