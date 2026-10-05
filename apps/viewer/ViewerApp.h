#pragma once

#include "robotics/runtime/FixedControlLoop.h"
#include "assets/GraphicsTypes.h"

#include <flecs.h>
#include <memory>

class Window;
class Renderer;
class Camera;
class OrbitCameraController;
class SceneManager;
class AssetManager;
class Shader;
class Entity;

namespace grasplink::physics
{
class PhysicsWorld;
}

namespace grasplink::robotics
{
class IRobotController;
class IGripperController;
namespace kinematics { class RobotKinematics; class GripperKinematics; }
}

namespace grasplink::viewer::robotics
{
class RobotTransformAdapter;
class GripperTransformAdapter;
}

namespace grasplink::gui
{
class GuiModule;
}

namespace grasplink::simulation
{
class PhysicsSystemModule;
class RobotPhysicsAdapter;
}

/**
 * @brief Viewer의 선택 실행 모드.
 * @details physicsDemo는 로봇 이동과 물리 디버그 객체를 명시적으로 켠다. smokeTest는 자동 검증용으로
 * 창 표시를 끄고 정해진 렌더 프레임 수를 실행한다.
 */
struct ViewerOptions
{
    bool physicsDemo = false;
    bool smokeTest = false;
};

/**
 * @brief Viewer, Scene, 로봇 제어, 물리와 GUI 모듈의 수명을 조정한다.
 * @details
 * 계산은 각 모듈에 위임한다. 초기화는 Window/Context와 ECS에서 시작해 Scene·모델, Controller/FK,
 * Physics 순으로 연결한다. Shutdown은 빌린 Scene handle과 observer를 먼저 정리한 뒤 ECS/Physics를
 * 해제하고, 마지막에 GPU 자원과 이를 해제할 OpenGL Context를 정리한다.
 */
class ViewerApp
{
public:
    explicit ViewerApp(ViewerOptions options = {});
    ~ViewerApp();

    /** @brief 초기화에 성공하면 프레임 반복을 실행하고 종료 코드를 반환한다. */
    int Run();

private:
    ViewerOptions m_Options;
    bool Init();

    // Window와 OpenGL Context를 먼저 만들고, 그 Context를 쓰는 GPU/ECS/Scene을 준비한다.
    bool InitViewer();

    // GPU 업로드 모델을 Scene의 Entity 계층으로 만들고, 로봇 모델 리소스는 collider 추출에도 보관한다.
    void InitScene(Entity& robotRoot, Entity& floorEntity);

    // Controller 상태 → FK 결과 → 시각 Entity와 Kinematic collider Entity 연결을 구성한다.
    bool InitRobot(const Entity& robotRoot);

    // Gripper의 연속 상태와 Local 회전 계산을 연결한다. 원본 GLB의 장착·관절 위치는 유지한다.
    bool InitGripper(const Entity& robotRoot);

    // ECS 설정에서 Jolt Body를 만들고, 시각/물리 프록시의 첫 자세를 동기화한다.
    void InitPhysics(const Entity& robotRoot, Entity& floorEntity);

    // 두 Controller snapshot에서 계산한 자세를 적용한다. World 행렬은 호출자가 이 뒤 갱신한다.
    void ApplyControllerPoses();

    // 입력, 누적 Fixed Update, 렌더 프레임 처리를 서로 다른 주기로 반복한다.
    void MainLoop();

    // 부분 초기화 실패 때도 호출된다. World observer와 OpenGL Context가 필요한 정리를 먼저 끝낸다.
    void Shutdown();

    // GLFW Window와 OpenGL Context 소유. Renderer/GPU 객체보다 오래 살아야 한다.
    std::unique_ptr<Window> m_Window;

    // OpenGL Render 담당
    std::unique_ptr<Renderer> m_Renderer;

    // View·Projection 행렬 관리
    std::unique_ptr<Camera> m_Camera;

    // Mouse 입력 → Camera 조작
    std::unique_ptr<OrbitCameraController> m_CameraController;

    // Scene 생성·수명 관리
    std::unique_ptr<SceneManager> m_SceneManager;

    // GPU Mesh·Material·Texture 관리
    std::unique_ptr<AssetManager> m_AssetManager;

    // Collider 추출용 CPU 데이터와 업로드된 GPU Mesh를 함께 공유한다. Context보다 먼저 해제한다.
    ModelResource m_RobotModel;

    // Robot·Debug Box 공용 Shader
    std::shared_ptr<Shader> m_RobotShader;

    // Flecs Entity·Component·System 소유. Physics observer가 Body를 정리할 때까지 PhysicsWorld가 살아 있어야 한다.
    flecs::world m_World;

    // Robot 제어 Backend
    std::unique_ptr<grasplink::robotics::IRobotController> m_RobotController;

    // Gripper 개폐 위치의 기준 상태를 소유한다. GUI는 요청을 보내고 snapshot을 읽는다.
    std::unique_ptr<grasplink::robotics::IGripperController> m_GripperController;

    // Adapter는 Scene Entity handle을 빌린다. Scene 정리 전에 파괴한다.
    std::unique_ptr<grasplink::robotics::kinematics::RobotKinematics> m_RobotKinematics;
    std::unique_ptr<grasplink::viewer::robotics::RobotTransformAdapter> m_RobotTransformAdapter;
    std::unique_ptr<grasplink::simulation::RobotPhysicsAdapter> m_RobotPhysicsAdapter;
    std::unique_ptr<grasplink::robotics::kinematics::GripperKinematics> m_GripperKinematics;
    std::unique_ptr<grasplink::viewer::robotics::GripperTransformAdapter> m_GripperTransformAdapter;

    // Robot·Physics 고정 시간 업데이트
    grasplink::robotics::runtime::FixedControlLoop m_ControlLoop;

    // Jolt World 소유
    std::unique_ptr<grasplink::physics::PhysicsWorld> m_PhysicsWorld;

    // Flecs 물리 설정과 Jolt 연결·고정 Step 실행
    std::unique_ptr<grasplink::simulation::PhysicsSystemModule> m_PhysicsSystemModule;

    // GUI lifecycle과 디버그 panel
    std::unique_ptr<grasplink::gui::GuiModule> m_GuiModule;
};
