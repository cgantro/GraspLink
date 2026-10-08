#pragma once

#include "robotics/runtime/FixedControlLoop.h"
#include "diagnostics/Logger.h"
#include "assets/GraphicsTypes.h"
#include "PickPlaceMission.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <flecs.h>
#include <memory>

class Window;
class Renderer;
class Camera;
class OrbitCameraController;
class Scene;
class AssetManager;
class Shader;
class Entity;

namespace grasplink::physics
{
class PhysicsWorld;
}

namespace grasplink::robotics
{
namespace kinematics { class RobotKinematics; class GripperKinematics; }
}

namespace grasplink::robotics::backends::simulation
{
class SimRobotController;
class SimGripperController;
}

namespace grasplink::viewer::robotics
{
class RobotTransformAdapter;
class GripperTransformAdapter;
}

namespace grasplink::gui
{
class GuiModule;
class GripperPanel;
class RobotPanel;
class PhysicsDebugPanel;
class ColliderOverlay;
}

namespace grasplink::simulation
{
class PhysicsSystemModule;
class RobotPhysicsAdapter;
class GripperGraspAdapter;
}

namespace grasplink::viewer
{
class ViewerRobotCollisionGuard;
}

/** Options for automated Viewer checks. */
struct ViewerOptions
{
    bool smokeTest = false;
};

/**
 * @brief Viewer 창, 장면, 로봇 제어, 물리와 GUI 객체를 만들고 종료 순서를 관리한다.
 * @details 그래픽 창은 GPU 자원을 만들고 해제하는 OpenGL context를 제공한다. Entity는 장면 물체 참조이고 Flecs World가 실제 물체와 값을 소유하므로 이 앱은 두 수명을 조정한다.
 * 초기화는 창과 World를 준비한 뒤 장면·모델·제어기·물리를 연결한다. 로봇 FK는 관절 각도에서 링크와 도구 끝의 위치·방향을 계산하며, 결과는 화면 모델과 물리 충돌용 단순 물체에 적용한다.
 * 종료할 때 Scene 참조, 삭제 observer, World, 물리 상태 순으로 정리하고 GPU 자원 해제까지 OpenGL context를 유지한다.
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
    grasplink::diagnostics::Logger m_Logger;
    bool Init();

    // Window와 OpenGL Context를 만든 뒤, 이 Context를 필요로 하는 GPU 자원과 ECS 및 Scene을 초기화한다.
    bool InitViewer();

    // GLB 모델의 node 부모 관계로 장면 물체를 만들고, 물리 충돌용 단순 모양을 만들 원본 꼭짓점 자료도 보관한다.
    void InitScene(Entity& robotRoot, Entity& floorEntity);

    // Controller의 관절 각도에서 각 링크 위치와 방향을 계산해 화면 모델과 로봇을 따라 움직이는 충돌 물체에 적용한다.
    bool InitRobot(const Entity& robotRoot);

    // 그리퍼 개폐 비율을 손가락 관절별 부모 기준 회전으로 바꾼다. GLB가 정한 장착 위치와 관절 위치는 유지한다.
    bool InitGripper(const Entity& robotRoot);

    // ECS의 충돌 모양 설정에서 Jolt 물체를 만들고, 초기 위치를 화면 물체 및 화면 모델을 따라가는 충돌용 물체에 맞춘다.
    void InitPhysics(const Entity& robotRoot, Entity& floorEntity);

    // Controller 상태에서 계산한 로봇과 그리퍼 위치·방향을 장면 물체에 적용한다. 호출자는 이후 Scene 전체 행렬을 다시 계산한다.
    void ApplyControllerPoses();

    // 입력, 누적 Fixed Update, 렌더 프레임 처리를 서로 다른 주기로 반복한다.
    void MainLoop();

    // 초기화가 일부만 성공한 경우에도 호출된다. ECS observer가 Body를 정리하고 GPU 자원이 해제될 수 있도록 World와 OpenGL Context 수명을 고려해 정리한다.
    void Shutdown();

    // GLFW Window와 OpenGL Context를 소유한다. Renderer와 GPU 객체의 소멸자가 실행될 때까지 Context를 유지한다.
    std::unique_ptr<Window> m_Window;

    // OpenGL 상태를 설정하고 Mesh를 화면에 그린다.
    std::unique_ptr<Renderer> m_Renderer;

    // 카메라의 위치와 View·Projection 행렬을 관리한다.
    std::unique_ptr<Camera> m_Camera;

    // 마우스 입력으로 바라보는 지점 주위를 도는 카메라를 움직인다.
    std::unique_ptr<OrbitCameraController> m_CameraController;
    // Owns the SceneRoot until physics cleanup is complete.
    std::unique_ptr<Scene> m_Scene;

    // 모델의 Mesh, Material, Texture를 GPU 자원으로 올리고 공유한다.
    std::unique_ptr<AssetManager> m_AssetManager;

    // Collider 추출에 필요한 원본 CPU 정점 자료를 보관한다.
    ModelResource m_RobotModel;

    // 로봇 모델과 디버그 상자 렌더링에 함께 사용하는 Shader다.
    std::shared_ptr<Shader> m_RobotShader;

    // 장면 물체와 위치 같은 값, 자동 실행 규칙을 저장하는 Flecs World다. 물체 삭제를 감지하는 callback이 Jolt 물체를 지울 때까지 PhysicsWorld가 살아 있어야 한다.
    flecs::world m_World;

    // 로봇의 관절 상태를 갱신하고 이동 명령을 처리하는 제어 backend다.
    std::unique_ptr<grasplink::robotics::backends::simulation::SimRobotController> m_RobotController;

    // 그리퍼의 개폐 명령과 현재 상태를 관리한다. GUI는 명령을 보내고 상태의 독립된 복사본을 읽는다.
    std::unique_ptr<grasplink::robotics::backends::simulation::SimGripperController> m_GripperController;

    // 아래 변환 adapter는 Scene Entity handle을 빌려 쓰므로 Scene보다 먼저 파괴해야 한다.
    std::unique_ptr<grasplink::robotics::kinematics::RobotKinematics> m_RobotKinematics;
    std::unique_ptr<grasplink::viewer::robotics::RobotTransformAdapter> m_RobotTransformAdapter;
    std::unique_ptr<grasplink::simulation::RobotPhysicsAdapter> m_RobotPhysicsAdapter;
    std::unique_ptr<grasplink::robotics::kinematics::GripperKinematics> m_GripperKinematics;
    std::unique_ptr<grasplink::viewer::robotics::GripperTransformAdapter> m_GripperTransformAdapter;

    // 렌더 프레임 시간과 분리해 Controller 및 Physics를 고정 간격으로 실행한다.
    grasplink::robotics::runtime::FixedControlLoop m_ControlLoop;

    // 충돌 형상과 강체 상태를 보관하고 시뮬레이션하는 Jolt World를 소유한다.
    std::unique_ptr<grasplink::physics::PhysicsWorld> m_PhysicsWorld;

    // ECS의 물리 설정을 Jolt Body에 연결하고 고정 시간 간격의 물리 step을 실행한다.
    std::unique_ptr<grasplink::simulation::PhysicsSystemModule> m_PhysicsSystemModule;

    // 물리 계산에서 확인한 손가락 접촉을 개폐 정지에 반영하고, 양쪽 손가락이 같은 물체를 잡으면 물리 제약의 생성과 해제를 관리한다.
    std::unique_ptr<grasplink::simulation::GripperGraspAdapter> m_GripperGraspAdapter;

    // Controller가 제안한 자세와 매 고정 tick의 실제 자세를 Environment와 검사한다. 빌린 객체보다 먼저 해제한다.
    std::unique_ptr<grasplink::viewer::ViewerRobotCollisionGuard> m_RobotCollisionGuard;

    // ImGui backend, 조작 패널, 디버그 패널과 Collider overlay의 수명을 앱이 함께 관리한다.
    std::unique_ptr<grasplink::gui::GuiModule> m_GuiModule;
    std::unique_ptr<grasplink::gui::GripperPanel> m_GripperPanel;
    std::unique_ptr<grasplink::gui::RobotPanel> m_RobotPanel;
    grasplink::viewer::PickPlaceMission m_PickPlaceMission{grasplink::robotics::models::hanwha::kHcr12a};
    std::unique_ptr<grasplink::gui::PhysicsDebugPanel> m_PhysicsDebugPanel;
    std::unique_ptr<grasplink::gui::ColliderOverlay> m_ColliderOverlay;

    // GUI가 장면 속 로봇 기준점과 파지 상자의 World 위치를 매 프레임 읽는다. wrapper는 Entity를 소유하지 않는다.
    std::unique_ptr<Entity> m_RobotRoot;
    std::unique_ptr<Entity> m_GraspBox;
    std::unique_ptr<Entity> m_PlacementArea;
};
