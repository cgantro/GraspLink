#include "Camera.h"
#include "Window.h"
#include "TestSupport.h"
#include "gui/GuiModule.h"
#include "gui/overlays/ColliderOverlay.h"
#include "gui/panels/GripperPanel.h"
#include "gui/panels/PhysicsDebugPanel.h"
#include "robotics/backends/simulation/SimGripperController.h"
#include "robotics/models/robotiq/TwoF85.h"
#include "scene/SceneManager.h"
#include "simulation/components/PhysicsComponents.h"
#include "systems/TransformSystemModule.h"

#include <imgui.h>

#include <iostream>

namespace
{
using namespace grasplink::robotics;

Colliders ConfiguredShapes()
{
    Colliders colliders;
    colliders.shapes.push_back(physics_colliders::Box({0.25F, 0.25F, 0.25F}, {-1.0F, 0.0F, 0.0F}));
    colliders.shapes.push_back(physics_colliders::Sphere(0.25F, {-0.3F, 0.0F, 0.0F}));
    colliders.shapes.push_back(physics_colliders::Cylinder(0.25F, 0.3F, {0.4F, 0.0F, 0.0F}));
    grasplink::physics::CollisionShapeDescription hull;
    hull.type = grasplink::physics::CollisionShapeType::ConvexHull;
    hull.localTransform.position = {1.1F, 0.0F, 0.0F};
    // 세 축에 두께가 있는 8개 꼭짓점 [m]. 투영된 외곽선도 화면 안에서 확인할 수 있게 배치한다.
    for (int corner = 0; corner < 8; ++corner)
        hull.pointsMeters.push_back({(corner & 1) ? 0.25F : -0.25F,
            (corner & 2) ? 0.25F : -0.25F, (corner & 4) ? 0.25F : -0.25F});
    colliders.shapes.push_back(hull);
    return colliders;
}

void RequireControllerUnchanged(const GripperState& actual, const GripperState& before)
{
    RequireNear(actual.closureFraction, before.closureFraction, 0.0, "panel draw preserves continuous position");
    Require(actual.mode == before.mode, "panel draw preserves mode");
    Require(actual.requestedPositionEcho == before.requestedPositionEcho, "panel draw preserves command echo");
    Require(actual.actualPosition == before.actualPosition, "panel draw preserves raw feedback");
    Require(actual.valid == before.valid && actual.closureFractionValid == before.closureFractionValid,
        "panel draw preserves position validity");
    Require(actual.activated == before.activated && actual.goToActive == before.goToActive,
        "panel draw preserves activation and movement");
}
}

/**
 * @brief 숨김 OpenGL 창에서 분리된 GUI 수명·패널·Collider 투영을 함께 검증한다.
 * @details 전경 draw list의 정점을 검사해 선 표시·숨김과 Scene 교체 후 캐시 갱신을 확인한다.
 * 패널을 읽기만 하는 프레임은 Controller 명령이나 시간 진행을 만들지 않아야 한다.
 * Window는 GUI보다, Flecs World는 SceneManager와 Overlay보다 오래 살아 있도록 선언한다.
 */
int main()
{
    try
    {
        Window::Properties properties;
        properties.width = 960;
        properties.height = 720;
        properties.title = "GUI composition regression";
        properties.visible = false;
        properties.vsync = false;
        Window window(properties);
        flecs::world world;
        world.import<TransformSystemModule>();
        SceneManager scenes(world);
        scenes.LoadScene<Scene>();
        scenes.OnUpdate(0.0F);
        Require(scenes.GetActiveScene() != nullptr, "identity scene is active");
        Entity collider = scenes.GetActiveScene()->CreateEntity("ConfiguredShapes");
        collider.set(RigidBody{grasplink::physics::BodyMotionType::Static,
            grasplink::physics::CollisionLayer::Environment}).set(ConfiguredShapes());
        TransformSystemModule::UpdateWorldTransforms(world);

        Camera camera({0.0F, 2.0F, 5.0F}, {0.0F, 0.0F, 0.0F}, 960.0F / 720.0F);
        grasplink::gui::GuiModule gui(window);
        // 테스트가 실행 위치의 사용자 ImGui 배치를 읽거나 저장하지 않게 한다.
        ImGui::GetIO().IniFilename = nullptr;
        grasplink::gui::GripperPanel gripperPanel;
        grasplink::gui::PhysicsDebugPanel physicsPanel;
        grasplink::gui::ColliderOverlay overlay(world);
        backends::simulation::SimGripperController controller(models::robotiq::kTwoF85);
        Require(controller.Connect().Ok() && controller.Activate().Ok(), "gripper controller activated");
        Require(controller.Command({180, 128, 128}).Ok(), "controller has a nontrivial movement request");
        controller.Update(0.2);
        const GripperState before = controller.GetState();
        Require(before.mode == GripperMode::Moving && before.closureFraction > 0.0,
            "controller movement is prepared before drawing");
        Require(!physicsPanel.IsColliderVisible(), "debug panel starts with lines hidden");

        const auto drawFrame = [&](bool visible)
        {
            window.PollEvents();
            gui.BeginFrame();
            Require(ImGui::GetIO().DisplaySize.x > 0.0F && ImGui::GetIO().DisplaySize.y > 0.0F,
                "hidden window provides drawable display size");
            gripperPanel.Draw(controller);
            physicsPanel.Draw();
            overlay.Draw(camera, visible);
            // 패널은 각 창의 draw list를 쓴다. 전경 정점은 Overlay 선에서만 생성된다.
            const int foregroundVertices = ImGui::GetForegroundDrawList()->VtxBuffer.Size;
            (void)gui.WantsMouse();
            (void)gui.WantsKeyboard();
            gui.EndFrame();
            RequireControllerUnchanged(controller.GetState(), before);
            Require(!physicsPanel.IsColliderVisible(), "drawing preserves debug checkbox state");
            return foregroundVertices;
        };

        Require(drawFrame(true) > 0, "configured shapes generate foreground wire lines");
        // 형상별로 캐시를 비우고 다시 표시해 네 투영 경로가 각각 선을 만드는지 확인한다.
        for (const auto& shape : ConfiguredShapes().shapes)
        {
            collider.set(Colliders{{shape}});
            Require(drawFrame(false) == 0, "shape change hidden frame remains empty");
            Require(drawFrame(true) > 0, "each configured shape generates foreground wire lines");
        }
        Require(drawFrame(false) == 0, "hidden overlay emits no foreground lines");
        scenes.LoadScene<Scene>();
        scenes.OnUpdate(0.0F);
        TransformSystemModule::UpdateWorldTransforms(world);
        Require(!collider.IsValid(), "scene replacement removes the configured collider");
        // 숨김→표시 전환은 100 ms를 기다리지 않고 새 Scene의 빈 query로 캐시를 갱신해야 한다.
        Require(drawFrame(false) == 0, "replacement scene hidden frame remains empty");
        Require(drawFrame(true) == 0, "replacement scene does not draw stale collider lines");
        std::cout << "GUI composition, collider visibility and scene replacement checks passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
