#include "Camera.h"
#include "Window.h"
#include "TestSupport.h"
#include "gui/GuiModule.h"
#include "gui/overlays/ColliderOverlay.h"
#include "gui/panels/GripperPanel.h"
#include "gui/panels/PhysicsDebugPanel.h"
#include "robotics/backends/simulation/SimGripperController.h"
#include "robotics/models/robotiq/TwoF85.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"
#include "systems/TransformSystemModule.h"

#include <imgui.h>

#include <iostream>
#include <memory>

namespace
{
using namespace grasplink::robotics;

Colliders ConfiguredShapes()
{
    Colliders colliders;
    colliders.shapes.push_back(physics_colliders::Box({0.25F, 0.25F, 0.25F}, {-1.0F, 0.0F, 0.0F}));
    colliders.shapes.push_back(physics_colliders::Box({0.2F, 0.2F, 0.2F}, {-0.3F, 0.0F, 0.0F}));
    colliders.shapes.push_back(physics_colliders::Box({0.25F, 0.3F, 0.25F}, {0.4F, 0.0F, 0.0F}));
    grasplink::physics::CollisionShapeDescription hull;
    hull.type = grasplink::physics::CollisionShapeType::ConvexHull;
    hull.localTransform.position = {1.1F, 0.0F, 0.0F};
    // 세 방향 모두 두께가 있는 상자의 여덟 꼭짓점을 [m] 단위로 지정한다. 카메라에 투영한 외곽선이 화면 안에 나타나도록 위치시킨다.
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
 * @brief 화면 뒤에서 실행한 GUI 프레임에서도 패널 입력과 충돌 모양 선이 기대대로 처리되는지 확인한다.
 * @details draw list는 ImGui가 현재 프레임에 화면에 그릴 점과 선을 담는 목록이다. 이 시험은 ColliderOverlay가 그리는 선을 검사해 표시·숨김과 Scene 교체 뒤 새 물체를 다시 읽는 동작을 확인한다.
 * 그리퍼 상태를 읽기만 한 프레임은 Controller 명령이나 시뮬레이션 시간 진행을 만들지 않아야 한다.
 * Window는 GUI 연결보다 오래 살아야 하고 Flecs World는 Scene과 Overlay보다 오래 살아야 한다.
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
        auto scene = std::make_unique<Scene>(world);
        Require(scene->GetSceneRoot().is_alive(), "Scene owns a live hierarchy root");
        Entity collider = scene->CreateEntity("ConfiguredShapes");
        collider.set(RigidBody{grasplink::physics::BodyMotionType::Static,
            grasplink::physics::CollisionLayer::Environment}).set(ConfiguredShapes());
        TransformSystemModule::UpdateWorldTransforms(world);

        Camera camera({0.0F, 2.0F, 5.0F}, {0.0F, 0.0F, 0.0F}, 960.0F / 720.0F);
        grasplink::gui::GuiModule gui(window);
        // 테스트 실행 위치에 사용자의 ImGui 설정 파일을 읽거나 기록하지 않도록 임시 설정 경로를 사용한다.
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
            // 일반 패널은 각 창 안에 그려진다. ColliderOverlay의 선은 물체에 가려지지 않도록 화면 맨 앞의 별도 목록에만 추가한다.
            const int foregroundVertices = ImGui::GetForegroundDrawList()->VtxBuffer.Size;
            (void)gui.WantsMouse();
            (void)gui.WantsKeyboard();
            gui.EndFrame();
            RequireControllerUnchanged(controller.GetState(), before);
            Require(!physicsPanel.IsColliderVisible(), "drawing preserves debug checkbox state");
            return foregroundVertices;
        };

        Require(drawFrame(true) > 0, "configured shapes generate foreground wire lines");
        // 표시를 껐다가 다시 켜면 이전 선이 남지 않고 상자와 볼록 껍질이 각각 새 선을 만드는지 확인한다.
        for (const auto& shape : ConfiguredShapes().shapes)
        {
            collider.set(Colliders{{shape}});
            Require(drawFrame(false) == 0, "shape change hidden frame remains empty");
            Require(drawFrame(true) > 0, "each configured shape generates foreground wire lines");
        }
        Require(drawFrame(false) == 0, "hidden overlay emits no foreground lines");
        scene.reset();
        scene = std::make_unique<Scene>(world);
        TransformSystemModule::UpdateWorldTransforms(world);
        Require(!collider.IsValid(), "scene replacement removes the configured collider");
        // 숨김 상태에서 Scene을 바꾸면 이전 물체의 선 자료를 버려야 한다. 다시 켤 때는 100 ms 주기를 기다리지 않고 새 Scene의 충돌 설정을 읽어 선을 만든다.
        Require(drawFrame(false) == 0, "replacement scene hidden frame remains empty");
        Require(drawFrame(true) == 0, "replacement scene does not draw stale collider lines");
        std::cout << "GUI composition and collider visibility checks passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
