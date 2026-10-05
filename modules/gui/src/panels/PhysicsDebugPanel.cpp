#include "gui/panels/PhysicsDebugPanel.h"

#include <imgui.h>

namespace grasplink::gui
{

void PhysicsDebugPanel::Draw()
{
    // 패널 문구로 선이 실제 Body 대신 ECS 설정의 근사임을 구분한다.
    ImGui::Begin("Configured colliders");
    ImGui::Checkbox("Show configured colliders", &visible_);
    ImGui::TextUnformatted("ECS approximation; refresh: 100 ms");
    ImGui::TextUnformatted("X-ray: Robot orange, Gripper purple, Dynamic cyan, Floor green");
    ImGui::End();
}

bool PhysicsDebugPanel::IsColliderVisible() const
{
    return visible_;
}

}
