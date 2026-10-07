#include "gui/panels/RobotPanel.h"

#include <glm/gtc/quaternion.hpp>
#include <imgui.h>

#include <cmath>
#include <string_view>

namespace grasplink::gui
{
namespace
{
glm::dquat Orientation(const std::array<double, 4>& xyzw)
{
    return glm::normalize(glm::dquat{xyzw[3], xyzw[0], xyzw[1], xyzw[2]});
}

double YawDegrees(const std::array<double, 4>& xyzw)
{
    const glm::dquat rotation = Orientation(xyzw);
    return glm::degrees(std::atan2(2.0 * (rotation.w * rotation.y + rotation.x * rotation.z),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)));
}
}

RobotPanelActions RobotPanel::DrawContents(const RobotPanelView& view)
{
    RobotPanelActions actions;
    ImGui::SeparatorText("Pick and place");
    const auto& state = view.state;
    const auto& graspBoxPoseInBase = view.graspBoxPoseInBase;
    const auto& placementPoseInBase = view.placementPoseInBase;
    if (state.valid && state.tcpPoseValid)
        ImGui::Text("TCP: %.3f, %.3f, %.3f m", state.tcpPose.positionMeters[0], state.tcpPose.positionMeters[1], state.tcpPose.positionMeters[2]);
    ImGui::Text("Box X/Z: %.3f, %.3f m", graspBoxPoseInBase.positionMeters[0], graspBoxPoseInBase.positionMeters[2]);
    ImGui::Text("Box yaw: %.1f deg", YawDegrees(graspBoxPoseInBase.orientationXyzw));
    ImGui::Text("Goal X/Z: %.3f, %.3f m", placementPoseInBase.positionMeters[0], placementPoseInBase.positionMeters[2]);
    ImGui::Text("Goal yaw: %.1f deg", YawDegrees(placementPoseInBase.orientationXyzw));
    ImGui::Text("TCP ceilings: %.1f m/s, %.1f rad/s", view.mission.maxLinearVelocityMetersPerSecond, view.mission.maxAngularVelocityRadiansPerSecond);
    const auto& specification = view.specification;
    if (ImGui::BeginTable("Joint positions", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
    {
        ImGui::TableSetupColumn("Joint", ImGuiTableColumnFlags_WidthFixed, 42.0F);
        ImGui::TableSetupColumn("Min / max", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Current", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        for (std::size_t joint = 0; joint < specification.jointCount; ++joint)
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            const std::string_view jointName = specification.joints[joint].name;
            ImGui::TextUnformatted(jointName.data(), jointName.data() + jointName.size());
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.1f / %.1f deg",
                glm::degrees(specification.joints[joint].minPositionRadians),
                glm::degrees(specification.joints[joint].maxPositionRadians));
            ImGui::TableSetColumnIndex(2);
            if (joint < state.jointPositionRadians.size())
                ImGui::Text("%.1f deg", glm::degrees(state.jointPositionRadians[joint]));
            else
                ImGui::TextUnformatted("--");
        }
        ImGui::EndTable();
    }
    ImGui::TextUnformatted("Angle values are current joint positions, not travel percentages.");
    const bool canStart = view.mission.canStart;
    if (canStart && ImGui::Button("Run random pick and place"))
        actions.start = true;
    if (view.mission.paused)
    {
        ImGui::SameLine();
        if (ImGui::Button("Resume task"))
            actions.resume = true;
    }
    else
    {
        ImGui::SameLine();
        if (ImGui::Button("Stop robot"))
            actions.stop = true;
    }
    ImGui::Text("Task: %s", view.mission.paused ? "Paused" : view.mission.stageLabel.data());
    if (view.mission.paused)
        ImGui::Text("Paused at: %s", view.mission.stageLabel.data());
    ImGui::Text("Mission success: %s", view.mission.missionSucceeded ? "YES" : "NO");
    ImGui::Text("Successful missions: %llu", static_cast<unsigned long long>(view.mission.completedCount));
    ImGui::Text("Automatic repeat: %s", view.mission.autoRepeat ? "ON" : "OFF");
    if (state.faultCode == static_cast<std::uint32_t>(robotics::ErrorCode::EnvironmentContact))
        ImGui::TextWrapped("A collision stopped the arm at its last safe pose. The task is retracting to the previous safe height.");
    if (view.mission.hasResult)
    {
        ImGui::Text("Last request: %s", view.mission.lastRequestAccepted ? "accepted" : "failed");
        if (!view.mission.lastMessage.empty())
            ImGui::TextWrapped("%.*s", static_cast<int>(view.mission.lastMessage.size()), view.mission.lastMessage.data());
    }
    return actions;
}
}
