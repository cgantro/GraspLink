#include "gui/panels/GripperPanel.h"

#include "robotics/core/IGripperController.h"

#include <imgui.h>

#include <cmath>
#include <cstdint>

namespace grasplink::gui
{
namespace
{
const char* GripperModeName(grasplink::robotics::GripperMode mode)
{
    using grasplink::robotics::GripperMode;
    switch (mode)
    {
    case GripperMode::Disconnected: return "Disconnected";
    case GripperMode::Inactive: return "Inactive";
    case GripperMode::Idle: return "Idle";
    case GripperMode::Moving: return "Moving";
    case GripperMode::Stopped: return "Stopped";
    case GripperMode::Fault: return "Fault";
    }
    return "Unknown";
}

const char* GripperObjectStatusName(grasplink::robotics::GripperObjectStatus status)
{
    using grasplink::robotics::GripperObjectStatus;
    switch (status)
    {
    case GripperObjectStatus::Moving: return "No contact / target not reached";
    case GripperObjectStatus::ContactWhileOpening: return "Contact while opening";
    case GripperObjectStatus::ContactWhileClosing: return "Contact while closing";
    case GripperObjectStatus::AtRequestedPosition: return "At requested position";
    }
    return "Unknown";
}

const char* ErrorCodeName(grasplink::robotics::ErrorCode code)
{
    using grasplink::robotics::ErrorCode;
    switch (code)
    {
    case ErrorCode::None: return "Success";
    case ErrorCode::NotConnected: return "Not connected";
    case ErrorCode::InvalidCommand: return "Invalid command";
    case ErrorCode::Busy: return "Busy";
    case ErrorCode::Fault: return "Fault";
    case ErrorCode::Unsupported: return "Unsupported";
    case ErrorCode::TransportError: return "Transport error";
    }
    return "Unknown";
}

}

void GripperPanel::Draw(grasplink::robotics::IGripperController& gripper)
{
    using grasplink::robotics::GripperCommand;
    ImGui::SetNextWindowPos(ImVec2(20, 160), ImGuiCond_FirstUseEver);
    ImGui::Begin("Gripper control");
    const auto submitClosure = [&]()
    {
        GripperCommand command;
        command.positionRequest = static_cast<std::uint8_t>(std::lround(
            static_cast<double>(requestedClosurePercent) * 255.0 / 100.0));
        command.speedRequest = static_cast<std::uint8_t>(requestedSpeed);
        command.forceRequest = 128;
        lastGripperResult = gripper.Command(command);
        hasGripperResult = true;
    };
    ImGui::Text("Connection: %s", gripper.IsConnected() ? "Connected" : "Disconnected");

    if (ImGui::Button("Activate"))
    {
        lastGripperResult = gripper.Activate();
        hasGripperResult = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset"))
    {
        lastGripperResult = gripper.Reset();
        hasGripperResult = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop"))
    {
        lastGripperResult = gripper.Stop();
        hasGripperResult = true;
    }

    if (ImGui::Button("Open"))
    {
        requestedClosurePercent = 0;
        submitClosure();
    }
    ImGui::SameLine();
    if (ImGui::Button("Close"))
    {
        requestedClosurePercent = 100;
        submitClosure();
    }

    ImGui::SliderInt("Requested closure (%)", &requestedClosurePercent, 0, 100);
    ImGui::SliderInt("Speed request (raw)", &requestedSpeed, 0, 255);
    ImGui::TextUnformatted("Force request: 128 (fixed; effect is not simulated)");
    if (ImGui::Button("Move to requested closure"))
        submitClosure();

    const grasplink::robotics::GripperState state = gripper.GetState();
    if (state.valid)
    {
        ImGui::Text("Mode: %s", GripperModeName(state.mode));
        ImGui::Text("Activated: %s", state.activated ? "Yes" : "No");
        ImGui::Text("Actual position (raw): %u", static_cast<unsigned int>(state.actualPosition));
        if (state.closureFractionValid && std::isfinite(state.closureFraction) &&
            state.closureFraction >= 0.0 && state.closureFraction <= 1.0)
            ImGui::Text("Actual closure: %.1f%%", state.closureFraction * 100.0);
        else
            ImGui::TextUnformatted("Actual closure: unavailable");
        ImGui::Text("Object status: %s", GripperObjectStatusName(state.objectStatus));
        if (state.currentValid)
            ImGui::Text("Current (raw): %u", static_cast<unsigned int>(state.currentRaw));
        else
            ImGui::TextUnformatted("Current: unavailable");
    }
    else
    {
        ImGui::TextUnformatted("Gripper state: unavailable");
        ImGui::TextUnformatted("Actual position: unavailable");
        ImGui::TextUnformatted("Actual closure: unavailable");
        ImGui::TextUnformatted("Object status: unavailable");
        ImGui::TextUnformatted("Current: unavailable");
    }

    if (hasGripperResult)
    {
        ImGui::Text("Last result: %s", ErrorCodeName(lastGripperResult.code));
        if (!lastGripperResult.message.empty())
            ImGui::TextWrapped("%s", lastGripperResult.message.c_str());
    }
    else
    {
        ImGui::TextUnformatted("Last result: no command sent");
    }
    ImGui::TextWrapped("Free-space linkage motion; contact-driven stopping and adaptive grasping are unavailable.");
    ImGui::End();
}

}
