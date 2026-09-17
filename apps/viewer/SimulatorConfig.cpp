#include "SimulatorConfig.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>

namespace
{
std::string Trim(std::string value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) { return {}; }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1U);
}

bool ParseFloat(const std::string& text, float& out)
{
    char* end = nullptr;
    const float candidate = std::strtof(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0' || !std::isfinite(candidate)) { return false; }
    out = candidate;
    return true;
}

bool ParsePositiveNumber(const std::string& text, std::size_t& out)
{
    char* end = nullptr;
    const unsigned long long candidate = std::strtoull(text.c_str(), &end, 10);
    if (end == text.c_str() || *end != '\0' || candidate == 0ULL) { return false; }
    out = static_cast<std::size_t>(candidate);
    return true;
}
} // namespace

SimulatorConfig SimulatorConfig::Load(const std::string& path)
{
    SimulatorConfig config{};

    const char* explicitPath = std::getenv("GRASPLINK_CONFIG");
    std::ifstream input;
    if (explicitPath != nullptr && *explicitPath != '\0') { input.open(explicitPath); }
    if (!input) { input.clear(); input.open(path); }
    if (!input) { input.clear(); input.open("../" + path); }
    if (!input) { input.clear(); input.open("../../" + path); }
    if (!input) { return config; }

    std::string line;
    while (std::getline(input, line))
    {
        line = Trim(line);
        if (line.empty() || line.front() == '#') { continue; }

        const std::size_t separator = line.find('=');
        if (separator == std::string::npos) { continue; }

        const std::string key = Trim(line.substr(0U, separator));
        const std::string value = Trim(line.substr(separator + 1U));
        float parsed = 0.0F;

        if (key == "SIM_WORKSPACE_MIN_X" && ParseFloat(value, parsed)) { config.workspaceMinimum.x = parsed; }
        else if (key == "SIM_WORKSPACE_MIN_Y" && ParseFloat(value, parsed)) { config.workspaceMinimum.y = parsed; }
        else if (key == "SIM_WORKSPACE_MIN_Z" && ParseFloat(value, parsed)) { config.workspaceMinimum.z = parsed; }
        else if (key == "SIM_WORKSPACE_MAX_X" && ParseFloat(value, parsed)) { config.workspaceMaximum.x = parsed; }
        else if (key == "SIM_WORKSPACE_MAX_Y" && ParseFloat(value, parsed)) { config.workspaceMaximum.y = parsed; }
        else if (key == "SIM_WORKSPACE_MAX_Z" && ParseFloat(value, parsed)) { config.workspaceMaximum.z = parsed; }
        else if (key == "SIM_TARGET_QW" && ParseFloat(value, parsed)) { config.targetOrientation.w = parsed; }
        else if (key == "SIM_TARGET_QX" && ParseFloat(value, parsed)) { config.targetOrientation.x = parsed; }
        else if (key == "SIM_TARGET_QY" && ParseFloat(value, parsed)) { config.targetOrientation.y = parsed; }
        else if (key == "SIM_TARGET_QZ" && ParseFloat(value, parsed)) { config.targetOrientation.z = parsed; }
        else if (key == "SIM_GRASP_POSITION_TOLERANCE_M" && ParseFloat(value, parsed) && parsed >= 0.0F)
        {
            config.graspPositionToleranceMetres = parsed;
        }
        else if (key == "SIM_GRASP_ORIENTATION_TOLERANCE_DEG" && ParseFloat(value, parsed) && parsed >= 0.0F)
        {
            config.graspOrientationToleranceRadians = parsed * 0.01745329252F;
        }
        else if (key == "SIM_IK_MAX_ITERATIONS")
        {
            std::size_t count = 0U;
            if (ParsePositiveNumber(value, count)) { config.ikMaximumIterations = count; }
        }
        else if (key == "SIM_IK_DAMPING")
        {
            char* end = nullptr;
            const double candidate = std::strtod(value.c_str(), &end);
            if (end != value.c_str() && *end == '\0' && std::isfinite(candidate) && candidate > 0.0)
            {
                config.ikDamping = candidate;
            }
        }
    }

    if (config.workspaceMinimum.x > config.workspaceMaximum.x) { std::swap(config.workspaceMinimum.x, config.workspaceMaximum.x); }
    if (config.workspaceMinimum.y > config.workspaceMaximum.y) { std::swap(config.workspaceMinimum.y, config.workspaceMaximum.y); }
    if (config.workspaceMinimum.z > config.workspaceMaximum.z) { std::swap(config.workspaceMinimum.z, config.workspaceMaximum.z); }

    return config;
}
