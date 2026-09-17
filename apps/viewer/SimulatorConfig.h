#pragma once

#include "Pose.h"

#include <cstddef>
#include <string>

/**
 * @brief Simulation-only runtime configuration.
 *
 * 외부 controller/serial/network 설정은 포함하지 않는다. workspace, target orientation,
 * grasp tolerance, IK parameter처럼 시뮬레이션 자체에 필요한 값만 로컬 `.env`에서 덮어쓸 수 있다.
 */
struct SimulatorConfig
{
    PoseLink::Position3D workspaceMinimum{-0.90F, 0.10F, -0.90F};
    PoseLink::Position3D workspaceMaximum{0.90F, 1.20F, 0.90F};
    PoseLink::Quaternion targetOrientation{};
    float graspPositionToleranceMetres = 0.020F;
    float graspOrientationToleranceRadians = 0.08726646F;
    std::size_t ikMaximumIterations = 100U;
    double ikDamping = 0.080;

    /** Defaults를 먼저 적용한 뒤 존재하는 file의 유효 key만 덮어쓴다. */
    static SimulatorConfig Load(const std::string& path = ".env");
};
