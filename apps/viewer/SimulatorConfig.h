#pragma once

#include "Pose.h"

#include <cstdint>
#include <string>

/**
 * @brief Simulator process의 외부 환경 계약이다.
 *
 * 로봇의 좌표/IK 상수와 개발자의 LAN endpoint를 소스 코드에서 분리한다. `.env`는
 * `KEY=VALUE` 한 줄씩만 해석하는 의도적으로 작은 형식이다. Wi-Fi password 같은
 * secret을 C++ binary나 repository에 남기지 않으며, 알 수 없는 key는 앞으로의
 * 설정 확장을 위해 무시한다.
 */
struct SimulatorConfig
{
    // USB-to-UART bridge creates a bidirectional virtual COM port. This v1
    // setting deliberately has no LAN address or Wi-Fi credential.
    std::string serialPort = "COM3";
    std::uint32_t serialBaudRate = 115200U;
    PoseLink::Position3D workspaceMinimum{-0.90F, 0.10F, -0.90F};
    PoseLink::Position3D workspaceMaximum{0.90F, 1.20F, 0.90F};
    PoseLink::Quaternion fixedTcpOrientation{};
    float graspPositionToleranceMetres = 0.020F;
    float graspOrientationToleranceRadians = 0.08726646F;
    std::size_t ikMaximumIterations = 100U;
    double ikDamping = 0.080;

    /** Defaults를 먼저 적용한 뒤 존재하는 file의 유효 key만 덮어쓴다. */
    static SimulatorConfig Load(const std::string& path = ".env");
};
