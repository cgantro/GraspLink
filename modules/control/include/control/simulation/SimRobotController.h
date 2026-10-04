#pragma once

#include "control/IRobotController.h"
#include "control/specs/DeviceSpecifications.h"

namespace control::simulation
{

class SimRobotController final : public IRobotController
{
public:
    explicit SimRobotController(const specs::RobotSpecification& specification);

    Result Connect() override;
    void Disconnect() noexcept override;
    [[nodiscard]] bool IsConnected() const noexcept override;

    Result MoveJoint(const JointMoveCommand& command) override;
    Result MoveLinear(const LinearMoveCommand& command) override;
    Result Stop() override;

    [[nodiscard]] RobotState GetState() const override;
    void Update(double dtSeconds) override;

    [[nodiscard]] const specs::RobotSpecification& GetSpecification() const noexcept;

private:
    const specs::RobotSpecification* specification_ = nullptr;
    RobotState state_{};
    JointVector targetPositionRadians_;
    double velocityScale_ = 1.0;
    double accelerationScale_ = 1.0;
    bool connected_ = false;
};

} // namespace control::simulation
