#pragma once

#include "robotics/core/IRobotController.h"
#include "robotics/models/RobotSpecification.h"

namespace grasplink::robotics::backends::simulation
{

class SimRobotController final : public IRobotController
{
public:
    explicit SimRobotController(const models::RobotSpecification& specification);

    Result Connect() override;
    void Disconnect() noexcept override;
    [[nodiscard]] bool IsConnected() const noexcept override;

    Result MoveJoint(const JointMoveCommand& command) override;
    Result MoveLinear(const LinearMoveCommand& command) override;
    Result Stop() override;

    [[nodiscard]] RobotState GetState() const override;
    void Update(double dtSeconds) override;

    [[nodiscard]] const models::RobotSpecification& GetSpecification() const noexcept;

private:
    const models::RobotSpecification* specification_ = nullptr;
    RobotState state_{};
    JointVector targetPositionRadians_;
    double velocityScale_ = 1.0;
    double accelerationScale_ = 1.0;
    bool connected_ = false;
};

} // namespace grasplink::robotics::backends::simulation
