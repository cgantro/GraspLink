#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics
{

/**
 * @brief Hardware/Simulation gripper backend가 공통으로 구현할 인터페이스.
 *
 * 특정 register packing이나 GLB finger joint를 상위 계층에 노출하지 않는다.
 */
class IGripperController
{
public:
    virtual ~IGripperController() = default;

    virtual Result Connect() = 0;
    virtual void Disconnect() noexcept = 0;
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    virtual Result Activate() = 0;
    virtual Result Reset() = 0;
    virtual Result Command(const GripperCommand& command) = 0;
    virtual Result Stop() = 0;

    [[nodiscard]] virtual GripperState GetState() const = 0;
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
