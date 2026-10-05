#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics { class IGripperController; }

namespace grasplink::gui
{

/**
 * @brief Gripper 조작 요청과 Controller 상태를 ImGui 패널로 표시한다.
 * @details 요청 개폐율·속도 입력과 마지막 명령 결과만 보관한다. Controller는 Draw 동안만 빌리며
 * 저장하거나 소유하지 않는다. GUI 입력은 Controller 명령으로 전달하고 실제 움직임과 상태 갱신은
 * Controller가 담당한다. raw 요청값은 장치 code이며 거리·속도·힘의 물리 단위가 아니다.
 */
class GripperPanel
{
public:
    /**
     * @brief 이번 프레임의 Gripper 입력을 받아 명령을 요청하고 상태를 표시한다.
     * @param gripper 이번 호출에 사용할 Controller. 포인터나 참조를 멤버에 저장하지 않는다.
     * @details GuiModule::BeginFrame과 EndFrame 사이에 호출한다. 연속 closureFraction은
     * 유효한 [0,1] 값일 때 백분율로 표시한다. 자유공간 동작만 제공하며 접촉 정지·adaptive grasp는
     * 지원하지 않는다는 기존 안내를 유지한다.
     */
    void Draw(::grasplink::robotics::IGripperController& gripper);

private:
    int requestedClosurePercent = 0;
    int requestedSpeed = 255;
    bool hasGripperResult = false;
    ::grasplink::robotics::Result lastGripperResult{};
};

}
