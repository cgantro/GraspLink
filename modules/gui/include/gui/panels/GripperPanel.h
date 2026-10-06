#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics { class IGripperController; }

namespace grasplink::gui
{

/**
 * @brief 그리퍼를 얼마나 열거나 닫을지 조작하고 Controller가 보고한 상태를 보여준다.
 * @details 패널은 요청한 개폐 비율과 속도, 마지막 명령의 성공 여부만 저장한다. Controller는 Draw()가 실행되는 동안만 빌려 쓰며 패널이 소유하지 않는다.
 * GUI 조작은 Controller에 명령으로 전달되고 실제 손가락 이동과 상태 변경은 Controller가 수행한다. raw 요청값은 장치가 쓰는 정수 명령 코드이므로 거리·속도·힘의 물리 단위가 아니다.
 */
class GripperPanel
{
public:
    /**
     * @brief 현재 프레임의 사용자 입력을 Controller 명령으로 보내고 상태를 화면에 그린다.
     * @param gripper 이번 호출 중에만 사용하는 Controller 참조. 패널은 이 참조를 저장하지 않는다.
     * @details GuiModule::BeginFrame()과 EndFrame() 사이에 호출한다. Controller의 개폐 비율이 0에서 1 사이이면 백분율로 보여 준다.
     * 현재는 물체에 닿지 않은 상태에서 여닫기만 지원하며, 접촉 시 멈추거나 물체 모양에 맞춰 잡는 동작은 지원하지 않는다고 안내한다.
     */
    void Draw(::grasplink::robotics::IGripperController& gripper);

private:
    int requestedClosurePercent = 0;
    int requestedSpeed = 255;
    bool hasGripperResult = false;
    ::grasplink::robotics::Result lastGripperResult{};
};

}
