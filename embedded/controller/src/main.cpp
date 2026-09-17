#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "grasplink/controller/ControllerInterfaces.h"
#include "grasplink/controller/FeedbackPresenter.h"
#include "grasplink/controller/SerialTransport.h"
#include "grasplink/controller/TargetInputController.h"

namespace
{
/*
 * 실제 board binding 전에도 firmware의 ownership을 고정한다.
 *
 * 이 객체들은 static storage duration을 가지므로 heap allocation, startup order가
 * 불명확한 service locator, hidden background thread가 없다. board overlay와 실제
 * GPIO/ADC/I2C/Wi-Fi 정보가 결정되면 Null adapter만 concrete adapter로 교체한다.
 */
constexpr grasplink::controller::TargetInputConfiguration kInputConfiguration{};
static grasplink::controller::TargetInputController gInput{kInputConfiguration};
static grasplink::controller::NullFeedbackOutput gFeedbackOutput{};
static grasplink::controller::FeedbackPresenter gFeedbackPresenter{gFeedbackOutput};
/*
 * ESP32-DevKitC V4의 micro-USB bridge는 UART0에 연결된다. zephyr_console chosen node는
 * board DTS가 지정한 이 물리 UART controller를 가져오기 위한 stable reference다.
 * prj.conf에서 console/log backend를 껐으므로 binary GLNK와 text log가 섞이지 않는다.
 */
static const device* const gConsoleUart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static grasplink::controller::SerialTransport gSerialTransport{gConsoleUart, gFeedbackPresenter};
static grasplink::controller::TargetInputWorkflow gInputWorkflow{gInput, gSerialTransport};
} // namespace

int main()
{
    /*
     * 최종 board binding의 worker path:
     *
 * GPIO ISR -> bounded event queue -> worker
 * ADC sample -> gInputWorkflow.OnAdcSample(raw)
 * button release -> gInputWorkflow.OnButtonReleased(duration)
 * USB-UART RobotState -> gSerialTransport.Poll() -> FeedbackPresenter
     *
     * 현재 board overlay가 없으므로 GPIO/ADC/OLED는 의도적으로 초기화하지 않는다.
     * 반면 UART0 transport는 DevKitC의 onboard USB bridge를 사용하므로 물리 peripheral
     * 추가 없이도 PC simulator와 binary frame을 주고받을 수 있다.
     */
    while (true) {
        /*
         * UART Poll은 non-blocking이라 현재 FIFO에 있는 byte만 처리한다. 이후 GPIO/ADC
         * worker가 long press를 workflow로 전달하면 같은 static transport가 송신한다.
         */
        gSerialTransport.Poll();
        k_sleep(K_MSEC(5));
    }

    return 0;
}
