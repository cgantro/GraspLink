# GraspLink ESP32 Serial Transport

1차 데모는 Wi-Fi/UDP가 아니라 ESP32-DevKitC V4의 micro-USB 케이블을 통한 USB-UART0
양방향 통신을 사용한다. USB 케이블 하나가 전원, firmware flash, PC simulator 통신을
함께 제공하므로 PC에 Wi-Fi 장치나 공유기가 없어도 된다.

    Button / potentiometer
            -> TargetInputWorkflow
            -> SerialTransport
            -> UART0 -> USB-to-UART bridge -> PC simulator
            <- UART0 <- USB-to-UART bridge <-
            <- RobotState -> FeedbackPresenter -> OLED / LED / buzzer

## Wire contract

SerialTransport는 기존 GLNK v1 packet layout을 바꾸지 않는다.

- ESP32 → PC: TargetCommand, 28 byte. Long press 순간의 target ID와 XYZ metre
  snapshot에 sender-local wrapping sequence를 붙인다.
- PC → ESP32: RobotState, 20 byte. acknowledgement sequence와 grasp/IK status를
  되돌려준다.
- 모든 integer와 float bit pattern은 big-endian이다. C/C++ struct memory와 padding은
  절대 전송하지 않는다.
- UART는 datagram 경계가 없는 stream이다. 수신기는 GLNK magic 및 type을 사용해 frame
  길이를 복원하고, ROM boot text 또는 유실된 byte 뒤에는 다음 magic으로 재동기화한다.
- RobotState sequence는 modular half-range 비교를 한다. duplicate, old, 정확히
  2^31만큼 떨어진 ambiguous state는 UI/buzzer에 반영하지 않는다.

## UART0 주의사항

DevKitC V4의 USB bridge는 UART0에 연결된다. board DTS의 기본 rate는 보통 115200 baud이며
PC는 동일한 COM port를 **115200, 8 data bits, no parity, 1 stop bit, no flow control**로
연다. PC 프로그램은 terminal text protocol이 아니라 raw GLNK byte stream을 read/write해야
한다.

prj.conf에서 CONFIG_UART_CONSOLE, CONFIG_PRINTK, CONFIG_LOG를 모두 끈다. UART
console text가 binary packet 중간에 삽입되면 receiver가 손상된 frame으로 처리하기 때문이다.
flash 직후 ESP32 ROM이 출력하는 아주 이른 boot text는 허용된다. receiver가 magic을 탐색해
무시한다.

## Ownership 및 실행 context

SerialTransport는 static storage로 만들고 FeedbackPresenter를 caller-owned reference로
받는다. 내부 packet buffer는 최대 28 byte 고정 배열이며 heap allocation, exception, RTTI를
쓰지 않는다. Send와 Poll은 worker/thread context에서만 호출한다. 특히 uart_poll_out은
전송 가능한 FIFO 공간을 기다릴 수 있으므로 GPIO ISR에서 실행하면 안 된다.

현재 main.cpp은 5 ms 간격으로 Poll만 실행한다. GPIO/ADC/OLED 물리 binding이 추가되면
button ISR은 bounded event만 enqueue하고 worker가 다음 API를 호출한다.

    workflow.OnAdcSample(rawAdc);
    workflow.OnButtonReleased(heldMilliseconds); // long press면 SerialTransport::Send

UART0의 GPIO1(TX)/GPIO3(RX)는 USB serial link용이므로 다른 module에 배정하지 않는다.
