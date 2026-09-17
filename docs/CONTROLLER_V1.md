# Controller v1 Integration Notes

## 구현 범위

Controller v1은 **입력 → USB-UART XYZ command → simulator robot state → UI feedback**
수직 경로만 구현한다. HCR-12A는 simulator 내부에서 6개의 관절과 TCP pose를 계산하며,
controller는 `(x,y,z)`만 보낸다. simulator의 `.env`가 고정 TCP orientation과 workspace를
제공하므로 사용자의 potentiometer 입력은 6-DoF pose의 translation 부분만 정한다.

제외 항목은 ArUco/vision, camera calibration, physical gripper contact, packet loss·jitter
주입, IMU이다. 이 기능들은 v1 transport와 target interface를 재사용하는 후속 확장이지
현재 firmware의 조건이 아니다.

## Zephyr 구성

`embedded/controller`는 C++ source로 controller core를 빌드하고 `CONFIG_CPP=y`를
사용한다. application target에는 `-fno-exceptions`와 `-fno-rtti`를 적용한다. core는
STL container/new/delete를 사용하지 않고 caller-owned interface reference와 static
객체만 사용한다. 따라서 RAM 사용량과 lifetime을 보드 bring-up에서도 추적할 수 있다.

현재 `main.cpp`은 Null sender/output를 등록한 skeleton이다. device binding 정보가 없는
상태에서 GPIO/ADC/I2C/socket을 추측해 초기화하지 않는다. 실제 binding 시에도 아래
data-flow와 ownership을 유지한다.

```text
GPIO ISR --bounded event--> input worker --long press--> SerialTransport sender
ADC driver ----------------> input worker
SerialTransport receiver --> feedback worker/output driver
```

worker가 `TargetInputController`의 유일한 caller가 되어야 한다. output driver도 UI worker
한 곳에서 호출한다. sender/receiver가 별도 thread라면 queue에는 pointer가 아니라
`TargetSnapshot`/`RobotState` 값을 복사해 lifetime coupling을 피한다.

## integration acceptance

1. button/potentiometer로 target을 만든 뒤 long press를 하면 1개의 snapshot이 protocol
   encoder로 전달된다.
2. TargetCommand의 sequence는 controller input core가 아니라 SerialTransport가 늘린다.
3. simulator가 ack target sequence와 state를 보내면 controller는 마지막 사용자 action과
   feedback 상태를 OLED에서 함께 볼 수 있다.
4. duplicate RobotState가 도착해도 success/failure buzzer event는 반복되지 않는다.
5. malformed command/state, USB cable disconnection, I/O timeout은 transport adapter에서
   `TransportError`로 바꾸며 input state 자체를 손상시키지 않는다.

`RobotState.status` adapter는 `Idle -> Idle`, `Tracking -> Moving`,
`Grasped -> GraspSuccess`, `IkUnreachable -> GraspFailed`,
`ProtocolError -> TransportError`로 고정한다. sender가 snapshot을 수락한 즉시 표시하는
`TargetReceived`는 local pending state이며 UART write 완료가 grasp 성공을 뜻하지는 않는다.
