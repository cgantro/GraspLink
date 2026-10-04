# Controller Interface 기준 정리

## 1. 목적

실제 장비와 시뮬레이터가 같은 상위 제어 코드를 사용하도록 Controller 계층을 분리한다.

```text
Application / Planner / IK
        |
        v
IRobotController / IGripperController
        |
        +-- Simulation backend
        |
        +-- Hardware backend
```

상위 계층은 Flecs Entity, GLB Node, Hanwha packet, Robotiq Modbus register를 직접 다루지 않는다.

---

## 2. HCR-12A에서 확인된 외부 제어 조건

한화로보틱스 공식 HCR-12A 제품/카탈로그 기준:

- 6 DOF
- Controller communication: TCP/IP, Modbus TCP
- Max joint speed: J1/J2 130 deg/s, J3~J6 200 deg/s
- 공식 다운로드 페이지에서 HCR 2세대 사용자 매뉴얼과 Rodi Script Programming Guide 제공

현재 공개 웹 인덱스에서는 Rodi Script Guide의 개별 함수 signature까지 안정적으로 확인되지 않았으므로,
`IRobotController`에 제조사 함수명을 그대로 박아 넣지 않는다.

따라서 공통 motion primitive는 다음 두 개만 먼저 둔다.

```text
MoveJoint(J1~J6 absolute target)
MoveLinear(TCP pose target)
```

실제 HCR backend가 구현될 때 정확한 Rodi/TCP command syntax와 응답 protocol을 adapter 내부에서 변환한다.

### 단위 규칙

```text
Joint angle     : rad
Joint velocity  : rad/s
Cartesian pos   : m
Quaternion      : xyzw
Time            : s
```

---

## 3. Robotiq 2F-85에서 확인된 제어 명세

Robotiq 2F-85 공식 Instruction Manual 기준:

### 통신

```text
Protocol        : Modbus RTU
Physical        : RS-485
Default baud    : 115200
Data bits       : 8
Stop bit        : 1
Parity          : None
Default slave   : 9
Command base    : 0x03E8 (1000)
Status base     : 0x07D0 (2000)
```

지원 Modbus function:

```text
FC03 Read Holding Registers
FC04 Read Input Registers
FC16 Preset Multiple Registers
FC23 Read/Write Multiple Registers
```

문서에서는 일반적인 command/status cycle을 약 200 Hz, 최소 5 ms 간격으로 권장한다.

### Command 의미

```text
rACT : activation
rGTO : Go To requested position
rPR  : position request 0~255
       0   = fully open
       255 = fully closed
rSP  : speed request 0~255
rFR  : force request 0~255
```

`rPR`과 실제 opening 관계는 quasi-linear로 설명되며 2F-85의 nominal stroke는 85 mm다.

### Status 의미

```text
gACT : activation status
gGTO : Go-To status
gSTA : activation/state code
gOBJ : object detection
       0 = moving
       1 = contact while opening
       2 = contact while closing
       3 = requested position reached / no retained object

gFLT : fault code
gPR  : requested position echo
gPO  : actual encoder position 0~255
gCU  : motor current, approximately 10 mA/count
```

이 때문에 `IGripperController`는 단순한 `open()/close()`가 아니라
`position/speed/force + status feedback`을 공통 contract로 잡는다.

---

## 4. Interface 파일

```text
modules/control/include/control/ControlTypes.h
modules/control/include/control/IRobotController.h
modules/control/include/control/IGripperController.h
```

### IRobotController

```cpp
Connect()
Disconnect()
IsConnected()
MoveJoint(...)
MoveLinear(...)
Stop()
GetState()
Update(dt)
```

### IGripperController

```cpp
Connect()
Disconnect()
IsConnected()
Activate()
Reset()
Command({rPR, rSP, rFR})
Stop()
GetState()
Update(dt)
```

---

## 5. 구현 예정 backend

### Simulation

```text
SimRobotController
  -> RobotModel / JointState / JointLimit
  -> Fixed Control Loop
  -> Viewer/Flecs Transform 반영

SimGripperController
  -> 2F-85 master q + mimic linkage
  -> 이후 Jolt contact/constraint와 연결
```

현재 `RobotJointController`는 Viewer-side visual bridge이므로 그대로 Hardware abstraction으로 승격하지 않는다.
향후 `SimRobotController` 내부에서 사용하거나 역할을 Transform adapter 수준으로 축소한다.

### Hardware

```text
HanwhaHcrController
  -> HCR TCP/IP / Modbus TCP / Rodi adapter

Robotiq2F85Controller
  -> Modbus RTU RS-485
  -> rACT/rGTO/rPR/rSP/rFR packing
  -> gACT/gSTA/gOBJ/gFLT/gPR/gPO/gCU parsing
```

---

## 6. 지금 인터페이스에 넣지 않은 것

다음은 정확한 vendor command 문서와 Control Loop 설계가 정해진 뒤 추가한다.

- Streaming joint servo command
- Torque/current command
- Hardware E-Stop 제어
- Safety reset/enable sequence
- Digital/Analog Tool I/O
- Trajectory queue/blending
- Vendor-specific fault enum

특히 `Stop()`은 소프트웨어 motion stop 요청이며 물리적인 Emergency Stop 회로를 대체하지 않는다.

---

## 7. 참고 문서

- Hanwha Robotics HCR-12A product/specification
  - https://www.hanwharobotics.com/kr/product/view?menuSeq=2&prdtSeq=79
- Hanwha Robotics download center: HCR 2nd Generation User Manual / Rodi Script Programming Guide
  - https://www.hanwharobotics.com/kr/newsroom/download?menuSeq=85
- Robotiq 2F-85 & 2F-140 Instruction Manual
  - https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_UR_PDF_20200211.pdf
