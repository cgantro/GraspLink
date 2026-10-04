# Controller Interface 기준 정리

## 1. 목적

실제 장비와 시뮬레이터가 같은 상위 제어 코드를 사용하도록 Controller 계층을 분리한다.

```text
Application / Planner / IK
        |
        v
IRobotController / IGripperController
        |
        +-- SimRobotController
        |
        +-- Hardware backend (future)
```

상위 계층은 Flecs Entity, GLB Node, Hanwha packet, Robotiq Modbus register를 직접 다루지 않는다.

---

## 2. 현재 코드 구조

```text
modules/control/
├─ include/control/
│  ├─ ControlTypes.h
│  ├─ IRobotController.h
│  ├─ IGripperController.h
│  ├─ specs/
│  │  ├─ DeviceSpecifications.h
│  │  ├─ Hcr12aSpecification.h
│  │  └─ Robotiq2F85Specification.h
│  └─ simulation/
│     └─ SimRobotController.h
└─ src/simulation/
   └─ SimRobotController.cpp

modules/viewer/
├─ include/robot/RobotTransformAdapter.h
└─ src/robot/RobotTransformAdapter.cpp
```

`modules/control`은 Viewer/Flecs/GLM에 의존하지 않는다.
`RobotTransformAdapter`만 Viewer Entity를 알고 있으며 `RobotState -> GLB Joint Transform` 변환만 담당한다.

---

## 3. Model Specification

모델별 상수는 `modules/control/include/control/specs/` 한 곳에서 관리한다.

`DeviceSpecifications.h`는 공통 schema만 정의한다.

```text
RobotSpecification
└─ JointSpecification[]
   ├─ name
   ├─ bindPivotMeters
   ├─ axis
   ├─ min/max position
   └─ max velocity

GripperSpecification
└─ GripperJointSpecification[]
   ├─ name
   ├─ bindPivotMeters
   ├─ axis
   ├─ master multiplier
   └─ joint range
```

현재 모델별 파일:

```text
Hcr12aSpecification.h
- J1~J6 name
- CAD/controller-ready GLB bind pivot
- joint axis
- angle limit
- max velocity
- ToolFrame bind position

Robotiq2F85Specification.h
- rPR/rSP/rFR range
- nominal closed master q = 0.7929 rad
- outer/inner/tip joint pivot
- mimic multiplier
- joint range
```

새 로봇을 추가할 때 기존 Controller 코드를 복사하지 않는다. 예를 들어 `NewRobotSpecification.h`를 추가해 `RobotSpecification`만 정의한 뒤 동일한 `SimRobotController`와 `RobotTransformAdapter`에 전달한다.

---

## 4. Joint 개수 확장

`ControlTypes.h`는 더 이상 6축 고정 `std::array`를 사용하지 않는다.

```cpp
using JointVector = std::vector<double>;
```

Joint 개수는 `RobotSpecification::jointCount`가 결정한다.
따라서 4축, 6축, 7축 로봇을 추가해도 `IRobotController` interface 자체를 수정할 필요가 없다.

---

## 5. SimRobotController 책임

`SimRobotController : IRobotController`는 Viewer를 알지 않는다.

현재 구현:

```text
Connect / Disconnect
MoveJoint
Joint count 검증
Joint position limit 검증
Joint max velocity 기반 target 추종
Stop
RobotState feedback
```

아직 구현하지 않은 기능:

```text
MoveLinear -> Unsupported
Acceleration shaping -> verified acceleration limit 확보 후 적용
TCP pose -> FK 계층 연결 후 valid 처리
```

알 수 없는 HCR acceleration 값을 임의로 생성하지 않는다.

---

## 6. RobotTransformAdapter 책임

기존 `RobotJointController`를 제거하고 Viewer-only `RobotTransformAdapter`로 역할을 명확히 했다.

```text
SimRobotController
      |
      | RobotState
      v
RobotTransformAdapter
      |
      v
Flecs / GLB J1~Jn Entity
```

Adapter는 다음만 한다.

- `RobotSpecification`의 joint name으로 GLB Entity 검색
- specification의 axis 사용
- `RobotState::jointPositionRadians`를 Entity local rotation으로 반영

Joint limit, target tracking, IK/FK는 Adapter 책임이 아니다.

현재 controller-ready GLB contract는 J1~J6 Joint node의 bind rotation이 identity인 것이다. Adapter는 이 조건이 깨지면 초기화 중 오류를 발생시킨다.

---

## 7. HCR-12A에서 확인된 외부 제어 조건

한화로보틱스 공식 HCR-12A 제품/카탈로그 기준:

- 6 DOF
- Controller communication: TCP/IP, Modbus TCP
- Max joint speed: J1/J2 130 deg/s, J3~J6 200 deg/s
- 공식 다운로드 페이지에서 HCR 2세대 사용자 매뉴얼과 Rodi Script Programming Guide 제공

공통 단위:

```text
Joint angle     : rad
Joint velocity  : rad/s
Cartesian pos   : m
Quaternion      : xyzw
Time            : s
```

실제 Hardware backend에서는 vendor 단위를 이 경계에서 공통 단위로 변환한다.

---

## 8. Robotiq 2F-85 제어 명세

```text
Protocol        : Modbus RTU
Physical        : RS-485
Default baud    : 115200
Default slave   : 9
Command base    : 0x03E8
Status base     : 0x07D0
```

Command:

```text
rACT : activation
rGTO : Go To
rPR  : position request 0~255
rSP  : speed request 0~255
rFR  : force request 0~255
```

Status:

```text
gACT / gGTO / gSTA / gOBJ
gFLT / gPR / gPO / gCU
```

현재는 Gripper control 구현을 미루고 specification과 interface만 유지한다.
실제 free-space mimic motion과 contact-dependent under-actuated motion은 이후 `SimGripperController + Jolt` 단계에서 연결한다.

---

## 9. 다음 단계

```text
RobotSpecification
      ↓
SimRobotController
      ↓
Fixed Control Loop
      ↓
RobotState
      ↓
RobotTransformAdapter
      ↓
Flecs / GLB
```

그 다음 순서:

1. acceleration limit 자료 확정 또는 simulator policy 정의
2. Fixed Control Loop를 render dt와 분리
3. FK를 RobotState/TCP feedback에 연결
4. DLS IK 결과를 MoveJoint target으로 연결
5. SimGripperController 구현
6. Jolt constraint/contact 연결
7. Hardware backend 추가

---

## 10. 참고 문서

- Hanwha Robotics HCR-12A product/specification
  - https://www.hanwharobotics.com/kr/product/view?menuSeq=2&prdtSeq=79
- Hanwha Robotics download center
  - https://www.hanwharobotics.com/kr/newsroom/download?menuSeq=85
- Robotiq 2F-85 & 2F-140 Instruction Manual
  - https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_UR_PDF_20200211.pdf
