# Robotics Controller Interface

## 목적

실제 장비와 시뮬레이터가 같은 상위 제어 코드를 사용하도록 robotics 계층을 분리한다.

```text
Application / Planner
        |
        v
IRobotController / IGripperController
        |
        +-- backends/simulation
        |
        +-- backends/hardware (future)
```

상위 계층은 Flecs Entity, GLB Node, 제조사 packet/register를 직접 다루지 않는다.

## 디렉터리

```text
modules/robotics/
├─ include/robotics/
│  ├─ core/
│  │  ├─ ControlTypes.h
│  │  ├─ IRobotController.h
│  │  └─ IGripperController.h
│  ├─ models/
│  │  ├─ ModelTypes.h
│  │  ├─ RobotSpecification.h
│  │  ├─ GripperSpecification.h
│  │  ├─ hanwha/Hcr12a.h
│  │  └─ robotiq/TwoF85.h
│  └─ backends/
│     └─ simulation/SimRobotController.h
└─ src/backends/simulation/SimRobotController.cpp
```

namespace root는 `grasplink::robotics`다.

## Model

`models/`는 장치에 고정된 상수의 source of truth다.

HCR-12A:
- J1~J6 name
- bind pivot
- local axis
- min/max angle
- max velocity
- ToolFrame bind 위치

Robotiq 2F-85:
- 0~255 position/speed/force request range
- master closed angle
- outer/inner/tip pivot
- linkage axis
- mimic multiplier

새 로봇은 `models/<manufacturer>/<Model>.h`에 `RobotSpecification`을 추가한다.
Simulation/Hardware backend는 이 specification을 주입받는다.

## Controller contract

`IRobotController`:

```text
Connect
Disconnect
IsConnected
MoveJoint
MoveLinear
Stop
GetState
Update
```

`IGripperController`:

```text
Connect
Disconnect
IsConnected
Activate
Reset
Command
Stop
GetState
Update
```

공통 단위:

```text
Joint angle     rad
Joint velocity  rad/s
Cartesian pos   m
Quaternion      xyzw
Time            s
```

위 quaternion 순서는 `CartesianPose`의 배열 기준이다. 모델/FK의 `QuaternionWxyz`는 wxyz 순서이며, 어댑터가 타입에 맞춰 명시적으로 변환한다.

## Simulation backend

`SimRobotController`는 `RobotSpecification`을 생성자에서 받고 모델의 joint count/limit/max velocity를 그대로 사용한다.
현재 책임은 연결 상태, joint target 검증, 속도 제한 기반 target 추종, Stop, state feedback이다. `accelerationScale`은 command에 보관하지만 시뮬레이터가 가속도 제한이나 ramp를 계산하지 않는다.

`MoveLinear`는 현재 `Unsupported`다. FK는 별도 `RobotKinematics` 계층에서 구현됐지만 controller command와 IK는 연결되지 않았다. FK가 ToolFrame pose를 계산해도 controller `RobotState.tcpPoseValid`는 false이며, TCP feedback을 만들어 내지 않는다.
확인되지 않은 최대 가속도 값은 제조사 사양처럼 임의로 넣지 않는다.

## Viewer adapter

```text
modules/viewer/include/viewer/robotics/RobotTransformAdapter.h
modules/viewer/src/robotics/RobotTransformAdapter.cpp
```

`RobotKinematics`가 `RobotState`를 pose로 바꾸고, Adapter는 그 결과를 Flecs/GLB transform으로 표현한다. Viewer는 FK나 제어 로직을 수행하지 않는다.

## Hardware backend 예정

```text
robotics/backends/hardware/hanwha/
robotics/backends/hardware/robotiq/
```

Hanwha backend는 HCR TCP/IP/Modbus TCP/Rodi adapter를 내부에 숨긴다.
Robotiq backend는 Modbus RTU RS-485 register packing/parsing을 내부에 숨긴다.
