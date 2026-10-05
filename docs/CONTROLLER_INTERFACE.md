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
│  ├─ backends/
│  │  └─ simulation/SimRobotController.h, SimGripperController.h
│  └─ kinematics/RobotKinematics.h, GripperKinematics.h
└─ src/                            # simulation backend / kinematics 구현
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

`SimGripperController`는 `GripperSpecification`을 빌리고 `SimGripperMotionSettings`를 값으로 보관한다. 연결 시 열린 위치로 초기화하며 활성화 뒤 유효한 raw 위치·속도·힘 요청을 수락한다. `GripperState.closureFraction`은 0=열림, 1=nominal closed인 연속 위치다. `valid`와 `closureFractionValid`를 함께 확인하며 `actualPosition` raw feedback은 표시용으로 반올림한다. `GripperMode`는 연결·활성화·이동·정지를 구분하고 `objectStatus`는 이동/목표 도달 분류를 제공한다.

위치는 raw 범위에서 fraction으로 선형 대응한다. raw speed는 기본 master 각속도 0.1..1.0 rad/s에 선형 대응하며 raw 0도 가장 느린 양의 속도다. 이 매핑과 속도 범위는 시뮬레이션 가정이며 제조사 보정식·속도 사양이 아니다. 유한한 양의 dt에서 목표를 추종하고 정확히 멈춘다. Stop은 현재 위치와 요청 echo를 유지한 채 `Stopped`로 전환하며, Reset은 현재 위치를 유지하고 비활성화한다. 중간 위치 Stop을 목표 도달로 보고하지 않는다.

`forceRequest`는 범위만 검사한다. 전류·힘·접촉 판정·접촉 시 정지·파지는 계산하지 않으며 `currentValid=false`다. 목표 도달은 자유공간 기구학 위치 도달을 뜻한다. 세부 상태 전이는 [그리퍼 런타임 설계](GRIPPER_RUNTIME_DESIGN.md)를 참고한다.

## Viewer adapter

```text
modules/viewer/include/viewer/robotics/RobotTransformAdapter.h
modules/viewer/src/robotics/RobotTransformAdapter.cpp
modules/viewer/include/viewer/robotics/GripperTransformAdapter.h
modules/viewer/src/robotics/GripperTransformAdapter.cpp
```

`RobotKinematics`가 `RobotState`를 pose로 바꾸고, Adapter는 그 결과를 Flecs/GLB transform으로 표현한다. Viewer는 FK나 제어 로직을 수행하지 않는다.

`GripperKinematics`는 분기형 여섯 관절의 master/mimic Local 회전 변화만 계산한다. `GripperTransformAdapter`는 저장한 bind 회전에 변화량을 오른쪽으로 곱하고 원본 Local 위치·크기·장착 변환을 보존한다. 앱은 4 ms마다 두 Controller 갱신 → 팔·그리퍼 자세 적용 → World 변환 갱신 → Jolt step → World 변환 재갱신 순서를 연결한다. 기존 일곱 그리퍼 proxy는 관절 자식이므로 같은 World 변환을 따른다. GUI는 `IGripperController`에 요청을 보내고 상태 복사본을 표시한다.

## Hardware backend 예정

```text
robotics/backends/hardware/hanwha/
robotics/backends/hardware/robotiq/
```

Hanwha backend는 HCR TCP/IP/Modbus TCP/Rodi adapter를 내부에 숨긴다.
Robotiq backend는 Modbus RTU RS-485 register packing/parsing을 내부에 숨긴다.
