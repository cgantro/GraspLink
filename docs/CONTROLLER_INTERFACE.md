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
│  ├─ kinematics/RobotKinematics.h, RobotInverseKinematics.h, GripperKinematics.h
│  └─ planning/                    # 경로 계획 계약과 정책
└─ src/                             # backend, kinematics, planning 구현
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
MovePose
BeginPosePlanning
BeginPosePlanningWithLinearContinuation
MoveLinearPath
BeginLinearPathPlanning
AdvanceMotionPlanning
IsMotionPlanning
TakeMotionPlanningResult
Stop
GetState
Update
```

`Begin*Planning`은 계획 시작을 요청한다. 증분 계획을 지원하는 Simulation은 `AdvanceMotionPlanning(workBudget)`로
작업 단위를 진행하고, `IsMotionPlanning()`과 `TakeMotionPlanningResult()`로 진행 중 여부와 완료 결과를 읽는다.
기본 인터페이스 구현은 해당 동기 명령으로 되돌아가거나 `Unsupported`를 반환할 수 있으므로 호출자는 구체 backend의
기능 지원을 고려해야 한다. 비동기 계획의 완료는 로봇 동작 완료와 구분되며, 동작 목표 도달은 `GetState()`로 확인한다.

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
Controller는 관절 목표를 검증하고, 관절 속도·Simulation 가속 제한과 TCP 목표 IK, 표본 기반 직선 경로 실행, Stop, 모델 기반 상태 제공을 담당한다. `MoveJoint` 가속도는 모델 사양에 없는 제조사 한계를 가정하지 않도록 최대 관절 속도에서 유도한 Simulation 정책으로 제한한다. 기본 `accelerationScale=1.0`은 최대 속도까지 0.20초 동안 가속하는 설정이다.

`MovePose`는 현재 관절각을 seed로 DLS IK를 풀고 관절 공간 경로를 검사하는 MoveJ 명령이다. `BeginPosePlanning`은 같은 목표를 프레임별 작업으로 나누어 계산한다. 직접 관절 경로가 충돌하면 제한된 RRT-Connect를 시도하고, 결과 경로를 다시 검사한다. 이 경로는 TCP 직선을 보장하지 않는다. `BeginPosePlanningWithLinearContinuation`은 접근 자세와 그 뒤의 선형 continuation을 함께 검사하고, 둘 다 유효할 때만 접근 동작을 시작한다. 접근 동작이 끝난 뒤 호출자가 continuation을 요청해야 한다.

`MoveLinear`은 목표 하나를 `MoveLinearPath`로 감싼다. MoveL은 직전 표본의 IK 해를 다음 seed로 재사용하고, 이어갈 해가 없을 때만 대체 분기를 찾는다. 경로 표본과 관절 경로의 검사에 실패하면 MoveL을 거부하며 MoveJ로 자동 대체하지 않는다. 고정 갱신에서는 사전 검증한 관절 표본을 보간한다. TCP의 직선·회전 속도와 가속도 제한은 전체 경로 프로파일에 적용한다. 표본 추종이 속도 제한을 넘는 특이 자세에서만 TCP를 거의 고정하는 재정렬 IK를 시도하며, 이 이동도 관절 경로와 TCP 오차를 검사한다. 목표는 Robot base 좌표 기준이다. 실패와 waypoint 처리 계약은 [로봇 이동과 파지](ROBOT_MOTION_AND_GRASP.md)를 참고한다.

관절 최대 속도는 모델 데이터에서 가져오지만 제조사가 가속도를 제공하지 않는 경우 Simulation 정책으로 제한한다. 문서의 0.20초 기본 가속 시간은 이 정책값이며 장비 사양이나 측정 결과가 아니다.

`SimGripperController`는 `GripperSpecification`을 빌리고 `SimGripperMotionSettings`를 값으로 보관한다. 연결 시 열린 위치로 초기화하며 활성화 뒤 유효한 raw 위치·속도·힘 요청을 수락한다. `GripperState.closureFraction`은 0=열림, 1=nominal closed인 연속 위치다. `valid`와 `closureFractionValid`를 함께 확인하며 `actualPosition` raw feedback은 표시용으로 반올림한다. `GripperMode`는 연결·활성화·이동·정지를 구분하고 `objectStatus`는 이동/목표 도달 분류를 제공한다.

위치는 raw 범위에서 fraction으로 선형 대응한다. raw speed는 기본 master 각속도 0.1..1.0 rad/s에 선형 대응하며 raw 0도 가장 느린 양의 속도다. 이 매핑과 속도 범위는 시뮬레이션 가정이며 제조사 보정식·속도 사양이 아니다. 유한한 양의 dt에서 목표를 추종하고 정확히 멈춘다. Stop은 현재 위치와 요청 echo를 유지한 채 `Stopped`로 전환하며, Reset은 현재 위치를 유지하고 비활성화한다. 중간 위치 Stop을 목표 도달로 보고하지 않는다.

`forceRequest`는 범위만 검사하고 전류와 실제 힘은 계산하지 않으므로 `currentValid=false`다. 물리 접촉은 `GripperGraspAdapter`가 전달하며 Controller는 접촉 시 현재 위치에서 멈추고 `ContactWhileOpening` 또는 `ContactWhileClosing`을 보고한다. 양쪽 손끝이 같은 Dynamic 물체를 서로 반대 방향에서 만지면 adapter가 고정 constraint로 물체를 유지한다. 실제 마찰 파지력과 개별 손가락 적응은 계산하지 않는다. 세부 상태 전이는 [그리퍼 런타임 설계](GRIPPER_RUNTIME_DESIGN.md)를 참고한다.

## Viewer adapter

```text
modules/simulation/include/simulation/robotics/RobotTransformAdapter.h
modules/simulation/src/robotics/RobotTransformAdapter.cpp
modules/simulation/include/simulation/robotics/GripperTransformAdapter.h
modules/simulation/src/robotics/GripperTransformAdapter.cpp
```

`RobotKinematics`가 `RobotState`를 pose로 바꾸고, `RobotTransformAdapter`는 그 결과를 Flecs/GLB transform으로 표현한다. Viewer는 FK나 제어 로직을 수행하지 않는다. 그리퍼의 기구학은 robotics 모듈이 계산하고, simulation adapter가 GLB 관절과 충돌 프록시 변환에 반영한다.

`GripperKinematics`는 분기형 여섯 관절의 master/mimic Local 회전 변화만 계산한다. `GripperTransformAdapter`는 저장한 bind 회전에 변화량을 오른쪽으로 곱하고 원본 Local 위치·크기·장착 변환을 보존한다. 앱은 4 ms마다 파지 해제 확인 → 두 Controller 갱신 → 팔·그리퍼 자세 적용 → World 변환 갱신 → Jolt step → 접촉 피드백과 파지 연결 → World 변환 재갱신 순서를 연결한다. 기존 일곱 그리퍼 proxy는 관절 자식이므로 같은 World 변환을 따른다. GUI는 Controller에 요청을 보내고 상태 복사본과 실제 접촉·파지 상태를 표시한다.

## Hardware backend 예정

```text
robotics/backends/hardware/hanwha/
robotics/backends/hardware/robotiq/
```

Hanwha backend는 HCR TCP/IP/Modbus TCP/Rodi adapter를 내부에 숨긴다.
Robotiq backend는 Modbus RTU RS-485 register packing/parsing을 내부에 숨긴다.
