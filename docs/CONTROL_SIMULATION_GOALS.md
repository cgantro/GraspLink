# Robotics Simulation Goals

## 목표

렌더링 구현이나 특정 하드웨어와 결합하지 않은 동일한 robotics core를
Simulation과 Real Hardware에서 재사용한다.

```text
Motion Command
      ↓
IRobotController
      ↓
Backend
├─ Simulation
└─ Hardware
      ↓
RobotState
      ↓
Kinematics / Physics / Viewer
```

## 우선순위

1. **Backend 분리**
   - `IRobotController` / `IGripperController`를 공통 contract로 사용
   - `SimRobotController`와 향후 Hardware backend를 분리
   - Viewer는 구체적인 Simulation/Hardware 구현에 직접 의존하지 않는다.

2. **Joint Angle Limit**
   - model별 `minPositionRadians`, `maxPositionRadians`
   - 적용 전 target 검증
   - 관절 제한을 벗어난 명령은 상태에 적용하지 않는다.

3. **Velocity / Acceleration Limit**
   - model별 최대 속도 사용
   - 목표 각도를 즉시 적용하지 않고 시간에 따라 상태를 변화
   - `velocityScale`은 model 최대 속도에 대한 비율로 사용
   - 제조사 최대 가속도 값이 확인되기 전에는 임의 상수를 실제 사양으로 취급하지 않음
   - Simulation용 가속도 값이 필요하면 제조사 사양과 분리해서 관리

4. **Fixed Control Loop**
   - Rendering FPS와 제어 주기를 분리
   - accumulator 기반 고정 `dt`로 Controller를 Update
   - Rendering frame의 `dt`를 Controller에 직접 전달하지 않는다.
   - `FixedControlLoop`는 특정 Controller 구현에 직접 의존하지 않는다.
   - callback 형태로 고정 timestep을 전달한다.
   - hard real-time scheduler가 아니며 thread/sleep을 직접 관리하지 않는다.

```text
Render Frame
    │
    │ frameDelta
    ▼
FixedControlLoop
    │
    ├─ accumulator += frameDelta
    │
    └─ while accumulator >= fixedDt
           │
           ├─ callback(fixedDt)
           └─ accumulator -= fixedDt
```

예:

```text
fixedDt = 0.004 s
frameDt = 0.010 s

tick(0.004)
tick(0.004)

remaining accumulator = 0.002 s
```

Rendering FPS가 변하더라도 Controller는 항상 동일한 `fixedDt`를 받는다.

```text
Rendering
60 FPS
144 FPS
200 FPS
...

Control
fixed dt
fixed dt
fixed dt
...
```

5. **Physics Base Integration**
   - Fixed Control Loop 다음에 Jolt Physics 최소 기반을 먼저 통합
   - 이 단계에서는 Robot/Gripper 물리를 완성하지 않는다.
   - Jolt가 프로젝트 구조 안에서 정상 동작하는 것부터 확인한다.

최소 구현 범위:

```text
PhysicsWorld
Gravity
Static Floor
Dynamic Box
Collision
Fixed Physics Step
Physics Transform -> Flecs Transform
```

구조:

```text
Jolt Physics
    ↓
Position / Rotation
    ↓
Flecs Transform
    ↓
Renderer
```

Jolt 전용 API는 `modules/physics` 내부에 격리한다.

Viewer나 Robotics Core에서 Jolt 타입에 직접 의존하지 않도록 한다.

6. **Robot Model / FK**
   - HCR-12A Joint State로부터 TCP Pose 계산
   - Joint hierarchy 기반 kinematic chain 정의
   - ToolFrame 포함
   - parent-relative transform 기준으로 구성
   - 현재 GLB의 world-space pivot 값만으로 FK를 구성하지 않는다.

```text
q1 ... q6
    ↓
Forward Kinematics
    ↓
TCP Position
TCP Orientation
```

검증 대상:

```text
Zero Pose
J1 only
J2 only
J3 only
Multiple Joint Pose
Tool Frame
```

FK 결과와 GLB ToolFrame 결과를 비교한다.

7. **IK / Cartesian Motion**
   - 목표 TCP Pose에서 Joint State 계산
   - 초기 방식은 Damped Least Squares 기반 iterative IK
   - 이후 `MoveLinear()`와 연결

```text
Target TCP Pose
      ↓
IK
      ↓
Joint Target
      ↓
Joint Controller
```

검토 항목:

- Position Error
- Orientation Error
- Jacobian
- Joint Limit
- Singularity
- Convergence
- Maximum Iteration
- Unreachable Target

8. **Gripper Runtime Controller**
   - `IGripperController`의 Simulation backend 구현
   - Robotiq 2F-85 request를 runtime state로 변환
   - Free-space에서는 mimic relation 사용
   - contact 이후 under-actuated 동작은 여기서 처리하지 않는다.

```text
positionRequest
0 ... 255
     ↓
master linkage angle q
     ↓
mimic relation
     ↓
Gripper Joint State
```

Free-space 관계:

```text
LeftOuter  = +q
RightOuter = -q

LeftInner  = +q
RightInner = -q

LeftTip    = -q
RightTip   = +q
```

9. **Robot / Gripper Physics**
   - 기본 Jolt integration 위에 실제 Robot/Gripper 물리 추가

```text
Robot Link
    ↓
Collision Shape
    ↓
Rigid / Kinematic Body

Gripper Finger
    ↓
Collision
    ↓
Constraint
    ↓
Contact
```

구현 대상:

- Robot Link Collider
- Gripper Link Collider
- Joint Constraint
- Self Collision 정책
- Environment Collision
- Object Contact

10. **Physics / Grasp**
   - 2F-85의 free-space mimic 관계는 물체 접촉 전까지 사용
   - 물체 접촉 이후에는 단순 `jointAngle = masterAngle * multiplier` 관계만 강제하지 않는다.
   - 접촉 이후 finger adaptation은 Physics / Constraint 계층에서 처리한다.

```text
Finger Closing
      ↓
Object Contact
      ↓
Constraint Response
      ↓
Finger Adaptation
      ↓
Grasp State
```

구현 대상:

- ContactWhileClosing
- ContactWhileOpening
- Object Detection
- Grasp
- Attach
- Detach
- Release

11. **Safety 확장**
   - software stop 상태
   - watchdog
   - E-Stop state model
   - zero offset
   - invalid state 차단
   - NaN / Inf command 차단
   - command timeout

실제 E-Stop 회로를 software `Stop()`이 대체하지 않는다.

## Physics Integration

```text
Flecs Entity
├─ Transform
├─ RigidBody
├─ Collider
└─ PhysicsBodyHandle
        │
        ▼
   Jolt BodyID
        │
        ▼
 Jolt PhysicsSystem
```

역할:

```text
Flecs
= Entity / ECS / Application State

Jolt
= Rigid Body / Collision / Constraint / Contact

Renderer
= 결과 시각화
```

Jolt 전용 API는 `modules/physics` 내부에 격리한다.

## 구현 순서

```text
1. robotics/core + model/backend 분리
                    ✅

2. Joint angle / velocity limit
                    ✅ 기본 구현

3. Fixed Control Loop
                    ← 현재 작업

4. Jolt Physics 최소 기반
   - PhysicsWorld
   - Gravity
   - Floor
   - Dynamic Box
   - Flecs Transform Sync

5. Robot Model / FK
   - Kinematic Chain
   - ToolFrame
   - FK 검증

6. IK / MoveLinear

7. Gripper Runtime Controller
   - 2F-85 request
   - master q
   - free-space mimic

8. Robot / Gripper Physics
   - Collider
   - Constraint
   - Contact

9. Grasp
   - Under-actuated contact
   - Attach
   - Detach
   - Pick & Place

10. Safety
    - Watchdog
    - E-Stop State
    - Zero Offset
```