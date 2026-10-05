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
   - 현재 `accelerationScale`은 저장·검증만 하며 실제 가속도 제한이나 ramp는 적용하지 않는다.
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

5. **Physics Base Integration — implemented**
   - `modules/physics`는 Jolt 초기화와 Body 생성·삭제·step만 담당한다.
   - Viewer ECS integration은 Entity 설정 컴포넌트를 읽어 Body를 생성하고 수명을 연결한다.
   - Render FPS와 독립적인 fixed step에서 Kinematic 입력과 Dynamic 결과 반영을 처리한다.
   - SceneRoot 아래 Entity는 World pose를 부모 역행렬로 Local Transform에 되돌린다.

최소 구현 범위:

```text
PhysicsWorld
Gravity
Static Floor
Dynamic Box
Collision
Fixed Physics Step
Flecs Entity physics config -> Jolt Body
Kinematic Entity Transform -> Physics
Dynamic Physics World pose -> Entity Local Transform
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

Jolt 전용 타입은 `modules/physics` 내부에 격리한다. Simulation integration에는 GLM과
프로젝트의 `PhysicsBodyHandle`만 보이며 Jolt `BodyID`는 노출하지 않는다.

6. **Robot Model / FK — implemented**
   - HCR-12A Joint State로부터 base 기준 link pose와 ToolFrame pose 계산
   - Joint hierarchy 기반 kinematic chain 정의
   - ToolFrame 포함
   - parent-relative transform 기준으로 구성
   - bind pivot 사이의 차이를 누적 회전에 적용해 base-frame pose를 계산한다.
   - FK ToolFrame output은 controller TCP feedback과 별도다. 현재 controller `tcpPoseValid`는 false다.

```text
q1 ... q6
    ↓
Forward Kinematics
    ↓
ToolFrame Position
ToolFrame Orientation
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

8. **Gripper Runtime Controller — 자유공간 개폐 구현**
   - `IGripperController` contract와 2F-85 specification은 준비됨
   - `SimGripperController`가 raw 요청을 연속 `closureFraction` 상태로 갱신
   - `GripperKinematics`의 master/mimic Local 회전을 `GripperTransformAdapter`가 GLB bind에 적용
   - raw 위치의 선형 fraction 매핑과 기본 master 속도 0.1..1.0 rad/s는 제조사 사양이 아닌 시뮬레이션 가정
   - force·contact 판정·접촉 시 정지·접촉 이후 under-actuated 동작·파지는 아직 구현하지 않음

```text
positionRequest
0 ... 255
     ↓
GripperState.closureFraction [0,1]
     ↓
master linkage angle q
     ↓
mimic relation
     ↓
Gripper Joint State
     ↓
GLB 관절 Local 회전 → World 변환 → 기존 7개 Kinematic proxy
```

4 ms마다 Controller 갱신 → 팔·그리퍼 자세 적용 → World 변환 갱신 → Jolt step → World 변환 재갱신 순서다. GUI는 인터페이스에 요청을 보내며 Entity를 직접 변경하지 않는다. 현재 계약과 검증은 [그리퍼 런타임 설계](GRIPPER_RUNTIME_DESIGN.md)에 정리한다.

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
   - GLB에서 만든 Convex Hull을 별도 Kinematic link proxy Entity에 붙이는 기반은 구현됨
   - proxy는 FK의 base-frame link pose를 따르며 관절 제약이나 토크를 푸는 articulated dynamics는 아님
   - Kinematic gripper collision proxy는 구현됨
   - 접촉 기반 그리퍼 정지·grasp 판단, 부착 상태별 collision filter 및 실제 Robot/Gripper 동역학은 다음 단계

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
├─ Transform / Render Components
├─ RigidBody
└─ Colliders
       │
       ▼
PhysicsSystemModule
  - config observer / private runtime binding
  - Local ↔ World 변환과 fixed-step 동기화
       │
       ▼
PhysicsWorld
  - PhysicsBodyHandle API
  - Jolt 내부 구현
```

역할:

```text
Flecs
= Entity / ECS / Application State

PhysicsWorld
= Jolt lifetime / rigid body / collision / fixed step

PhysicsSystemModule
= Flecs component 해석 / Body 연결 / Transform 동기화

Renderer
= 결과 시각화
```

두 계층은 기능 중복이 아니다. `PhysicsWorld`는 Flecs를 몰라야 하고, ViewerApp은 Body handle이나
좌표 변환 세부사항을 직접 관리하지 않아야 한다. `PhysicsBodyBinding`은 Entity에 붙는 private
runtime component이며 Entity나 물리 설정이 제거될 때 observer가 Jolt Body를 삭제한다.

Collider 반 크기는 meter 단위이며 Entity scale을 자동 곱하지 않는다. Robot Link와 Dynamic Entity는 unit
scale을 쓰고, Floor는 렌더 GLB scale과 독립된 collider 크기를 명시한다. Dynamic Body 조상 아래의 Dynamic
Body는 binding 생성 시 거부된다. Dynamic Entity의 parent가 scale/shear를 가지는 경우도 지원 범위가 아니다.

## 구현 순서

```text
1. robotics/core + model/backend 분리
                    ✅

2. Joint angle / velocity limit
                    ✅ 기본 구현

3. Fixed Control Loop
                    ✅ 250 Hz fixed step

4. Jolt Physics 최소 기반
   - PhysicsWorld
   - Gravity
   - Floor
   - Dynamic Box
   - ECS 설정 컴포넌트와 lifetime 연동
   - Robot J1~J6 Kinematic collision proxy
                    ✅ 기본 기반

5. Robot Model / FK
   - Kinematic Chain
   - ToolFrame
   - FK 검증
                    ✅ 구현

6. IK / MoveLinear

7. Gripper Runtime Controller
   - 2F-85 request
   - master q
   - free-space mimic
                    ✅ 자유공간 backend / 기구학 / GLB·proxy 연결
   - Force / contact stop / grasp
                    ← 후속 물리 작업

8. Robot / Gripper Physics
   - Asset 메시 기반 Gripper Kinematic collider proxy
                    ✅ 기본 기반
   - Constraint / articulated dynamics
                    ← 다음 물리 확장

9. Physics / Grasp
   - Contact와 under-actuated adaptation
   - Attach / Detach / Release
                    ← 미구현

10. Safety
    - Watchdog
    - E-Stop State
    - Zero Offset
```
