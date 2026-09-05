# PoseLink Architecture

## 1. 문서 목적

PoseLink는 **원격 Vision Node에서 추정한 6DoF Pose를 UDP로 전송하고, Viewer에서 시간축을 복원하여 3D로 시각화하는 C++ 실시간 응용 소프트웨어**다.

이 문서는 구현 세부 코드보다 다음 경계를 고정한다.

- 두 실행 프로그램의 책임
- 모듈 간 의존 방향
- thread / queue / lifetime 정책
- Viewer의 ECS 사용 범위
- GUI가 실시간 데이터 경로에 끼어들지 않는 방식
- 현재 구현과 목표 구조의 차이

> 문서의 수치 목표는 `ACCEPTANCE_CRITERIA.md`에서 별도로 정의한다.

---

## 2. 시스템 구성

최종 시스템은 두 개의 주 실행 프로그램과 하나의 선택적 실험 도구로 구성한다.

```text
┌──────────────────────────────┐
│ poselink_vision_node         │
│                              │
│ Camera / Synthetic Source    │
│        ↓                     │
│ 6DoF Pose Estimation         │
│        ↓                     │
│ Protocol Encode              │
│        ↓                     │
│ UDP Sender                   │
└──────────────┬───────────────┘
               │ UDP
               ▼
┌──────────────────────────────┐
│ poselink_viewer              │
│                              │
│ UDP Receiver                 │
│        ↓                     │
│ Validation / Sequence        │
│        ↓                     │
│ Timestamped PoseBuffer       │
│        ↓                     │
│ LERP / SLERP                 │
│        ↓                     │
│ ECS Transform                │
│        ↓                     │
│ OpenGL Renderer              │
└──────────────────────────────┘

Optional:
Vision Node → poselink_net_proxy → Viewer
```

### 프로그램별 역할

`poselink_vision_node`

- 카메라 또는 Synthetic source에서 Pose 생성
- Camera calibration 결과 로드
- ArUco 검출 및 `solvePnP`
- Pose에 sequence / monotonic timestamp 부여
- binary protocol serialization
- UDP publish
- 카메라 preview, detection overlay, 송신 상태를 GUI로 표시
- GUI 없이 실험할 수 있는 headless mode 지원

`poselink_viewer`

- UDP packet 수신
- protocol 검증 / deserialize
- loss / reorder / duplicate 분석
- timestamp 기준 PoseBuffer 관리
- rendering timeline 기준 interpolation
- OpenCV camera frame → OpenGL frame 좌표 변환
- flecs Entity의 `Transform` 갱신
- OpenGL 3D visualization
- 선택적 ImGui diagnostics panel

`poselink_net_proxy`

- delay / jitter / loss / reorder 재현
- seed 기반 재현 가능한 network impairment
- Linux에서는 `tc netem`으로 대체 가능

---

## 3. 현재 구현 상태와 목표 상태

### 현재 구현

현재 repository에서 실제 build target으로 구성된 것은 Viewer 쪽이 중심이다.

```text
ViewerApp
├─ Window
├─ Camera
├─ Renderer
└─ flecs::world
   ├─ Entity
   │  ├─ Transform
   │  └─ Renderable
   └─ RenderSystem
```

현재 graphics pipeline은 다음을 검증하는 단계다.

```text
Mesh
→ VAO / VBO / EBO
→ Shader / Texture
→ Model / View / Projection
→ Depth Test
→ OpenGL Draw
```

flecs는 Viewer의 scene-state와 반복 처리 규칙에 사용한다.

### 목표 상태

```text
SyntheticPoseSource / ArUcoPoseSource
          ↓
      PoseSample
          ↓
   PacketEncoder
          ↓
     UDP Socket
          ↓
     PoseReceiver
          ↓
Validation + Sequence Metrics
          ↓
     PoseBuffer
          ↓
PoseInterpolator
          ↓
CoordinateConverter
          ↓
ECS Transform
          ↓
RenderSystem
          ↓
Renderer
```

---

## 4. 모듈 구조와 의존 방향

```text
modules/
├─ common
│  └─ Pose / PoseSample / time-related value types
│
├─ vision
│  ├─ IPoseSource
│  ├─ SyntheticPoseSource
│  └─ ArUcoPoseSource
│
├─ transport
│  ├─ Protocol
│  └─ UdpSocket
│
├─ streaming
│  ├─ PoseReceiver
│  ├─ PoseBuffer
│  ├─ SequenceMetrics
│  └─ PoseInterpolator
│
└─ viewer
   ├─ components
   ├─ systems
   └─ graphics
```

의존 방향은 다음 원칙을 따른다.

```text
              common
          ↗      ↑      ↖
     vision   transport   viewer
                  ↑
              streaming
```

정확히는 application target이 필요한 module을 조립한다.

### 금지하는 의존

- `common`이 OpenGL / OpenCV / flecs를 include
- `transport`가 Viewer나 OpenGL을 참조
- `vision`이 Viewer의 `Transform`을 직접 수정
- `Renderer`가 UDP packet을 해석
- `PoseBuffer`가 OpenGL draw를 호출

`Pose`와 `Transform`은 의도적으로 분리한다.

```text
Pose
= vision / network domain data

Transform
= rendering / scene data

Pose → Transform
= application boundary에서 변환
```

---

## 5. Viewer의 ECS 구조

flecs는 **Viewer에 한정해 사용**한다. Vision Node에는 현재 ECS가 필요하지 않다.

### Entity

Entity는 identity다.

```text
TrackedObject
RobotLink
DebugAxis
CameraObject
```

### Component

현재 핵심:

```text
Transform
Renderable
```

향후 필요 시:

```text
RemoteTracked
RobotLink
PoseBufferRef
TrackingState
```

같은 tag/component를 추가할 수 있다.

### System 사용 기준

```text
특정 Entity 하나에 외부 입력을 적용
→ 일반 application update로 충분

동일한 Component 조합의 Entity 전체에
반복 규칙을 적용
→ flecs System
```

예:

```text
Transform + Renderable
→ RenderSystem

RemoteTracked + InterpolationState
→ 향후 PoseInterpolationSystem 후보
```

모든 `Update()`를 System으로 만들지 않는다.

---

## 6. Viewer frame loop

목표 frame loop:

```text
PollEvents
    ↓
Drain Received Pose Queue
    ↓
Update PoseBuffer / Metrics
    ↓
Select render timestamp
    ↓
Interpolate Pose
    ↓
Apply Pose → Transform
    ↓
Renderer::BeginFrame
    ↓
world.progress(dt)
    ↓
RenderSystem
    ↓
ImGui Diagnostics
    ↓
SwapBuffers
```

예상 형태:

```cpp
while (!window.ShouldClose())
{
    float dt = clock.Tick();

    window.PollEvents();

    DrainNetworkPackets();
    UpdateTrackedPose(dt);

    renderer.BeginFrame();

    world.progress(dt);

    DrawDiagnosticsUI();

    window.SwapBuffers();
}
```

OpenGL 호출은 render/main thread에 귀속한다.

---

## 7. Vision Node architecture

Vision Node는 ECS보다 **pipeline / worker ownership**이 중요하다.

```text
Camera Capture
      ↓
ArUco Detection
      ↓
solvePnP
      ↓
PoseSample
      ↓
Bounded Pose Queue
      ↓
UDP Sender
```

GUI가 활성화되면:

```text
Capture/Vision Worker
   ├─ latest preview frame
   └─ latest pose / diagnostics
              ↓
        Main/UI Thread
        Dear ImGui
```

### 권장 thread 구성

MVP에서는 먼저 단일 thread로 correctness를 검증하고, camera processing 때문에 UI나 송신 주기가 불안정해질 때 아래로 확장한다.

```text
Thread 1: UI / OpenGL
Thread 2: Capture + Vision
Thread 3: UDP Sender
```

Thread 2 → Thread 3은 bounded queue를 사용한다.

### Queue policy

실시간 Pose는 오래된 데이터를 모두 보존하는 것보다 최신성이 중요하다.

```text
capacity 제한
overflow → DROP_OLDEST
```

queue depth, dropped sample count를 metric으로 기록한다.

Preview 영상은 frame queue를 길게 두지 않고 **latest-frame snapshot / double buffer**를 우선한다.

---

## 8. GUI architecture

GUI toolkit은 `TECH_STACK.md`의 비교 결과에 따라 **Dear ImGui를 기본 선택**한다.

중요한 원칙:

```text
GUI
≠ core pipeline
```

즉:

```text
ArUcoPoseSource
UdpPublisher
PoseBuffer
Protocol
```

은 ImGui를 몰라야 한다.

GUI는 아래 상태를 읽고 command/configuration만 전달한다.

```text
Camera Preview
Marker Overlay
Current Pose
Detection State
Source FPS
Send Rate
Target Host / Port
Calibration File
Marker Size
Packet Counters
Queue Depth
```

이 구조 덕분에 동일 core를 다음 두 모드에서 실행할 수 있다.

```text
Interactive GUI mode
Headless benchmark mode
```

---

## 9. Ownership / lifetime

### Viewer

```text
ViewerApp
├─ owns Window
├─ owns Renderer
├─ owns Camera
├─ owns flecs::world
├─ owns network receiver
└─ owns UI context
```

OpenGL resource는 OpenGL context보다 먼저 파괴한다.

```text
ECS Renderable / Mesh / Texture
→ Renderer/UI GPU resources
→ Window / OpenGL Context
```

### Vision Node

```text
VisionNodeApp
├─ owns Camera
├─ owns PoseSource
├─ owns Sender
├─ owns worker threads
└─ owns UI context
```

종료 순서:

```text
stop request
→ capture stop
→ queue close
→ worker join
→ socket close
→ GUI/OpenGL resource release
→ window/context release
```

RAII와 `std::jthread` 또는 명시적 join을 사용하며 detached worker는 사용하지 않는다.

---

## 10. 시간 모델

서로 다른 시간을 구분한다.

### Frame `dt`

```text
현재 application frame 간격
```

사용:

- synthetic animation
- camera controls
- UI update
- local simulation

### Pose timestamp

```text
Pose sample 생성 시각
```

사용:

- packet age
- jitter buffer
- interpolation timeline
- E2E latency 분석

duration 측정에는 monotonic clock (`std::chrono::steady_clock`)을 사용한다.

```text
frame dt
≠
pose timestamp
≠
wall clock
```

---

## 11. 좌표계 경계

Vision:

```text
OpenCV Camera Frame
+X right
+Y down
+Z forward
```

Viewer:

```text
OpenGL convention
+X right
+Y up
-Z forward
```

따라서 다음 변환을 명시적 함수/모듈로 둔다.

```text
OpenCV Pose
→ CoordinateConverter
→ Viewer Pose
→ Transform
```

좌표계 변환을 `Renderer`, `solvePnP`, `Transform` 내부에 흩뿌리지 않는다.

---

## 12. 향후 Robotics 확장 경계

기본 PoseLink가 안정화된 뒤 선택적으로 확장한다.

```text
Remote Target Pose
        ↓
        IK
        ↓
   Joint Angles
        ↓
        FK
        ↓
 Link Transforms
        ↓
     Viewer ECS
```

이 단계에서 flecs Relationship / `ChildOf`를 이용해:

```text
Base
└─ Link1
   └─ Link2
      └─ EndEffector
```

hierarchy를 표현할 수 있다.

Robotics는 핵심 MVP 완료 전에는 구현 범위를 늘리지 않는다.
