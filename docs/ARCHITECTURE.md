# PoseLink Architecture

## 1. 문서 목적

PoseLink는 **원격 Vision Node에서 추정한 6DoF Pose를 UDP로 전송하고, Viewer에서 시간축을 복원하여 3D로 시각화하는 C++ 실시간 응용 소프트웨어**다.

이 문서는 구현 세부 코드보다 다음 경계를 고정한다.

- 두 실행 프로그램의 책임
- 모듈 간 의존 방향
- thread / queue / lifetime 정책
- Viewer의 ECS 사용 범위
- Vision Node를 graphics-free core로 유지하는 원칙
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
- CLI/config 기반 runtime 설정
- 선택적으로 OpenCV HighGUI debug preview 제공

`poselink_viewer`

- UDP packet 수신
- protocol 검증 / deserialize
- loss / reorder / duplicate 분석
- timestamp 기준 PoseBuffer 관리
- rendering timeline 기준 interpolation
- OpenCV camera frame → OpenGL frame 좌표 변환
- flecs Entity의 `Transform` 갱신
- OpenGL 3D visualization
- 선택적 Dear ImGui diagnostics panel

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
- `vision`이 GLFW / GLAD / flecs / Dear ImGui를 참조
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
Dear ImGui Diagnostics (optional)
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

    DrawDiagnosticsUI(); // optional

    window.SwapBuffers();
}
```

OpenGL / Dear ImGui 호출은 render/main thread에 귀속한다.

---

## 7. Vision Node architecture

Vision Node는 **graphics application이 아니다.**

핵심 구조:

```text
Camera Capture
      ↓
ArUco Detection
      ↓
solvePnP
      ↓
PoseSample
      ↓
Protocol Encode
      ↓
UDP Sender
```

기본 동작은 window 없이 실행된다.

```bash
poselink_vision_node \
  --camera 0 \
  --calibration camera.yml \
  --marker-size 0.05 \
  --host 192.168.0.10 \
  --port 5000 \
  --rate 30
```

### Debug preview

ArUco detection을 개발할 때만 선택적으로 OpenCV HighGUI를 사용할 수 있다.

```text
Capture / Detection
   ├─ PoseSample → UDP
   └─ annotated cv::Mat → OpenCV HighGUI (optional)
```

예:

```bash
poselink_vision_node ... --preview
```

이 preview는 제품 GUI가 아니라 debugging 기능이다.

`ArUcoPoseSource` 내부에 `cv::imshow()`를 박지 않고 application/debug-view 경계에서 표시한다.

---

## 8. Vision Node 동시성

MVP에서는 가능한 한 단순하게 시작한다.

### 1단계

```text
Single Thread
Capture
→ Detect
→ Pose
→ Send
```

먼저 correctness와 처리시간을 측정한다.

### 2단계

실제 측정에서 capture/vision 처리 때문에 송신 주기나 입력 처리 지연이 문제가 될 때 분리한다.

```text
Capture + Vision Worker
        ↓
 bounded Pose Queue
        ↓
    UDP Sender
```

필요 시 main thread는 process control / optional HighGUI preview를 담당할 수 있다.

### Queue policy

실시간 Pose는 오래된 데이터를 모두 보존하는 것보다 최신성이 중요하다.

```text
capacity 제한
overflow → DROP_OLDEST
```

queue depth, dropped sample count를 metric으로 기록한다.

Preview 영상은 긴 frame queue보다 latest-frame snapshot을 우선한다.

---

## 9. GUI / Debug UI 경계

### Vision Node

```text
정식 GUI 없음
```

설정은 CLI/configuration으로 전달한다.

```text
Camera Index
Calibration File
Marker Size
Target Host
Target Port
Send Rate
Preview On/Off
```

선택적 preview는 OpenCV HighGUI만 사용한다.

Vision Node가 링크하지 않아야 하는 것:

```text
OpenGL
GLFW
GLAD
flecs
Dear ImGui
Qt
MFC
```

### Viewer

Viewer는 본래 graphics application이므로 선택적으로 Dear ImGui diagnostics를 사용한다.

```text
Receive Rate
Loss / Reorder / Duplicate
Packet Age
PoseBuffer Occupancy
Interpolation Delay
Render FPS
```

즉:

```text
Vision Node
→ headless producer

Viewer
→ graphics consumer + optional diagnostics UI
```

으로 책임을 나눈다.

---

## 10. Ownership / lifetime

### Viewer

```text
ViewerApp
├─ owns Window
├─ owns Renderer
├─ owns Camera
├─ owns flecs::world
├─ owns network receiver
└─ owns optional ImGui context
```

OpenGL resource는 OpenGL context보다 먼저 파괴한다.

```text
ECS Renderable / Mesh / Texture
→ Renderer / ImGui GPU resources
→ Window / OpenGL Context
```

### Vision Node

```text
VisionNodeApp
├─ owns Camera
├─ owns PoseSource
├─ owns Sender
├─ owns optional workers
└─ owns optional DebugPreview helper
```

종료 순서:

```text
stop request
→ capture stop
→ queue close
→ worker join
→ socket close
→ optional HighGUI close
```

Vision Node에는 OpenGL context lifecycle이 없다.

RAII와 명시적 join을 사용하며 detached worker는 사용하지 않는다.

---

## 11. 시간 모델

서로 다른 시간을 구분한다.

### Frame `dt`

```text
현재 application frame 간격
```

사용:

- synthetic animation
- local simulation
- Viewer camera/input

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

## 12. 좌표계 경계

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

## 13. 향후 Robotics 확장 경계

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
