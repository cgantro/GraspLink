# PoseLink Project Background

## 1. 프로젝트를 시작한 이유

기존 C++ 프로젝트 경험에서 OpenGL rendering, 실시간 통신, streaming, multithreading을 각각 다뤘지만, **Vision에서 생성된 공간 상태가 네트워크를 지나 다른 프로그램의 3D scene으로 재현되는 전체 경로**를 작은 코드베이스에서 처음부터 설계하고 검증할 필요가 있었다.

단순 OpenGL 예제는 다음만 보여준다.

```text
Vertex
→ Shader
→ Draw
```

단순 OpenCV 예제는 다음만 보여준다.

```text
Camera
→ Detection
→ Pose
```

단순 UDP 예제는 다음만 보여준다.

```text
sendto
→ recvfrom
```

PoseLink는 이 세 영역을 하나의 문제로 연결한다.

```text
Physical / Synthetic Motion
        ↓
Vision Pose
        ↓
Serialization
        ↓
Unreliable Network
        ↓
Time Reconstruction
        ↓
3D Visualization
```

---

## 2. 해결하려는 핵심 문제

원격 3D 상태를 화면에 그릴 때 단순히 **가장 최근 도착한 Pose를 즉시 적용**하면 network jitter가 그대로 시각적 흔들림으로 나타난다.

예:

```text
송신 시각
0   33   66   99   132 ms

도착 시각
8   72   79   160  169 ms
```

송신 주기는 일정해도 수신 주기는 불규칙하다.

즉 Viewer가 받는 문제는 단순한 위치 데이터가 아니라:

```text
값(value)
+
생성 시각(timestamp)
+
순서(sequence)
```

가 있는 **시간축 상태 스트림**이다.

PoseLink의 중심 질문은 다음이다.

> 약간의 의도된 buffering latency를 허용하면, 원격 Pose 시각화의 jitter를 얼마나 줄일 수 있는가?

---

## 3. 왜 Synthetic Pose가 필요한가

실제 Camera + ArUco부터 시작하면 오차 원인이 섞인다.

```text
Camera Noise
Marker Detection Error
solvePnP Error
Network Jitter
Packet Loss
Interpolation Error
Rendering Error
```

결과가 흔들려도 어디서 발생했는지 알기 어렵다.

그래서 첫 source는 deterministic한 `SyntheticPoseSource`로 둔다.

예:

```text
x(t) = sin(t)
rotation(t) = known quaternion trajectory
```

예상 Pose를 알고 있으므로:

```text
Ground Truth
vs
Received Pose
vs
Rendered Pose
```

를 비교할 수 있다.

이 설계는 Vision 정확도와 network/interpolation 성능을 분리하기 위한 것이다.

---

## 4. 왜 두 프로그램으로 분리하는가

한 process에서:

```text
Camera
→ Pose
→ Cube
```

만 구현하면 네트워크 시스템의 문제를 검증할 수 없다.

따라서 의도적으로:

```text
VisionNode Process
        ↓ UDP
Viewer Process
```

로 나눈다.

이렇게 해야 실제로 다음 현상을 관찰할 수 있다.

- packet loss
- packet reorder
- duplicate
- receive interval jitter
- socket/thread 경계
- producer/consumer 속도 차이
- shutdown/lifetime 문제

---

## 5. 왜 UDP인가

Pose는 명령 로그나 파일 전송과 달리 **최신 상태의 가치가 가장 높다.**

예:

```text
Pose #100
Pose #101
Pose #102
```

#100을 늦게 복구하는 동안 이미 #102가 존재한다면 #100의 실시간 가치는 낮다.

따라서 PoseLink에서는:

```text
reliability보다 freshness 우선
```

인 UDP를 선택하고, reliability 부족을 숨기지 않고 다음을 직접 측정한다.

- sequence gap
- out-of-order
- packet age
- drop
- jitter

UDP가 모든 실시간 시스템의 정답이라는 의미는 아니다. 이 프로젝트의 state-update 특성과 실험 목적에 맞는 선택이다.

---

## 6. 왜 Jitter Buffer와 Interpolation인가

Viewer에 Pose A와 Pose B가 존재한다고 하자.

```text
A(t0) -------- B(t1)
```

Viewer는 실제 현재 시각보다 조금 과거인:

```text
renderTime = now - bufferDelay
```

를 선택한다.

그 시점이 A와 B 사이에 있다면:

```text
position
→ LERP

rotation
→ quaternion SLERP
```

로 Pose를 복원한다.

trade-off:

```text
bufferDelay 증가
→ interpolation 가능성 / 안정성 증가
→ latency 증가

bufferDelay 감소
→ freshness 증가
→ jitter / extrapolation 위험 증가
```

이 trade-off를 수치로 설명하는 것이 프로젝트의 핵심 산출물 중 하나다.

---

## 7. 왜 OpenGL Viewer인가

Viewer는 결과 확인용 화면만이 아니라 다음을 직접 검증하는 도구다.

- 6DoF position/orientation 변환
- Model/View/Projection
- OpenCV ↔ OpenGL coordinate conversion
- quaternion rotation
- interpolation 결과
- robot hierarchy 확장 가능성

기존 game engine을 사용하면 빠르게 화면은 만들 수 있지만 graphics data path가 가려진다.

PoseLink에서는 OpenGL을 직접 사용하여:

```text
Pose
→ Transform
→ Model Matrix
→ Shader
→ Rasterization
```

경계를 코드로 확인할 수 있게 한다.

---

## 8. 왜 flecs를 사용하는가

첫 Cube 하나만 그릴 때 ECS는 필요하지 않다.

하지만 Viewer가 확장되면 scene에는 다음 객체가 생긴다.

```text
Tracked Target
Reference Object
Camera
Robot Base
Robot Links
Debug Axis
Marker
```

상속 계층을 늘리는 대신:

```text
Entity
+ Transform
+ Renderable
+ optional role components
```

형태로 조합하고, 동일 component 조합을 System이 처리하도록 한다.

다만 ECS를 모든 문제에 강제하지 않는다.

```text
UDP parser
OpenCV pipeline
Pose protocol
```

은 일반 C++ module로 유지한다.

---

## 9. 왜 Vision Node에 GUI가 필요한가

카메라 프로그램은 headless sender만으로도 기능할 수 있지만 개발과 검증에는 다음 상태를 동시에 봐야 한다.

```text
원본 Camera Frame
Detected Marker
Reprojection / Axis Overlay
Current 6DoF Pose
Detection FPS
Send Rate
Target Endpoint
Calibration Status
Packet Counters
```

따라서 GUI는 제품 기능보다 **관측 가능성(observability)**을 위한 engineering interface다.

이 성격 때문에 Qt/MFC보다 Dear ImGui를 기본 선택한다.

GUI를 core에 섞지 않아:

```text
GUI mode
Headless benchmark mode
```

를 모두 유지한다.

---

## 10. 프로젝트가 보여주려는 역량

PoseLink의 목적은 library 사용 개수를 늘리는 것이 아니다.

코드와 실험으로 다음을 설명할 수 있어야 한다.

### C++ 응용 SW

- RAII와 ownership
- component/module boundary
- object lifetime
- CMake target dependency
- error handling / shutdown

### Graphics

- VAO/VBO/EBO
- Shader
- Texture
- MVP
- Depth
- quaternion transform

### Real-time data flow

- `dt`와 timestamp 구분
- bounded queue
- freshness vs completeness
- latency / jitter / throughput 구분
- producer-consumer

### Network

- UDP
- binary serialization
- endian
- sequence number
- packet validation
- loss/reorder metrics

### Computer Vision

- calibration
- ArUco
- solvePnP
- coordinate frame
- reprojection error

### Architecture

- core와 GUI 분리
- domain data와 rendering data 분리
- Vision / Transport / Streaming / Viewer 책임 분리
- testable synthetic source

---

## 11. 프로젝트 범위의 당위성

프로젝트가 너무 넓어지는 것을 막기 위해 MVP에서는 다음을 하지 않는다.

- 복수 카메라 sensor fusion
- markerless tracking
- neural network detector
- distributed clock synchronization
- physics engine
- full scene editor
- production authentication/encryption
- full robot dynamics

핵심은 다음 한 경로를 완성하는 것이다.

```text
Known / Real Pose
→ UDP
→ Timestamp Buffer
→ Interpolation
→ 3D Reconstruction
```

그 이후에만 FK/IK 로봇 시각화를 확장한다.

---

## 12. 완료 후 설명하고 싶은 한 문장

> PoseLink는 Vision에서 생성한 6DoF 상태를 UDP로 전송하고 timestamp 기반 buffering과 보간으로 원격 3D 시각화를 복원하며, 네트워크 jitter와 추가 latency의 trade-off를 synthetic ground truth로 측정하는 C++ 실시간 응용 프로젝트다.
