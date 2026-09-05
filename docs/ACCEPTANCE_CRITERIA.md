# PoseLink Acceptance Criteria

## 1. 목적

이 문서는 "기능이 돌아간다"가 아니라 **어느 수준이면 프로젝트를 완료했다고 볼 것인지**를 고정한다.

수치는 아직 측정 결과가 아니라 **목표/합격 기준**이다. 실제 hardware와 camera 조건을 기록한 뒤 실험 결과로 검증한다.

기준은 다음 세 단계로 나눈다.

```text
MVP
→ 전체 경로가 정확하게 동작

Target
→ 실시간성과 안정성이 만족스러움

Stretch
→ 네트워크/비전 품질까지 정량적으로 설명
```

---

# 2. 기준 실험 환경

최종 결과에는 반드시 실제 환경을 기록한다.

예:

```text
OS
CPU
GPU
Camera
Resolution
Camera FPS
Marker Size
Marker Distance
Build Type
Compiler
OpenCV Version
Viewer Resolution
Pose Send Rate
```

성능 수치는 Debug build가 아니라 Release build에서 측정한다.

---

# 3. 기능 요구사항

## FR-01 Viewer 3D Rendering

- OpenGL 3.3 Core context 생성
- 최소 하나 이상의 Entity 렌더링
- position / quaternion / scale이 Model Matrix에 반영
- Camera View / Perspective Projection 적용
- depth test 정상 동작

### 합격

```text
Synthetic Pose를 직접 적용했을 때
예상된 위치와 회전으로 객체가 움직인다.
```

---

## FR-02 Synthetic Pose Source

- deterministic trajectory 생성
- frame `dt` 기반 시간 진행
- position + quaternion 출력
- 같은 time sequence에 동일 결과

예:

```text
x(t) = sin(t)
Y-axis rotation = t
```

### 합격

고정된 `dt` sequence를 사용한 unit test에서 동일 Pose sequence가 재현된다.

---

## FR-03 Binary Protocol

Packet에 최소 다음 정보를 포함한다.

- magic/version
- sequence number
- monotonic timestamp
- position XYZ
- quaternion WXYZ

요구:

- 명시적 serialization
- 명시적 endian 처리
- packet size/version 검증
- invalid packet reject

### 합격

```text
Pose → Encode → Decode → Pose
```

round-trip test를 통과하고 malformed packet이 crash를 유발하지 않는다.

---

## FR-04 UDP Vision Node → Viewer

- 별도 process 실행
- host / port 설정 가능
- configurable send rate
- Viewer bind port 설정 가능

### 합격

localhost에서 30 Hz Synthetic Pose를 10분간 송수신할 때:

```text
decode error = 0
application crash = 0
unexpected duplicate/reorder = 0
```

운영체제/환경에 의한 실제 UDP loss가 발생하면 loss를 숨기지 않고 metric에 기록한다.

---

## FR-05 Sequence Analysis

Viewer가 다음을 구분한다.

- 정상 packet
- sequence gap
- duplicate
- out-of-order

### 합격

테스트에서 의도적으로:

```text
1, 2, 4, 3, 4
```

를 넣었을 때 각각 loss/reorder/duplicate metric이 기대값과 일치한다.

---

## FR-06 Timestamped PoseBuffer

- timestamp 정렬
- bounded capacity
- 오래된 sample 제거
- render timestamp를 감싸는 두 Pose 조회

### 합격

unit test에서:

- ordered insert
- out-of-order insert
- oldest eviction
- before/inside/after range lookup

을 모두 검증한다.

---

## FR-07 Interpolation

Position:

```text
LERP
```

Rotation:

```text
Quaternion SLERP
```

요구:

- quaternion normalize
- shortest-path 고려
- alpha clamp 또는 명시적 extrapolation policy

### 합격

Known trajectory unit test에서:

```text
alpha = 0   → A
alpha = 1   → B
alpha = 0.5 → 중간 Pose
```

가 허용 오차 내에서 일치한다.

---

## FR-08 Vision Camera Pipeline

- camera open/close
- calibration file load
- ArUco marker detect
- marker ID 표시
- solvePnP
- rvec/tvec → Position/Quaternion 변환
- detection 실패 상태 표현

### 합격

실제 marker를 움직였을 때 position/orientation 방향이 육안상 일치하고, 좌표계 검증 테스트를 통과한다.

---

## FR-09 Vision Node GUI

Dear ImGui 기반으로 최소 다음을 제공한다.

### Preview

- camera image
- detected marker corners
- marker ID
- axis 또는 pose overlay

### Controls

- camera index
- calibration file
- marker size
- target host
- target port
- send rate
- start/stop

### Diagnostics

- capture FPS
- detection FPS
- send rate
- sequence
- current position
- current quaternion
- queue depth/drop count
- detection state

### 합격

GUI 조작으로 process 재시작 없이 sender endpoint와 주요 runtime option을 확인/변경할 수 있다.

---

## FR-10 Headless Mode

GUI 없이:

```bash
poselink_vision_node --headless ...
```

형태로 실행 가능해야 한다.

### 이유

자동 benchmark에서 GUI render cost와 사용자 조작을 제거하기 위함.

---

## FR-11 Viewer Diagnostics

최소 표시:

- receive rate
- packet loss/reorder/duplicate
- latest packet age
- pose buffer occupancy
- configured buffer delay
- interpolation/extrapolation state
- render FPS

GUI는 선택 기능이며 core Viewer는 UI code에 의존하지 않는다.

---

# 4. 비기능 요구사항

## NFR-01 Bounded Memory

다음은 무제한 증가하면 안 된다.

- receive queue
- PoseBuffer
- preview frame queue
- log buffer

### 목표

```text
PoseBuffer <= 256 samples
network queue <= 256 samples
preview queue = latest-frame 또는 <= 3 frames
```

10분 soak test에서 queue 크기가 계속 증가하지 않는다.

---

## NFR-02 Freshness-first Backpressure

producer가 consumer보다 빠를 때 실시간 state stream에서는 오래된 데이터가 계속 밀려 있으면 안 된다.

기본 policy:

```text
bounded queue
overflow → DROP_OLDEST
```

drop 수는 metric으로 기록한다.

---

## NFR-03 Clean Shutdown

다음 상태에서 종료 가능해야 한다.

- camera active
- UDP receive blocking
- sender worker active
- marker detection 중

### 목표

정상 종료 요청 후:

```text
2 seconds 이내 process 종료
```

worker detach 없이 join 완료.

---

## NFR-04 Thread Ownership

- OpenGL / ImGui 호출은 UI/render thread
- socket/capture worker의 ownership 명확화
- shared mutable data 최소화
- 필요한 경우 mutex/atomic/queue 사용

### 합격

AddressSanitizer/ThreadSanitizer를 적용 가능한 target에서 치명적 lifetime/data-race 문제 없이 테스트한다.

---

## NFR-05 Error Handling

다음 오류가 process crash로 바로 이어지지 않아야 한다.

- camera open failure
- calibration file 없음
- invalid marker size
- socket bind failure
- malformed packet
- unsupported protocol version

오류 원인과 현재 state를 로그 또는 GUI에 표시한다.

---

## NFR-06 Separation

다음 compile-time dependency를 금지한다.

```text
common → OpenGL/OpenCV/flecs
transport → viewer
vision → Renderer
protocol → GUI
```

---

# 5. 실시간 성능 목표

아래 값은 **Release build의 초기 만족 기준**이다.

## 5.1 Vision Pose Rate

720p 또는 동급 입력, 단일 marker 기준:

```text
capture rate target       >= 30 FPS
valid pose update target  >= 25 Hz
UDP publish target        = 30 Hz configurable
```

카메라 자체가 30 FPS 미만이면 실제 입력 upper bound를 별도 기록한다.

---

## 5.2 Viewer Rendering

1280×720, tracked object 1~10개 기준:

```text
render target          >= 60 FPS
frame-time p95         <= 16.7 ms
```

VSync 대기 시간을 포함/제외했는지 측정 문서에 명시한다.

---

## 5.3 Localhost E2E

Synthetic 30 Hz, network impairment 없음:

```text
Pose 생성 timestamp
→ Viewer application까지의 packet age

p50 <= 20 ms
p95 <= 50 ms
```

환경에 따라 실패하면 측정값과 병목을 기록하고 기준을 재검토한다. 기준을 맞추기 위해 숫자를 숨기지 않는다.

---

# 6. Vision 품질 목표

실제 camera 수치는 hardware/조명/marker 크기에 영향을 받으므로 측정 조건과 함께 사용한다.

## 6.1 Camera Calibration

ChArUco calibration:

```text
RMS reprojection error <= 1.0 px
```

권장 목표:

```text
<= 0.7 px
```

1.0 px를 넘으면 sample image / board coverage / focus를 다시 확인한다.

---

## 6.2 Stationary Pose Stability

조건 예:

```text
marker distance = 0.5 m
camera fixed
marker fixed
sample = 10 s
```

초기 목표:

```text
position stddev <= 5 mm
orientation stddev <= 1.5 deg
```

달성하지 못하면 network 측정 전에 Vision noise와 network jitter를 분리하여 보고한다.

---

## 6.3 Tracking Recovery

marker가 다시 보인 뒤:

```text
<= 200 ms
```

안에 valid pose publish 상태로 복귀하는 것을 목표로 한다.

---

# 7. Network Impairment 기준 실험

Synthetic source를 사용해 Vision noise를 제거한다.

기준 profile:

```text
send rate    = 30 Hz
base delay   = 50 ms
jitter       = ±20 ms
loss         = 1%
reorder      = 1%
duration     = 120 s
```

비교:

```text
Mode A: Immediate Rendering
Mode B: Buffered Interpolation
```

buffer delay 후보:

```text
0 / 20 / 40 / 60 / 80 ms
```

---

# 8. 안정성 / latency trade-off 만족 기준

프로젝트의 핵심 성공 기준이다.

## 측정 지표

### Pose Age

```text
render monotonic time
-
rendered pose source timestamp
```

### Position Error

Synthetic ground truth:

```text
|| rendered_position - expected_position(renderTime) ||
```

### Rotation Error

quaternion angular distance.

### Visual Motion Jitter

ground truth 대비 frame-to-frame velocity/acceleration error 또는 trajectory error로 계산한다.

---

## Target 결과

기준 impairment profile에서 Buffered Interpolation이 Immediate 대비:

```text
trajectory / motion jitter metric
>= 30% 개선
```

을 목표로 한다.

동시에 선택한 buffer policy에서:

```text
rendered pose age p95 <= 150 ms
```

를 만족하는 것을 초기 목표로 한다.

즉 다음 둘을 동시에 만족해야 한다.

```text
안정성 개선
+
지연 무제한 증가 금지
```

---

# 9. Packet Robustness 목표

120초 impairment test 동안:

```text
invalid memory access = 0
decode crash = 0
buffer overflow = 0
```

metric에서:

```text
injected loss/reorder
↔
measured loss/reorder
```

가 실험 허용 범위 내에서 일치해야 한다.

---

# 10. CPU / Memory 목표

초기 프로젝트 규모에서는 절대 수치보다 **bounded behavior**를 우선한다.

### CPU

Viewer와 Vision Node 각각:

```text
idle busy-loop 금지
worker가 입력 없을 때 100% core 점유 금지
```

### Memory

10분 steady-state test:

```text
RSS가 지속적으로 단조 증가하지 않음
```

초기 warm-up / driver allocation은 제외하고 누수 여부를 별도 확인한다.

---

# 11. 완료 기준

## MVP Complete

다음 경로가 동작하면 MVP:

```text
SyntheticPoseSource
→ UDP
→ Viewer
→ Transform
→ OpenGL Cube
```

그리고 sequence/timestamp를 확인할 수 있다.

---

## Project Complete

다음을 모두 만족하면 기본 프로젝트 완료:

- Synthetic / ArUco가 같은 Pose publish interface 사용
- 별도 Vision Node / Viewer process
- binary UDP protocol
- sequence/timestamp metrics
- bounded PoseBuffer
- LERP/SLERP
- OpenCV → OpenGL coordinate conversion
- Vision Node ImGui preview/diagnostics
- headless experiment mode
- network impairment 실험
- Immediate vs Buffered 결과 비교
- 선택한 buffer delay의 근거를 수치로 설명

---

## Stretch Complete

다음 중 하나 이상:

- robot model hierarchy
- FK
- target Pose 기반 IK
- multiple tracked objects
- adaptive jitter buffer
- recording/replay
- protocol version compatibility test

Stretch 기능은 기본 완료 기준을 대체하지 않는다.
