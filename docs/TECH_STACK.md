# PoseLink Technology Stack

## 1. 기술 선택 원칙

PoseLink의 기술 스택은 다음 순서로 선택한다.

1. C++ 실시간 응용 소프트웨어의 데이터 흐름이 드러날 것
2. Viewer와 Vision Node를 별도 process로 유지할 것
3. Linux / Windows에서 동일 core를 최대한 재사용할 것
4. 네트워크/렌더링/비전 계층을 서로 강하게 결합하지 않을 것
5. Vision Node의 핵심 책임은 Pose 생성과 송신으로 제한할 것
6. benchmark와 자동 실험을 위해 GUI 없이 실행 가능할 것
7. 기능보다 framework 학습량이 더 커지는 선택은 피할 것

---

# 2. 공통 기술

| 영역 | 선택 | 용도 |
|---|---|---|
| Language | C++17 | 모든 native application / module |
| Build | CMake 3.20+ | target/의존성 구성 |
| Test | CTest + C++ test target | protocol/buffer/interpolation 검증 |
| Version Control | Git/GitHub | 소스 및 실험 기록 |
| Time | `std::chrono::steady_clock` | `dt`, monotonic timestamp |
| Network | BSD Socket / WinSock abstraction | Pose packet 송수신 |

---

# 3. Viewer 프로그램

실행 파일:

```text
poselink_viewer
```

## 핵심 스택

| 영역 | 기술 | 선택 이유 |
|---|---|---|
| Window / Context | GLFW | 작은 API, Windows/Linux 지원, OpenGL과 직접 결합하기 쉬움 |
| OpenGL Loader | GLAD | OpenGL function loading |
| Rendering | OpenGL 3.3 Core | graphics pipeline을 직접 제어 |
| Math | GLM | matrix/quaternion, OpenGL convention과 결합 |
| ECS | flecs | Viewer scene state와 반복 system 관리 |
| GUI (선택) | Dear ImGui | Viewer diagnostics/debug tooling |
| Asset | 초기에는 직접 Mesh/Shader/Texture | graphics abstraction을 직접 이해하기 위함 |

Viewer에서 ImGui는 core renderer를 대체하지 않는다.

```text
OpenGL Renderer
→ 3D Scene

Dear ImGui (optional)
→ Metrics / Controls / Diagnostics Overlay
```

Dear ImGui는 Viewer가 이미 GLFW + OpenGL context를 가지므로 추가 graphics architecture를 만들지 않고 붙일 수 있다.

---

# 4. Vision Node 프로그램

실행 파일:

```text
poselink_vision_node
```

## 핵심 스택

| 영역 | 기술 | 선택 이유 |
|---|---|---|
| Camera / CV | OpenCV | VideoCapture, calibration, ArUco, solvePnP |
| Calibration | ChArUco | corner 기반 camera calibration |
| Pose Estimation | ArUco + solvePnP | 크기를 아는 marker의 6DoF 추정 |
| Network | UDP | 최신 상태 우선, 손실을 application에서 측정 가능 |
| Serialization | Custom binary protocol | padding/ABI에 독립적인 packet |
| UI | **없음** | Vision Node의 책임을 Pose 생성/송신에 집중 |
| Debug Preview (선택) | OpenCV HighGUI | 개발 중 camera/detection 결과 확인 |
| Runtime Configuration | CLI arguments / config | headless 실행 및 자동 실험에 적합 |

기본 실행 구조:

```text
Camera / Synthetic Source
        ↓
Pose Estimation
        ↓
PoseSample
        ↓
Protocol Encode
        ↓
UDP Sender
```

Vision Node는 OpenGL, GLFW, flecs, Dear ImGui에 의존하지 않는다.

---

# 5. Vision Node GUI 후보 검토

Vision Node에 별도 GUI를 붙일 수 있는 후보로 Dear ImGui, Qt, MFC, OpenCV HighGUI를 비교했지만 **현재 프로젝트에서는 정식 GUI를 사용하지 않는다.**

이유는 Vision Node가 다음 역할의 프로그램이기 때문이다.

```text
Camera Input
→ Pose Estimation
→ Network Publish
```

별도 UI framework를 넣으면 Vision Node에 window/event/rendering lifecycle이 추가되어 핵심 pipeline보다 application framework의 비중이 커진다.

---

## 5.1 Dear ImGui

### 장점

- C++ 중심
- 실시간 diagnostics 표시에 편리
- GLFW / SDL / Win32 등 platform backend 제공
- OpenGL / DirectX / Vulkan renderer backend 제공
- Viewer에서는 기존 GLFW + OpenGL 환경에 쉽게 통합 가능

### 단점

- Vision Node에 사용하려면 결국 window + graphics backend가 필요
- camera preview를 표시하려면 `cv::Mat → GPU texture` 경로가 추가됨
- Pose 생성/UDP 송신 프로그램에 OpenGL context lifecycle이 생김
- headless node라는 구조적 장점이 약해짐

### PoseLink 결정

```text
Viewer diagnostics: 적합
Vision Node: 사용하지 않음
```

---

## 5.2 Qt Widgets

### 장점

- 완성도 높은 desktop UI toolkit
- Windows/Linux/macOS cross-platform
- layout, dialog, menu, file picker, table, model/view 제공
- Qt Designer 사용 가능
- 카메라 설정 프로그램이 제품 수준으로 커질 경우 적합

### 단점

- 현재 PoseLink 규모에는 dependency와 framework 비중이 큼
- event loop, deployment/runtime plugin 관리가 추가됨
- 핵심 실시간 pipeline보다 GUI application 구현 비중이 커질 수 있음

### PoseLink 결정

```text
현재: 사용하지 않음
향후 별도 운영/설정 도구가 필요해질 때 재검토
```

예를 들어 향후 아래 요구가 생기면 Qt를 고려할 수 있다.

```text
여러 Camera Profile 관리
Calibration Wizard
Persistent Settings
복잡한 장비 설정 화면
운영자용 Desktop Tool
```

이 경우에도 Vision Core와 Qt GUI는 별도 계층으로 분리한다.

---

## 5.3 MFC

### 장점

- Windows native desktop C++ 개발 경험
- Win32/MFC 기반 산업용 기존 코드베이스와 연결 시 실용적
- Visual Studio ecosystem과 결합

### 단점

- Windows 전용
- Linux 목표와 충돌
- 현재 OpenCV/UDP core의 cross-platform 구조에 플랫폼 분기가 생김
- 프로젝트의 핵심 기술 목표와 직접 관련이 적음

### PoseLink 결정

```text
사용하지 않음
```

MFC 자체 학습이 목표인 별도 Windows application이라면 의미가 있지만 PoseLink에 넣을 이유는 약하다.

---

## 5.4 OpenCV HighGUI

### 장점

- 이미 사용하는 OpenCV 안에서 camera frame을 바로 표시 가능
- 별도 GUI framework가 필요 없음
- ArUco corner, marker ID, axis 등의 detection debugging에 충분

예:

```cpp
cv::imshow("PoseLink Vision Debug", frame);
cv::waitKey(1);
```

### 단점

- 일반적인 application GUI toolkit이 아님
- 복잡한 설정/상태 UI에 부적합
- 자동 실험 환경에서는 window 자체가 불필요

### PoseLink 결정

```text
정식 GUI: 아님
개발용 optional debug preview: 사용 가능
```

`--preview` 같은 option으로 켜고 끌 수 있게 하는 정도가 적절하다.

---

# 6. GUI 최종 결정

## Vision Node

```text
GUI 없음
```

기본 실행:

```bash
poselink_vision_node \
  --camera 0 \
  --calibration camera.yml \
  --marker-size 0.05 \
  --host 192.168.0.10 \
  --port 5000 \
  --rate 30
```

선택적 debugging:

```bash
poselink_vision_node ... --preview
```

`--preview`가 켜진 경우에만 OpenCV HighGUI로 raw/detection frame을 표시한다.

### 표시 가능한 debug 정보

```text
Camera Image
Detected Marker Corners
Marker ID
Pose Axis
Capture FPS
Detection FPS
Current Pose
```

Target host/port, calibration path, marker size 같은 설정은 CLI/configuration으로 전달한다.

## Viewer

```text
OpenGL + GLFW
+
Dear ImGui diagnostics (optional)
```

Viewer는 애초에 graphics application이므로 ImGui를 붙여도 새로운 graphics dependency boundary가 생기지 않는다.

표시 후보:

```text
Receive Rate
Loss / Reorder / Duplicate
Packet Age
PoseBuffer Occupancy
Interpolation Delay
Render FPS
```

---

# 7. Vision Core와 Debug Preview 분리

OpenCV HighGUI 호출을 `ArUcoPoseSource`의 핵심 처리 코드 안에 직접 박지 않는다.

피해야 할 구조:

```cpp
ArUcoPoseSource::Update()
{
    ...
    cv::imshow(...);
}
```

권장 구조:

```text
ArUcoPoseSource
├─ Pose 결과
└─ optional debug frame / detection result
          ↓
VisionNodeApp
          ↓
DebugPreview (optional)
```

즉:

```text
Vision Core
= Camera / Detection / Pose

Application
= CLI / Loop / Publisher

Debug Preview
= 개발 보조 기능
```

으로 분리한다.

---

# 8. Network Stack

## UDP

선택 이유:

- 최신 Pose가 과거 Pose보다 중요
- transport-level retransmission으로 지연을 숨기지 않음
- loss/reorder를 application에서 직접 관찰 가능
- 작은 fixed-size Pose packet에 적합

TCP 대비 의도:

```text
TCP
→ reliable ordered byte stream

PoseLink UDP
→ freshness 우선 state update stream
```

UDP가 항상 우월해서가 아니라 프로젝트 요구가 다르기 때문에 선택한다.

---

# 9. Binary Protocol

목표 packet은 명시적 field serialization을 사용한다.

```text
Magic
Version
Flags
Sequence
Timestamp
Position XYZ
Quaternion WXYZ
Reserved / Check
```

하지 않는 것:

```cpp
sendto(socket, &cppStruct, sizeof(cppStruct), ...);
```

이유:

- padding
- alignment
- ABI
- endianness
- compiler 차이

프로토콜 상세는 `protocol.md`를 따른다.

---

# 10. OpenCV Stack

## Calibration

```text
ChArUco board
→ corner observations
→ camera matrix K
→ distortion coefficients
```

## Runtime pose

```text
VideoCapture
→ ArUco Detect
→ Marker Corners
→ solvePnP
→ rvec / tvec
→ Quaternion / Position
```

OpenCV의 역할은 **Pose 생성**까지다.

네트워크와 rendering 정책을 OpenCV class 안에 넣지 않는다.

---

# 11. Viewer의 flecs 사용 범위

사용:

- Entity identity
- `Transform`
- `Renderable`
- component query
- render system
- 향후 hierarchy / tracked tags

사용하지 않는 곳:

- UDP protocol encode/decode
- OpenCV pose estimation
- socket wrapper
- binary serialization
- Vision Node

ECS를 프로젝트 전체 framework로 강제하지 않는다.

---

# 12. 의존성 경계

최종 목표:

```text
poselink_vision_node
├─ C++17
├─ OpenCV
├─ transport/common modules
└─ OS socket API

poselink_viewer
├─ C++17
├─ GLFW
├─ GLAD
├─ OpenGL
├─ GLM
├─ flecs
├─ optional Dear ImGui
└─ streaming/transport/common modules
```

Vision Node가 다음을 링크하지 않는 상태를 유지한다.

```text
OpenGL
GLFW
GLAD
GLM (필요 없다면)
flecs
Dear ImGui
```

---

# 13. 의존성 추가 원칙

새 library는 아래 중 하나가 명확할 때만 추가한다.

```text
직접 구현 가치보다 검증된 library 사용 가치가 큰가?
platform abstraction이 필요한가?
현재 코드에서 실제 중복/복잡성이 발생했는가?
```

초기에는 다음을 추가하지 않는다.

- 대형 rendering engine
- full scene editor
- physics engine
- generic event bus
- service locator
- DI framework
- Vision Node용 desktop GUI framework
- Assimp/glTF abstraction before model requirement

---

# 14. 참고 자료

- Dear ImGui backends: https://github.com/ocornut/imgui/blob/master/docs/BACKENDS.md
- Qt 6 Widgets: https://doc.qt.io/qt-6/qtwidgets-index.html
- MFC overview: https://learn.microsoft.com/en-us/cpp/mfc/mfc-desktop-applications
- OpenCV HighGUI: https://docs.opencv.org/4.x/d7/dfc/group__highgui.html
