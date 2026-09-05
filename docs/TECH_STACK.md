# PoseLink Technology Stack

## 1. 기술 선택 원칙

PoseLink의 기술 스택은 다음 순서로 선택한다.

1. C++ 실시간 응용 소프트웨어의 데이터 흐름이 드러날 것
2. Viewer와 Vision Node를 별도 process로 유지할 것
3. Linux / Windows에서 동일 core를 최대한 재사용할 것
4. 네트워크/렌더링/비전 계층을 서로 강하게 결합하지 않을 것
5. benchmark와 debugging을 위해 GUI 없이도 core가 실행 가능할 것
6. 기능보다 framework 학습량이 더 커지는 선택은 피할 것

---

# 2. 공통 기술

| 영역 | 선택 | 용도 |
|---|---|---|
| Language | C++17 | 모든 native application / module |
| Build | CMake 3.20+ | target/의존성 구성 |
| Math | GLM | Viewer vector/matrix/quaternion |
| ECS | flecs | Viewer scene state와 systems |
| Test | CTest + C++ test target | protocol/buffer/interpolation 검증 |
| Version Control | Git/GitHub | 소스 및 실험 기록 |
| Time | `std::chrono::steady_clock` | `dt`, monotonic timestamp |
| Network | BSD/WinSock UDP abstraction | Pose packet 송수신 |

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
| Rendering | OpenGL 3.3 Core | 직접 graphics pipeline 학습 및 제어 |
| Math | GLM | OpenGL convention과 잘 맞는 matrix/quaternion API |
| ECS | flecs | Entity/Component query 및 반복 system 관리 |
| GUI (선택) | Dear ImGui | diagnostics/debug tooling |
| Asset | 초기에는 직접 Mesh/Shader/Texture | graphics abstraction을 직접 이해하기 위함 |

Viewer에서 ImGui는 core renderer를 대체하지 않는다.

```text
OpenGL Renderer
→ 3D Scene

Dear ImGui
→ Metrics / Controls / Diagnostics Overlay
```

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
| GUI | **Dear ImGui + GLFW + OpenGL3** | engineering/debug UI에 적합 |
| Headless | GUI optional | 자동 실험 / benchmark 가능 |

---

# 5. Vision Node GUI 후보 조사

Vision Node GUI에 필요한 것은 일반 소비자용 desktop application UI가 아니다.

필수 화면:

```text
Camera Preview
Marker Detection Overlay
Current Position / Quaternion
Detection Status
Camera / Source FPS
UDP Send Rate
Sequence Counter
Target IP / Port
Calibration File
Marker Size
Start / Stop
Queue / Drop Metrics
```

즉 성격은:

```text
Camera engineering console
+
real-time diagnostics tool
```

에 가깝다.

---

## 5.1 Dear ImGui

### 장점

- C++ 중심
- immediate-mode 방식이라 실시간 상태를 표시하기 쉽다
- 기존 GLFW + OpenGL3 backend가 공식 제공된다
- 작은 dependency와 빠른 integration
- camera/pose/network metric처럼 매 frame 바뀌는 debug data에 적합
- Viewer에도 같은 GUI stack을 재사용할 수 있다
- MIT license
- Windows/Linux 양쪽에서 사용 가능
- headless core와 UI를 분리하기 쉽다

Dear ImGui의 공식 backend 문서에는 GLFW platform backend와 OpenGL3 renderer backend가 표준 backend로 제공된다고 명시되어 있다.

### 단점

- native desktop widget toolkit이 아니다
- 접근성, 복잡한 국제화, OS-native UX는 Qt보다 약하다
- `cv::Mat` camera frame을 ImGui에 표시하려면 texture upload 경로가 필요하다
- 일반 소비자용 application 수준의 polished UI에는 추가 작업이 많다

### PoseLink 적합도

```text
★★★★★
```

현재 목적과 가장 잘 맞는다.

---

## 5.2 Qt Widgets

### 장점

- 완성도 높은 desktop UI toolkit
- Windows/Linux/macOS cross-platform
- layout, dialog, menu, file picker, table, model/view 제공
- Qt Designer 사용 가능
- CMake 공식 지원
- camera configuration tool이 커지고 UI 복잡도가 높아질 때 유리
- native desktop application에 가까운 UX 구성 가능

### 단점

- PoseLink 규모에는 dependency가 크다
- Qt event loop와 현재 GLFW/OpenGL application loop를 함께 고려해야 한다
- 배포 시 Qt runtime/plugin 관리가 필요하다
- Viewer와 Vision Node의 low-level OpenGL 학습 코드보다 Qt framework가 더 큰 비중을 차지할 수 있다
- 라이선스 조건(LGPL/GPL 또는 commercial)을 프로젝트 배포 방식과 함께 확인해야 한다

### PoseLink 적합도

```text
★★★☆☆
```

**제품화된 desktop tool**로 커진다면 좋은 선택이지만 현재 학습/엔지니어링 프로젝트에는 과하다.

---

## 5.3 MFC

### 장점

- Windows native desktop C++ 개발 경험을 직접 보여줄 수 있다
- Visual Studio Resource Editor와 Win32 ecosystem에 익숙한 조직에서는 실용적
- Windows 전용 산업용/장비용 기존 코드베이스와 연결할 때 의미가 있다

### 단점

- Windows 전용
- Linux 목표와 충돌
- Visual Studio / Win32 중심
- OpenGL + OpenCV cross-platform 구조와 결합 시 플랫폼 분기가 커진다
- 신규 cross-platform engineering tool을 만들기 위한 선택으로는 비효율적
- 본 프로젝트의 GUI는 제품 UI보다 debug/visualization tool 성격이라 MFC의 장점이 크게 살아나지 않는다

### PoseLink 적합도

```text
★★☆☆☆
```

직무가 **Windows/MFC 유지보수 자체를 목표**로 할 때만 선택할 이유가 크다.

---

## 5.4 OpenCV HighGUI

비교 대상으로 함께 본다.

### 장점

- 가장 빠르게 camera frame을 띄울 수 있다
- OpenCV 외 추가 GUI dependency가 없다

### 단점

- 복잡한 control panel / status / docking / metrics UI에 부적합
- engineering console로 확장하기 어렵다

### 용도

```text
첫 ArUco detection spike
```

까지만 유용하다.

최종 Vision Node GUI로는 사용하지 않는다.

---

# 6. GUI 최종 결정

## 선택: Dear ImGui

Vision Node:

```text
GLFW Window
   ↓
OpenGL3 Context
   ├─ camera preview texture
   └─ Dear ImGui UI
```

Viewer:

```text
기존 GLFW + OpenGL3
   ├─ 3D scene
   └─ Dear ImGui diagnostics
```

두 프로그램에서 동일한 backend 조합을 사용할 수 있다.

```text
imgui_impl_glfw
+
imgui_impl_opengl3
```

### 선택 이유 요약

```text
Qt
→ UI 제품을 만드는 데 강함

MFC
→ Windows native/legacy application에 강함

Dear ImGui
→ 실시간 C++ visualization/debug tool에 강함
```

PoseLink의 GUI는 세 번째에 해당한다.

---

# 7. GUI와 Core 분리

GUI dependency가 core module로 전파되지 않게 한다.

금지:

```cpp
// ArUcoPoseSource.h
#include <imgui.h>
```

권장:

```text
ArUcoPoseSource
→ PoseResult / Diagnostics 반환

VisionNodeApp
→ ImGui에서 Diagnostics 표시
```

향후 구조:

```text
apps/vision_node
├─ VisionNodeApp
└─ VisionNodeUI

apps/viewer
├─ ViewerApp
└─ ViewerDiagnosticsUI
```

공통 ImGui wrapper가 실제로 중복되기 시작한 뒤에만:

```text
modules/ui
```

를 만든다.

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
→ Coordinate conversion
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

ECS를 프로젝트 전체 framework로 강제하지 않는다.

---

# 12. 의존성 추가 원칙

새 library는 아래 중 하나가 명확할 때만 추가한다.

```text
직접 구현 가치보다 검증된 library 사용 가치가 큰가?
platform abstraction이 필요한가?
현재 코드에서 실제 중복/복잡성이 발생했는가?
```

따라서 초기에는 다음을 추가하지 않는다.

- 대형 rendering engine
- full scene editor
- physics engine
- generic event bus
- service locator
- DI framework
- Assimp/glTF abstraction before model requirement

---

# 13. 참고 자료

- Dear ImGui backends: https://github.com/ocornut/imgui/blob/master/docs/BACKENDS.md
- Qt 6 Widgets: https://doc.qt.io/qt-6/qtwidgets-index.html
- MFC overview: https://learn.microsoft.com/en-us/cpp/mfc/mfc-desktop-applications
- OpenCV HighGUI: https://docs.opencv.org/4.x/d7/dfc/group__highgui.html
