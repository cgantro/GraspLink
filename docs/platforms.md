# 플랫폼과 의존성

## 1. 현재 빌드 기준

현재 root `CMakeLists.txt`에서 확인되는 기본 요구사항:

- C++17
- CMake 3.20+
- OpenGL development environment
- `POSELINK_BUILD_GRAPHICS=ON` 기본값

현재 Viewer dependency는 `cmake/Dependencies.cmake`에서 다음처럼 구성된다.

```text
GLFW 3.4        FetchContent
GLM 1.0.1       FetchContent
Flecs 4.1.5     FetchContent
GLAD            repository bundled source
OpenGL          find_package(OpenGL REQUIRED)
```

현재 실제 executable target:

```text
poselink_viewer
```

`poselink_vision_node`, transport/streaming test target, robot kinematics target은 아직 root CMake에 구성되지 않았다.

---

## 2. 플랫폼 목표

### Windows

목표:
- Viewer: GLFW + OpenGL
- UDP: WinSock wrapper
- Vision: OpenCV
- network impairment: 필요 시 별도 UDP proxy

### Linux

목표:
- Viewer: GLFW + OpenGL
- UDP: POSIX socket
- Vision: OpenCV
- network impairment: `tc netem`

현재 CMake가 Windows/Linux에서 모든 예정 module을 완성된 형태로 지원한다고 주장하지 않는다. Viewer가 먼저 대상이다.

---

## 3. 단계별 dependency

### 현재: Synthetic Pose → Cube

필요:

```text
C++17
CMake
OpenGL
GLFW
GLAD
GLM
Flecs
```

### UDP Object Pose

추가 예정:

```text
WinSock / POSIX socket abstraction
```

별도 network library를 도입할 필요가 있는지는 구현 복잡도를 본 뒤 판단한다.

### ArUco Object Detection

추가 예정:

```text
OpenCV
- core
- calib3d
- aruco
- videoio
- imgproc/highgui (debug preview가 필요할 때)
```

실제 CMake component 이름은 사용할 OpenCV package 구성에 맞춰 추가한다.

### Robot Model / FK / IK

아직 dependency 미정.

먼저 사용할 robot model과 description format을 결정한다.

가능한 추가 요소:

- URDF data
- mesh asset
- kinematics용 math helper

하지만 현재 GLM만으로 충분한 부분은 별도 library를 추가하지 않는다.

---

## 4. Vision Node의 graphics dependency

Vision Node는 기본적으로 다음을 링크하지 않는다.

```text
OpenGL
GLFW
GLAD
Flecs
Dear ImGui
```

개발 중 optional preview는 OpenCV HighGUI로 제한한다.

---

## 5. Clock / Network 측정 제약

동일 머신의 sender/viewer process에서는 monotonic time 기반 실험을 구성하기 쉽다.

서로 다른 PC에서는 `steady_clock` epoch를 직접 비교할 수 없으므로:

- NTP/PTP
- 별도 clock offset estimation

등이 없으면 sender timestamp만으로 정확한 one-way E2E latency를 주장하지 않는다.

---

## 6. 현재 source tree의 빈 골격

아래 path는 존재하지만 master 기준 구현 완료를 의미하지 않는다.

```text
apps/vision_node/
modules/vision/
modules/transport/
modules/streaming/
tests/test_main.cpp
```

문서에서는 파일 존재와 기능 구현을 구분한다.
