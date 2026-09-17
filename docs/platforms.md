# 플랫폼과 의존성

## 1. 현재 빌드 기준

현재 root `CMakeLists.txt` 기준 요구사항:

- C++17
- CMake 3.20+
- OpenGL development environment
- `GRASPLINK_BUILD_GRAPHICS=ON` 기본값

Viewer dependency:

```text
GLFW 3.4        FetchContent
GLM 1.0.1       FetchContent
Flecs 4.1.5     FetchContent
GLAD            repository bundled source
OpenGL          find_package(OpenGL REQUIRED)
```

현재 executable:

```text
grasplink_simulator
```

---

## 2. 플랫폼 목표

### Windows

- GLFW + OpenGL 기반 Simulator
- MSVC 또는 C++17 지원 compiler
- Robot kinematics/core는 platform-specific API에 의존하지 않음

### Linux

- GLFW + OpenGL 기반 Simulator
- GCC/Clang C++17
- 동일 robot kinematics/core 재사용

프로젝트는 외부 MCU, socket runtime, Camera device를 요구하지 않는다.

---

## 3. 단계별 dependency

### Scene / Synthetic Target

```text
C++17
CMake
OpenGL
GLFW
GLAD
GLM
Flecs
```

### Robot Model / FK / IK

현재 GLM 기반 math로 시작한다. 별도 solver/library는 실제 필요성이 확인되기 전에는 추가하지 않는다.

가능한 추가 요소:

- robot description config
- GLB/mesh asset preprocessing
- deterministic unit-test framework

---

## 4. 의도적으로 제외한 dependency

```text
Zephyr / ESP-IDF
OpenCV
WinSock / POSIX socket abstraction
ROS2
Physics engine
```

시뮬레이션 목표 달성에 필요하지 않으므로 기본 의존성에 포함하지 않는다.

---

## 5. Runtime 기준

정량 결과를 기록할 때 최소 다음을 명시한다.

```text
OS
CPU
GPU
Compiler
Build Type
Robot Model
Input Trajectory
Run Duration
```

성능 비교는 가능하면 Release build에서 수행한다.
