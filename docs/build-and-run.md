# Build and Run

## 1. 현재 빌드 가능한 범위

연결된 GitHub master 기준 root `CMakeLists.txt`에서 실제 executable target으로 구성된 것은 다음 하나다.

```text
poselink_viewer
```

현재 다음 기능은 아직 CMake target에 연결되어 있지 않다.

```text
poselink_vision_node
UDP transport / streaming
OpenCV vision pipeline
Robot kinematics
Automated tests
```

따라서 이 문서는 **현재 Viewer build 방법**과 **향후 단계별 target이 추가될 위치**를 구분해 설명한다.

---

## 2. 요구 환경

### 공통

- CMake 3.20 이상
- C++17 지원 compiler
- Git
- OpenGL development environment

### CMake가 현재 가져오는 dependency

`cmake/Dependencies.cmake` 기준:

- GLFW 3.4 — `FetchContent`
- GLM 1.0.1 — `FetchContent`
- Flecs 4.1.5 — `FetchContent`
- GLAD — `third_party/glad`에 포함
- OpenGL — `find_package(OpenGL REQUIRED)`

최초 configure 시 GLFW/GLM/Flecs fetch를 위한 네트워크 연결이 필요하다.

---

## 3. Configure

Repository root에서:

```bash
cmake -S . -B build -DPOSELINK_BUILD_GRAPHICS=ON
```

`POSELINK_BUILD_GRAPHICS`의 현재 기본값은 `ON`이다.

---

## 4. Build

### Single-config generator

Linux + Make/Ninja 등:

```bash
cmake --build build -j
```

Release를 명시하려면 configure 시:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### Multi-config generator

Visual Studio 등:

```bash
cmake --build build --config Release
```

---

## 5. 실행

CMake는 build 후 `assets/shaders`를 `poselink_viewer` executable directory의 `shaders/`로 복사한다.

따라서 executable directory에서 실행하면 shader relative path 문제를 피하기 쉽다.

### Single-config 예

```bash
cd build
./poselink_viewer
```

### Visual Studio / multi-config 예

실제 generator에 따라 보통 다음과 같은 위치가 된다.

```text
build/Release/poselink_viewer.exe
```

해당 executable directory에서 실행한다.

---

## 6. 현재 정상 동작 확인

현재 master의 Viewer는 다음을 확인할 수 있어야 한다.

- GLFW window 생성
- OpenGL context 초기화
- Camera/View/Projection 적용
- depth test 적용
- CubeA/CubeB 렌더링
- `flecs::world`의 `RenderSystem`이 `world.progress(dt)`에서 실행

프로젝트 진행 상태에서는 `Synthetic Pose → Cube`가 완료 단계로 관리되고 있으나, 연결된 master snapshot의 `SyntheticPoseSource` 파일은 아직 빈 상태다. 해당 local 구현이 push되면 이 항목을 실제 build 기준으로 다시 갱신한다.

---

## 7. 자주 확인할 오류

### OpenGL을 찾지 못함

예:

```text
Could NOT find OpenGL
```

확인:
- OS의 OpenGL development package / graphics SDK
- CMake generator/toolchain

### GLFW / GLM / Flecs fetch 실패

확인:
- 최초 configure 시 network access
- proxy/firewall
- `_deps` cache가 깨졌는지

필요하면 build directory를 새로 만든다.

```bash
rm -rf build
cmake -S . -B build
```

Windows에서는 build directory를 파일 탐색기 또는 PowerShell에서 삭제한다.

### Shader 파일을 열지 못함

현재 shader는 runtime relative path를 사용한다.

```text
shaders/Cube.glsl
```

post-build copy가 수행된 executable directory에서 실행했는지 확인한다.

### 창은 뜨지만 object가 보이지 않음

확인 순서:

1. shader compile/link error
2. camera position / projection
3. `Transform`
4. `Renderable` resource null 여부
5. `RenderSystem` import 여부
6. `world.progress(dt)` 호출 여부

---

## 8. 향후 target 계획

### 단계 2: UDP Object Pose

추가 예정:

```text
transport library
a synthetic sender executable 또는 vision_node synthetic mode
viewer receiver path
protocol tests
```

### 단계 3: ArUco Object Detection

추가 예정:

```text
OpenCV dependency
poselink_vision_node target
camera calibration/runtime options
```

### Robot 단계

사용할 robot model과 kinematics module 구조를 먼저 결정한 뒤 target/source를 추가한다.

문서에 target 이름을 미리 확정해 실제 CMake와 어긋나게 만들지 않는다.

---

## 9. Test command 주의

현재 `tests/test_main.cpp`는 빈 골격이며 root CMake에 `enable_testing()`, `add_test()`로 연결된 실제 test target이 확인되지 않는다.

따라서 현재 상태에서:

```bash
ctest --test-dir build
```

를 프로젝트 검증 방법이라고 문서화하지 않는다.

테스트 target이 추가된 뒤 이 문서를 갱신한다. 예정 검증 항목은 [testing.md](testing.md)를 참고한다.
