# Build and Run

## 1. Simulator

현재 root CMake executable target은 다음과 같다.

```text
grasplink_simulator
```

### 요구 환경

- CMake 3.20+
- C++17 compiler
- Git
- OpenGL development environment

현재 dependency:

- GLFW 3.4
- GLM 1.0.1
- Flecs 4.1.5
- GLAD
- OpenGL

### Configure

```bash
cmake -S . -B build -DGRASPLINK_BUILD_GRAPHICS=ON
```

### Build

Single-config generator:

```bash
cmake --build build -j
```

Visual Studio 등 multi-config generator:

```bash
cmake --build build --config Release
```

### Run

Single-config 예:

```bash
cd build
./grasplink_simulator
```

Windows multi-config 예:

```text
build/Release/grasplink_simulator.exe
```

CMake post-build 단계에서 `assets/shaders`를 executable directory의 `shaders/`로 복사한다.

---

## 2. 프로젝트 실행 경계

GraspLink는 단일 Simulator application만 실행한다.

다음 별도 runtime은 존재하지 않는다.

```text
Embedded Controller
Vision Node
UDP Sender/Receiver
Network Proxy
```

시뮬레이션 입력은 application 내부의 deterministic target/object state에서 생성한다.

---

## 3. 현재 구현 상태

현재 실행 가능한 중심 기능은 OpenGL/Flecs scene rendering이다.

HCR-12A 6DoF Robot Model/FK/IK와 2F85 Gripper/Grasp는 로드맵에 따라 단계적으로 추가한다. 문서의 목표 구조와 현재 구현 상태를 구분한다.

---

## 4. 권장 개발 순서

```text
기존 OpenGL scene 정상 렌더링
        ↓
Synthetic Target/Object
        ↓
HCR-12A / 2F85 assembly-preserving GLB 정리
        ↓
6DoF RobotDescription (J1~J6)
        ↓
FK
        ↓
Gripper mount / 6D Grasp Pose
        ↓
6DoF IK
        ↓
Joint tracking q1..q6
        ↓
Object Attach / Release
        ↓
Verification / Measurement
```

자세한 순서는 [ROADMAP.md](ROADMAP.md)를 따른다.
