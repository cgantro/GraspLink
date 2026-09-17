# Build and Run

## 1. Simulator

현재 executable target:

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
cmake -S . -B build \
  -DGRASPLINK_BUILD_GRAPHICS=ON \
  -DGRASPLINK_BUILD_TESTS=ON
```

### Build

```bash
cmake --build build --config Release
```

### Test

```bash
ctest --test-dir build -C Release --output-on-failure
```

### Run

Windows multi-config 예:

```text
build/Release/grasplink_simulator.exe
```

CMake post-build 단계에서 shader와 HCR-12A GLB를 executable directory로 복사한다.

---

## 2. 실행 경계

GraspLink는 단일 Simulator application만 실행한다.

```text
Simulation Target/Object State
→ DLS IK
→ Joint Update
→ FK
→ HCR-12A Visual Rig
→ Grasp / Attach
→ OpenGL Render
```

별도 embedded controller, serial process, network sender/receiver, Vision Node는 없다.

---

## 3. 현재 구현 상태

현재 코드에는 다음이 포함되어 있다.

- HCR-12A nominal 6DoF robot specification
- FK / geometric Jacobian
- Damped Least Squares IK
- joint speed-limited update
- kinematic grasp/attach
- HCR-12A CAD visual rig
- OpenGL/Flecs scene rendering
- deterministic kinematics/grasp test

---

## 4. 권장 개발 순서

```text
HCR-12A / 2F85 asset 검증
        ↓
J1~J6 frame / limit 검증
        ↓
FK reference pose 검증
        ↓
6D Grasp Pose / Gripper mount
        ↓
DLS IK 정확도 / 수렴성 검증
        ↓
Joint tracking 개선
        ↓
Synthetic target pose 조작 UI
        ↓
Object Attach / Release
        ↓
Verification / Measurement
```

자세한 순서는 [ROADMAP.md](ROADMAP.md)를 따른다.
