# GraspLink

C++17, OpenGL, Flecs, Jolt를 사용하는 HCR-12A + 2F-85 로봇 시뮬레이터다. 관절 상태를 공통 순기구학(FK)으로 변환해 화면 모델과 Jolt의 Kinematic 충돌 프록시에 적용한다. Damped Least Squares IK는 관절 공간 목표 이동(MoveJ)과 TCP 직선 경로(MoveL)에 쓰인다. Pick-and-Place의 장거리 이동에는 충돌 검사를 포함한 제한된 RRT-Connect를, 물체 주변 접근·하강·들기에는 MoveL을 사용한다.

시뮬레이터의 관절 가속 ramp와 그리퍼 개폐 매핑은 조정 가능한 정책 값이며 제조사 사양으로 검증된 값이 아니다. 그리퍼가 물체를 운반할 때는 양쪽 손끝이 같은 Dynamic 물체의 반대 면에 닿으면 고정 constraint를 만든다. 실제 힘 계산과 개별 손가락 적응은 구현하지 않았다. FK ToolFrame과 Controller의 TCP feedback은 별도 경로다.

## 디렉터리

```text
apps/
└─ simulator/                      # 프로그램 조립 / main loop

modules/
├─ diagnostics/                    # 비동기 로그·수치·성능 기록
├─ robotics/                       # Robot/Gripper domain
│  ├─ include/robotics/
│  │  ├─ core/                     # Controller contract / 공통 상태 타입
│  │  ├─ models/                   # 모델별 축, pivot, limit, gripper linkage
│  │  │  ├─ hanwha/
│  │  │  └─ robotiq/
│  │  ├─ backends/
│  │  │  └─ simulation/            # Simulation controller
│  │  ├─ kinematics/              # 공통 FK / ToolFrame
│  │  └─ runtime/                 # Fixed Control Loop
│  └─ src/                        # Simulation backend / FK
│
├─ physics/                        # Jolt World / Body API
├─ simulation/                     # ECS 물리 연결 / 로봇 충돌 프록시 / 바닥 설정
├─ gui/                            # ImGui / 설정된 collider 표시
└─ viewer/                         # OpenGL/Flecs/GLB visualization
   └─ include/graphics/            # Window, camera, and rendering APIs

assets/                             # GLB / shaders
docs/                               # 설계 / 사양 문서
```

## 문서 안내

| 필요한 내용 | 문서 |
| --- | --- |
| 모듈 책임과 의존성 | [Architecture](docs/ARCHITECTURE.md) |
| Controller 계약과 실행 주기 | [Controller Interface](docs/CONTROLLER_INTERFACE.md), [Control Simulation Goals](docs/CONTROL_SIMULATION_GOALS.md) |
| 로봇·그리퍼 모델과 데이터 출처 | [HCR-12A + 2F-85 Simulation Specs](docs/HCR12A_2F85_simulation_specs.md), [Model Data Provenance](docs/MODEL_DATA_PROVENANCE.md), [GLB Normalization](docs/HCR12A_GLB_NORMALIZATION.md) |
| 물리, 충돌, 이동·파지 동작 | [Physics / ECS Integration](docs/PHYSICS_ECS_INTEGRATION.md), [Self-Collision](docs/SELF_COLLISION.md), [Robot Motion and Grasp](docs/ROBOT_MOTION_AND_GRASP.md), [Gripper Runtime Design](docs/GRIPPER_RUNTIME_DESIGN.md) |
| IK·MoveL 학습 예제 | [HCR-12A TCP 이동 튜토리얼](docs/IK_MOVEL_TUTORIAL.md) |
| 진단 도구 | [Diagnostics](docs/DIAGNOSTICS.md) |
| 개발 이력과 경험 기록 | [STAR 기록 색인](docs/diary/README.md), [감사 결과](docs/history) |

설계 문서는 현재 구조와 제약을 설명하고, `docs/diary`는 시점별 작업·검증 이력을 보존한다. 두 곳의 상태 설명이 다르면 각 기록의 날짜와 근거 범위를 확인한다. 대화 요약과 커밋 목록은 사건의 맥락을 찾는 자료이며, 테스트 결과나 현재 코드 동작을 대신 증명하지 않는다.

## 책임 분리

```text
Application / IK / Planner
            ↓
      IRobotController
            ↓
   Simulation backend
            ↓
         RobotState
            ↓
      RobotKinematics
            └─ RobotKinematicState
                  ├─ RobotTransformAdapter → Flecs / GLB
                  └─ RobotPhysicsAdapter → Kinematic collider Entity
```

`RobotTransformAdapter`와 `RobotPhysicsAdapter` 구현은 `modules/simulation`에 있다. 앞의 adapter는 GLB 관절 Entity를 갱신하고, 뒤의 adapter는 별도 충돌 Entity를 갱신한다. `modules/viewer`는 렌더링을 담당한다.

`models/`는 로봇이 무엇인지 정의하고, `backends/`는 그 모델을 어떻게 움직이는지 구현한다. 현재 실행 가능한 backend는 시뮬레이션용이며 실제 하드웨어 backend는 제공하지 않는다.
새 로봇을 추가할 때 기존 Controller interface를 수정하지 않고 `models/<manufacturer>/<model>.h`를 추가하는 방향을 기준으로 한다.

그리퍼는 `IGripperController → GripperState → GripperKinematics → GripperTransformAdapter → GLB 관절 → 기존 충돌 proxy` 경로를 사용한다. 4 ms마다 두 Controller 갱신, 관절 자세 적용, World 변환 갱신, Jolt step, World 변환 재갱신 순서로 진행한다. raw 위치의 선형 fraction 매핑과 기본 master 속도 0.1..1.0 rad/s는 시뮬레이션 가정이며 제조사 보정식·속도 사양이 아니다. 계약과 검증 범위는 [그리퍼 런타임 설계](docs/GRIPPER_RUNTIME_DESIGN.md)에 정리했다.

GLB 노드와 Entity의 Local 회전은 단위 quaternion으로 저장한다. glTF의 `[x,y,z,w]`를 GLM 생성자 `(w,x,y,z)`로 옮기고, FK·Physics 자세도 quaternion으로 전달해 Euler 왕복 변환을 없앴다. 영 quaternion·NaN 등 비유한 입력은 거부하며 `q`와 `-q`는 같은 회전이다. Entity pose와 행렬은 여전히 float이고, 관절각·각속도는 rad·rad/s 스칼라를 사용한다. Orbit 카메라의 마우스 입력용 yaw/pitch 각도도 유지한다.

물리 갱신은 `PrePhysicsSync → Jolt Step → PostPhysicsSync`로 나눈다. Static·Kinematic은 Scene 자세를 따르고 Dynamic은 Physics 결과를 따른다. Dynamic은 SceneRoot 또는 항등 grouping 조상만 허용해 World 자세를 Local에 직접 저장한다. 상세 계층 조건과 목표 캐시는 [Physics / Flecs Integration](docs/PHYSICS_ECS_INTEGRATION.md)에 정리했다.

`GuiModule`은 ImGui context·입력 backend와 `BeginFrame/EndFrame` 수명을 담당한다. `ViewerApp`이 `GripperPanel`, `PhysicsDebugPanel`, `ColliderOverlay`를 별도로 만들고 두 프레임 호출 사이에 명시적으로 그린다. 패널은 요청·표시 상태를 보관하고, Overlay는 ECS collider 투영과 100 ms 선 캐시를 담당한다.

## 빌드

Windows에서는 Visual Studio의 **Developer PowerShell / x64 Native Tools Command Prompt**에서 실행한다. CMake, Ninja와 MSVC가 PATH에 있어야 한다. 의존성은 CMake FetchContent가 준비한다.

Debug:

```powershell
cmake --preset ninja
cmake --build --preset ninja-debug
ctest --test-dir build-ninja-debug --output-on-failure
```

Release:

```powershell
cmake --preset ninja-release
cmake --build --preset ninja-release
ctest --test-dir build-ninja-release --output-on-failure
```

Ninja는 단일 configuration generator이므로 Debug/Release는 별도 build directory와 `CMAKE_BUILD_TYPE`으로 선택한다. `--config Release`로 Debug directory를 Release로 바꾸지 않는다.

화면 없이 Robotics, Physics, ECS Simulation, GLB CPU 로더 테스트를 빌드하고 실행하려면:

```powershell
cmake -S . -B build-headless-debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DGRASPLINK_BUILD_GRAPHICS=OFF -DBUILD_TESTING=ON
cmake --build build-headless-debug --parallel
ctest --test-dir build-headless-debug --output-on-failure
```

실행 파일은 `grasplink_simulator`다. Shader/GLB가 복사된 실행 파일 directory에서 실행한다.

```powershell
cd build-ninja-debug
.\grasplink_simulator.exe
```

기본 실행은 HCR-12A와 2F-85, 바닥, 무작위 Pick&Place 상자와 목표 영역을 표시한다. GUI에서 그리퍼 Open/Close, 요청 개폐율·raw 속도, Activate/Reset/Stop을 조작한다. GUI의 force request는 128로 고정되며 실제 힘 효과를 계산하지 않는다. `Show configured colliders`는 ECS 설정의 X-ray 외곽선이며 Jolt가 만든 실제 형상을 조회하지 않는다. 렌더러는 MSAA와 기본 조명만 사용하며 shadow map은 만들지 않는다.

모듈 경계는 [Architecture](docs/ARCHITECTURE.md), 물리 제약은 [Physics / Flecs Integration](docs/PHYSICS_ECS_INTEGRATION.md), 과거 감사 기록은 [Code Documentation Audit](docs/history/CODE_DOCUMENTATION_AUDIT.md)을 참고한다.

Logger usage and file output policy are documented in [Diagnostics](docs/DIAGNOSTICS.md). Tracy provides runtime frame and zone timing; see the same document for setup.

그리퍼 충돌, 팔 형상 수 감소와 한글 Doxygen 보강 당시의 결과는 [후속 작업 기록](docs/history/GRIPPER_COLLISION_AND_COMMENT_RESULTS.md)에 정리했다.
