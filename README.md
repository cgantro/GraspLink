# GraspLink

C++17/OpenGL/Flecs 기반 HCR-12A + 2F-85 로봇 시뮬레이터 프로젝트다.
Controller가 관절 상태를 갱신하면 공통 FK 결과를 Viewer의 J1~J6 변환과 Jolt Kinematic 충돌 프록시에 반영한다. Damped Least Squares IK는 관절 공간 목표 이동(MoveJ)과 TCP 직선 경로(MoveL)를 지원한다. 장거리 Pick-and-Place 이동은 충돌 검사와 제한된 RRT-Connect를 사용하는 MoveJ로 계획하고, 물체 주변 접근·하강·들기에는 MoveL을 사용한다. UI 계획은 프레임 단위로 나뉘어 진행한다. Cartesian 경로에는 TCP 기준 가속·감속 프로파일을, 관절 이동에는 최대 속도까지 0.20초를 기본값으로 하는 Simulation 가속 프로파일을 적용한다. 이 관절 ramp는 조정 가능한 시뮬레이터 정책이며 HCR-12A 제조사 가속도 사양이 아니다. 2F-85는 연속 개폐를 지원하며, 양쪽 손끝이 같은 Dynamic 물체의 반대 면에 닿으면 고정 constraint를 만들어 운반한다. 실제 힘 계산과 개별 손가락 적응은 구현하지 않았다. FK ToolFrame과 backend TCP feedback은 별도다.

## 디렉터리

```text
apps/
└─ viewer/                         # 프로그램 조립 / main loop

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
   ├─ include/viewer/robotics/     # Robot state -> Flecs transform adapter
   └─ src/robotics/

assets/                             # GLB / shaders
docs/                               # 설계 / 사양 문서
```

## 책임 분리

```text
Application / IK / Planner
            ↓
      IRobotController
            ↓
   Simulation / Hardware backend
            ↓
         RobotState
            ↓
      RobotKinematics
            ├─ RobotTransformAdapter → Flecs / GLB
            └─ RobotPhysicsAdapter → Kinematic collider Entity
```

`models/`는 로봇이 무엇인지 정의하고, `backends/`는 그 모델을 어떻게 움직이는지 구현한다.
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
