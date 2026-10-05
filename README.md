# GraspLink

C++17/OpenGL/Flecs 기반 HCR-12A + 2F-85 로봇 시뮬레이터 프로젝트다.
Controller가 관절 상태를 갱신하면 공통 FK 결과를 Viewer의 J1~J6 변환과 Jolt Kinematic 충돌 프록시에 반영한다. 2F-85는 authored Gripper/joint 계층을 따라가는 mesh별 Kinematic hull proxy를 갖지만, Gripper controller나 grasp 동역학은 아직 구현하지 않았다. FK ToolFrame은 backend TCP feedback과 별도이며, IK·가속도 제한·관절 동역학도 구현 범위 밖이다.

## 디렉터리

```text
apps/
└─ viewer/                         # 프로그램 조립 / main loop

modules/
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

화면 없이 robotics/physics 테스트만 실행하려면:

```powershell
cmake -S . -B build-headless-debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DGRASPLINK_BUILD_GRAPHICS=OFF -DBUILD_TESTING=ON
cmake --build build-headless-debug --parallel
ctest --test-dir build-headless-debug --output-on-failure
```

실행 파일은 `grasplink_simulator`다. Shader/GLB가 복사된 실행 파일 directory에서 실행한다.

```powershell
cd build-ninja-debug
.\grasplink_simulator.exe
.\grasplink_simulator.exe --physics-demo
```

기본 실행은 로봇과 바닥을 표시한다. `--physics-demo`는 Debug/Release 모두에서 50개 낙하 Box와 로봇 데모 명령을 추가한다. GUI의 `Show configured colliders`는 ECS 설정의 X-ray 근사 외곽선이며 Jolt 실제 형상 조회가 아니다.

모듈 경계는 [Architecture](docs/ARCHITECTURE.md), 물리 제약은 [Physics / Flecs Integration](docs/PHYSICS_ECS_INTEGRATION.md), 현재 점검 결과는 [Code Documentation Audit](docs/CODE_DOCUMENTATION_AUDIT.md)를 참고한다.

그리퍼 충돌, 팔 형상 수 감소와 한글 Doxygen 보강 결과는 [후속 작업 결과](docs/GRIPPER_COLLISION_AND_COMMENT_RESULTS.md)에 정리했다.
