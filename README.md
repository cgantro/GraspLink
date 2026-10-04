# GraspLink

C++17/OpenGL/Flecs 기반 HCR-12A + 2F-85 로봇 시뮬레이터 프로젝트다.
현재는 controller-ready GLB를 렌더링하고, backend-neutral robot controller를 통해 관절 상태를 시간에 따라 갱신한 뒤 Viewer adapter가 J1~J6 transform에 반영한다.

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
│  │  └─ backends/
│  │     └─ simulation/            # Simulation controller
│  └─ src/backends/simulation/
│
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
    RobotTransformAdapter
            ↓
         Flecs / GLB
```

`models/`는 로봇이 무엇인지 정의하고, `backends/`는 그 모델을 어떻게 움직이는지 구현한다.
새 로봇을 추가할 때 기존 Controller interface를 수정하지 않고 `models/<manufacturer>/<model>.h`를 추가하는 방향을 기준으로 한다.

## 빌드

```powershell
cmake --preset ninja
cmake --build --preset ninja-release
```

또는:

```powershell
cmake -S . -B build-ninja -G Ninja -DGRASPLINK_BUILD_GRAPHICS=ON
cmake --build build-ninja --config Release --parallel
```

실행 파일은 `grasplink_simulator`다.
