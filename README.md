# GraspLink

GraspLink는 **C++로 구현하는 6DoF 로봇팔 grasp 시뮬레이터**다.

외부 임베디드 장치, 네트워크 입력, Camera/Vision Node는 프로젝트 범위에서 제거한다. 하나의 Simulator process 안에서 목표 물체를 생성하고, 로봇의 FK/IK와 grasp/attach 과정을 직접 구현하고 검증하는 데 집중한다.

## Robot Arm 기준

로봇팔은 **HCR-12A 기반 6DoF serial manipulator + 2F85 gripper**를 기준으로 한다.

```text
J1
└─ J2
   └─ J3
      └─ J4
         └─ J5
            └─ J6
               └─ EndEffector
                  └─ Gripper
```

J1~J6은 각각 revolute joint로 취급하며 실제 joint origin, axis, limit는 `RobotDescription`에 명시한다. Gripper open/close는 6개 robot joint와 별도 actuator/state로 관리한다.

IK target은 End Effector의 **6D pose(position + orientation)** 를 기준으로 한다. 초기 구현은 FK correctness와 joint frame 검증을 먼저 완료한 뒤 numerical IK로 확장한다.

## 프로젝트 목표

```text
Synthetic Target / Object
        ↓
Simulation Scene
        ↓
6D Grasp Pose
        ↓
6DoF IK
        ↓
Joint State (q1..q6)
        ↓
FK
        ↓
Robot Link / End Effector Transform
        ↓
Grasp Condition
        ↓
Object Attach / Release
```

핵심은 로봇 모델의 좌표계와 6축 관절 계층을 직접 정의하고, FK로 계산한 링크 transform과 IK 결과를 시각적으로 검증한 뒤 kinematic grasp까지 연결하는 것이다.

## 개발 단계

1. Synthetic Target/Object 배치
2. HCR-12A 6DoF Robot Model 정리
3. J1~J6 joint origin/axis/limit 및 link hierarchy 정의
4. FK 구현과 기준 자세 검증
5. End Effector / 6D Grasp Pose 정의
6. 2F85 Gripper mount transform 연결
7. 6DoF IK 구현
8. Joint trajectory/update 적용
9. End Effector target pose 추종
10. Grasp 성공 시 Object Attach
11. Release 및 상태 전이
12. deterministic test와 성능/오차 측정

## Asset 원칙

CAD에서 변환한 GLB는 렌더링 자산으로 사용한다. STEP→GLB 변환 시 **assembly hierarchy와 local transform을 보존**하되, CAD assembly tree 자체를 robot kinematics의 source of truth로 사용하지 않는다.

HCR-12A CAD에는 J1/J2/1st Arm/J3-J4/2nd Arm/J5-J6/Tool IO 단위의 조립 계층이 존재하지만, 운동학 계층은 다음과 같은 6축 serial chain으로 별도 정의한다.

```text
Robot Base
└─ Joint1
   └─ Link1
      └─ Joint2
         └─ Link2
            └─ Joint3
               └─ Link3
                  └─ Joint4
                     └─ Link4
                        └─ Joint5
                           └─ Link5
                              └─ Joint6
                                 └─ Link6
                                    └─ EndEffector
                                       └─ Gripper
```

로봇 GLB와 그리퍼 GLB는 별도 asset으로 유지하고, 그리퍼는 End Effector transform에 mount offset을 곱해 연결한다.

```text
T_world_gripper = T_world_ee × T_ee_gripper
```

## Repository 구조

```text
apps/
└─ viewer/                  # C++ Simulator application

modules/
├─ common/                  # 공통 수학/domain type
└─ viewer/                  # OpenGL / Flecs rendering

assets/                     # shader 및 robot/object asset
docs/                       # architecture, roadmap, testing 등
tests/                      # deterministic test 확장 위치
```

외부 장치·네트워크·Vision 전용 디렉터리는 유지하지 않는다.

## 현재 상태

현재 repository의 실제 구현은 OpenGL/Flecs 기반 Simulator Viewer가 중심이다. Robot Model/FK/IK/Grasp는 이후 단계에서 구현하며, 구현되지 않은 기능은 완료된 것으로 간주하지 않는다.

## 빌드

요구 사항:

- C++17 compiler
- CMake 3.20+
- OpenGL development environment
- 최초 configure 시 GLFW, GLM, Flecs를 가져올 네트워크 연결

```bash
cmake -S . -B build -DGRASPLINK_BUILD_GRAPHICS=ON
cmake --build build --config Release
```

현재 executable target 이름은 `grasplink_simulator`다.

## 설계 원칙

- 프로젝트 경계는 단일 C++ Simulator process로 제한한다.
- Kinematics와 rendering을 분리한다.
- Robot arm은 HCR-12A 기반 6DoF serial chain으로 정의하고 gripper는 별도 상태로 둔다.
- FK correctness를 먼저 고정한 뒤 IK를 구현한다.
- CAD assembly hierarchy와 kinematic hierarchy를 분리한다.
- Robot asset과 kinematic description을 분리한다.
- 1차 grasp는 physics contact가 아닌 kinematic threshold + Object Attach로 제한한다.
- synthetic target을 사용해 같은 입력에서 같은 결과를 재현할 수 있게 한다.
- 수치 목표는 baseline 측정 후 정한다.

## 문서

- [Roadmap](docs/ROADMAP.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Project Background](docs/PROJECT_BACKGROUND.md)
- [Technology Stack](docs/TECH_STACK.md)
- [Coordinate System](docs/coordinate_system.md)
- [Build & Run](docs/build-and-run.md)
- [Testing](docs/testing.md)
- [Acceptance Criteria](docs/ACCEPTANCE_CRITERIA.md)
- [Design Decisions](docs/decisions.md)
- [Experiments](docs/experiments.md)
