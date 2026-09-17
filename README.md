# GraspLink

GraspLink는 **C++로 구현하는 HCR-12A 6DoF 로봇팔 grasp 시뮬레이터**다.

프로젝트는 하나의 Simulator process 안에서 목표 물체를 생성하고, 6DoF FK/IK와 grasp/attach 과정을 직접 구현하고 검증하는 데 집중한다. 외부 임베디드 장치, serial/UDP transport, Camera/Vision Node는 프로젝트 경계에 포함하지 않는다.

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
                  └─ 2F85 Gripper
```

J1~J6은 revolute joint로 취급하며 실제 joint origin, axis, limit는 `RobotSpecification`/kinematics 계층에 명시한다. Gripper state는 6개 robot joint와 별도 상태로 관리한다.

IK target은 End Effector의 **6D pose(position + orientation)** 를 기준으로 한다.

## 프로젝트 흐름

```text
Synthetic Target / Object
        ↓
Simulation Scene
        ↓
6D Grasp Pose
        ↓
6DoF IK (DLS)
        ↓
Joint State q1..q6
        ↓
FK
        ↓
HCR-12A Link / End Effector Transform
        ↓
Grasp Condition
        ↓
Object Attach / Release
        ↓
OpenGL / Flecs Visualization
```

## 현재 구현

현재 repository에는 다음 기반이 구현되어 있다.

- OpenGL/GLFW/GLAD 기반 Viewer
- Flecs 기반 scene/render system
- HCR-12A nominal 6DoF `RobotSpecification`
- Forward Kinematics
- Geometric Jacobian
- Damped Least Squares IK
- Grasp pose / attach state
- HCR-12A CAD visual rig
- deterministic kinematics/grasp test
- static GLB loading과 HCR-12A mesh contract

외부 controller/transport/protocol/vision module은 사용하지 않는다.

## Asset 원칙

CAD에서 변환한 GLB는 렌더링 자산으로 사용한다. STEP→GLB 변환 시 assembly hierarchy와 local transform을 보존하되, CAD assembly tree 자체를 robot kinematics의 source of truth로 사용하지 않는다.

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
                                       └─ 2F85 Gripper
```

로봇 GLB와 그리퍼 GLB는 별도 asset으로 유지할 수 있으며, 그리퍼는 End Effector transform에 mount offset을 곱해 연결한다.

```text
T_world_gripper = T_world_ee × T_ee_gripper
```

## Repository 구조

```text
apps/
└─ viewer/                  # Simulator lifecycle / scene

modules/
├─ common/                  # Pose, JointState 등 공통 domain type
├─ kinematics/              # HCR-12A FK / Jacobian / DLS IK / grasp
└─ viewer/                  # OpenGL / Flecs rendering

assets/
├─ hcr12a/                  # HCR-12A visual asset
└─ shaders/

tools/
└─ cad/                     # CAD→GLB preprocessing

tests/                      # deterministic FK/IK/grasp test
docs/                       # architecture, kinematics, testing 등
```

## 빌드

요구 사항:

- C++17 compiler
- CMake 3.20+
- OpenGL development environment
- 최초 configure 시 GLFW, GLM, Flecs를 가져올 네트워크 연결

```bash
cmake -S . -B build -DGRASPLINK_BUILD_GRAPHICS=ON -DGRASPLINK_BUILD_TESTS=ON
cmake --build build --config Release
```

현재 executable target 이름은 `grasplink_simulator`다.

테스트:

```bash
ctest --test-dir build -C Release --output-on-failure
```

## 개발 단계

1. HCR-12A / 2F85 asset 정리
2. J1~J6 joint origin/axis/limit 검증
3. FK reference pose 검증
4. End Effector / 6D Grasp Pose 검증
5. 2F85 Gripper mount transform 연결
6. DLS IK 정확도/수렴성 검증
7. Joint trajectory/update 개선
8. Synthetic target pose 제어 UI 추가
9. Grasp 성공 시 Object Attach / Release
10. deterministic test와 성능/오차 측정

## 설계 원칙

- 프로젝트 경계는 단일 C++ Simulator process로 제한한다.
- Kinematics와 rendering을 분리한다.
- HCR-12A는 6DoF serial chain으로 정의한다.
- FK correctness를 먼저 고정한 뒤 IK를 검증한다.
- CAD assembly hierarchy와 kinematic hierarchy를 분리한다.
- Robot asset과 kinematic description을 분리한다.
- 초기 grasp는 rigid-body contact가 아닌 kinematic threshold + Object Attach로 제한한다.
- synthetic target을 사용해 같은 입력에서 같은 결과를 재현할 수 있게 한다.
- 수치 목표는 baseline 측정 후 정한다.

## 문서

- [Roadmap](docs/ROADMAP.md)
- [Architecture](docs/ARCHITECTURE.md)
- [HCR-12A Kinematics](docs/HCR12A_KINEMATICS.md)
- [HCR-12A CAD Conversion](docs/HCR12A_CAD_CONVERSION.md)
- [Project Background](docs/PROJECT_BACKGROUND.md)
- [Technology Stack](docs/TECH_STACK.md)
- [Coordinate System](docs/coordinate_system.md)
- [Build & Run](docs/build-and-run.md)
- [Testing](docs/testing.md)
- [Acceptance Criteria](docs/ACCEPTANCE_CRITERIA.md)
- [Design Decisions](docs/decisions.md)
- [Experiments](docs/experiments.md)
