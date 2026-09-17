# GraspLink Technology Stack

## 1. 기술 선택 원칙

GraspLink는 **단일 C++ 로봇 시뮬레이터**에 필요한 기술만 사용한다.

1. Rendering과 robot kinematics를 분리한다.
2. 외부 장치/네트워크/Vision dependency를 두지 않는다.
3. Robot asset과 kinematic description을 분리한다.
4. CAD assembly hierarchy와 6DoF kinematic hierarchy를 구분한다.
5. Windows/Linux에서 공통 core를 재사용한다.
6. 새 library는 실제 필요가 생길 때만 추가한다.
7. 성능 수치는 Release build에서 baseline을 측정한 뒤 기록한다.

---

## 2. 현재 실제 사용 기술

| 영역 | 기술 | 상태 |
|---|---|---:|
| Language | C++17 | 사용 |
| Build | CMake 3.20+ | 사용 |
| Rendering | OpenGL | 사용 |
| Window / Context | GLFW 3.4 | 사용 |
| OpenGL Loader | GLAD | 사용 |
| Math | GLM 1.0.1 | 사용 |
| ECS | Flecs 4.1.5 | 사용 |
| Robot kinematics | 자체 C++ 구현 예정 | 미구현 |
| GLB asset | static mesh loader / asset preprocessing | 진행 중 |

OpenCV, UDP socket, Zephyr/ESP32 관련 dependency는 프로젝트에서 제거한다.

---

## 3. Simulator

실행 파일:

```text
grasplink_simulator
```

현재 stack:

| 영역 | 기술 | 역할 |
|---|---|---|
| Window | GLFW | window/event/OpenGL context |
| Rendering | OpenGL Core profile | scene draw |
| Loader | GLAD | OpenGL function loading |
| Math | GLM | matrix/quaternion/transform |
| ECS | Flecs | scene entity/component/system |
| Build | CMake | target/dependency 구성 |

---

## 4. Robot Model / Kinematics

기준 모델은 **HCR-12A 6DoF robot arm + 2F85 gripper**다.

필요 정보:

```text
Robot Base
J1~J6 origin
J1~J6 axis
J1~J6 joint limits
Parent / Child link
Link mesh mapping
End Effector frame
2F85 Gripper mount transform
```

6DoF 모델은 코드 또는 별도 config에 명시적으로 정의한다. STEP→GLB 변환으로 CAD assembly hierarchy/local transform은 보존하되 GLB node hierarchy 자체를 kinematics의 source of truth로 사용하지 않는다.

### FK

FK는 직접 구현해 6축 transform chain을 이해하고 검증하는 것을 우선한다.

```text
q1..q6
→ local joint transforms
→ accumulated Link1..Link6 transforms
→ T_base_ee
```

### IK

IK target은 End Effector의 6D pose(position + orientation)다. numerical IK가 필요하면 Jacobian 기반 Damped Least Squares를 우선 후보로 둔다.

```text
Target SE(3) Pose
→ position/orientation error
→ Jacobian / DLS
→ Δq1..Δq6
→ joint limit 적용
```

Solver는 rendering/Flecs에 의존하지 않는다.

---

## 5. Asset

Robot과 Gripper는 별도 GLB asset으로 유지한다.

```text
HCR-12A GLB
→ CAD assembly/part meshes

2F85 GLB
→ base / finger / fingertip meshes
```

STEP→GLB 변환 시 assembly tree와 local transform을 보존한다. 이후 HCR-12A visual node들을 Link1~6에 매핑하고 실제 관절 pivot/axis는 RobotDescription에서 관리한다.

---

## 6. 좌표계 / 수학

필요한 수학 영역:

- 3D vector/matrix
- quaternion
- rigid transform / SE(3)
- homogeneous coordinate
- parent-child transform accumulation
- 6DoF FK
- Jacobian 기반 IK
- position/orientation error metric

기본 공간 단위는 meter, 내부 각도는 radian을 사용한다.

---

## 7. 단계별 기술

| 단계 | 기술 |
|---|---|
| Synthetic Target/Object | OpenGL/GLM/Flecs |
| HCR-12A / 2F85 Asset | STEP→GLB + assembly-preserving preprocessing |
| Robot Description | C++ data/config |
| 6DoF FK | GLM 기반 transform math |
| 6D Grasp Pose | frame composition / SE(3) |
| 6DoF IK | Jacobian/DLS 우선 검토 |
| Joint Tracking | frame `dt`, speed/step limit |
| Attach/Release | scene transform ownership/state machine |
| Verification | deterministic test + debug visualization + timing |

---

## 8. 기본 범위에서 사용하지 않는 기술

```text
ESP32 / Zephyr RTOS
OpenCV / ArUco
UDP / WinSock / POSIX socket
ROS2
Physics engine
```

필요성이 생기기 전까지 프로젝트에 다시 추가하지 않는다.

---

## 9. 참고 자료

- Flecs: https://www.flecs.dev/flecs/
- OpenGL Wiki: https://wikis.khronos.org/opengl/Main_Page
- GLFW: https://www.glfw.org/docs/latest/
- GLM: https://github.com/g-truc/glm
- Khronos glTF: https://www.khronos.org/gltf/
- Modern Robotics: https://modernrobotics.northwestern.edu/
