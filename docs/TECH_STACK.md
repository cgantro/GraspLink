# GraspLink Technology Stack

## 1. 기술 선택 원칙

GraspLink는 **단일 C++ 로봇 시뮬레이터**에 필요한 기술만 사용한다.

1. Rendering과 robot kinematics를 분리한다.
2. 외부 장치/네트워크/Vision dependency를 두지 않는다.
3. Robot asset과 kinematic description을 분리한다.
4. Windows/Linux에서 공통 core를 재사용한다.
5. 새 library는 실제 필요가 생길 때만 추가한다.
6. 성능 수치는 Release build에서 baseline을 측정한 뒤 기록한다.

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

필요 정보:

```text
Robot Base
Joint origin
Joint axis
Joint limits
Parent / Child link
Link mesh
End Effector frame
Gripper mount transform
```

4DoF 모델은 코드 또는 별도 config에 명시적으로 정의한다. GLB node hierarchy를 그대로 신뢰하지 않는다.

### FK

FK는 직접 구현해 transform chain을 이해하고 검증하는 것을 우선한다.

```text
q1..q4
→ local joint transforms
→ accumulated link transforms
→ T_base_ee
```

### IK

초기 범위는 4DoF target에 맞춘 analytic 또는 numerical solver를 사용한다. numerical IK가 필요하면 Jacobian 기반 Damped Least Squares를 우선 후보로 둔다.

Solver는 rendering/Flecs에 의존하지 않는다.

---

## 5. Asset

Robot과 Gripper는 별도 GLB asset으로 유지할 수 있다.

```text
Robot GLB
→ Base / Link mesh

Gripper GLB
→ Gripper body / jaw mesh
```

필요 시 CAD/GLB를 link 단위 asset으로 전처리한다. 실제 관절 pivot과 axis는 RobotDescription에서 관리한다.

---

## 6. 좌표계 / 수학

필요한 수학 영역:

- 3D vector/matrix
- quaternion
- rigid transform
- homogeneous coordinate
- parent-child transform accumulation
- FK
- IK/Jacobian
- position/orientation error metric

기본 공간 단위는 meter, 내부 각도는 radian을 사용한다.

---

## 7. 단계별 기술

| 단계 | 기술 |
|---|---|
| Synthetic Target/Object | OpenGL/GLM/Flecs |
| Robot Asset | GLB mesh / asset preprocessing |
| Robot Description | C++ data/config |
| FK | GLM 기반 transform math |
| Grasp Pose | frame composition |
| IK | analytic 또는 Jacobian/DLS |
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
