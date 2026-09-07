# PoseLink Technology Stack

## 1. 기술 선택 원칙

PoseLink의 기술 스택은 기능 목록보다 **프로그램별 책임과 의존 경계**를 기준으로 정한다.

1. Vision Node와 Viewer/Simulator를 별도 process로 유지한다.
2. Vision Node는 Pose 생성과 송신에 집중하고 graphics dependency를 갖지 않는다.
3. Viewer/Simulator는 OpenGL과 Flecs를 사용해 scene과 robot을 표현한다.
4. Robot kinematics 계산은 rendering API와 분리한다.
5. Windows/Linux에서 공통 core를 최대한 재사용한다.
6. 아직 필요하지 않은 대형 framework는 먼저 추가하지 않는다.
7. 성능 수치는 실제 구현 후 Release build에서 측정한다.

---

## 2. 현재 실제 사용 기술

| 영역 | 기술 | 현재 사용 여부 | 코드/설정 근거 |
|---|---|---:|---|
| Language | C++17 | 사용 | root `CMakeLists.txt` |
| Build | CMake 3.20+ | 사용 | root `CMakeLists.txt` |
| Rendering | OpenGL | 사용 | `modules/viewer/src/graphics/*` |
| Window / Context | GLFW 3.4 | 사용 | `cmake/Dependencies.cmake` |
| OpenGL Loader | GLAD | 사용 | `third_party/glad`, `cmake/Dependencies.cmake` |
| Math | GLM 1.0.1 | 사용 | `cmake/Dependencies.cmake`, Viewer code |
| ECS | Flecs 4.1.5 | 사용 | `cmake/Dependencies.cmake`, `RenderSystem` |
| Domain Pose | 자체 `Pose` type | 사용 | `modules/common/include/Pose.h` |
| OpenCV | 예정 | 미사용 | CMake dependency 없음 |
| UDP | 예정 | 미사용 | `Protocol/UdpSocket` 파일은 현재 빈 골격 |
| Robot kinematics | 예정 | 미사용 | module/path 미정 |

---

## 3. Viewer / Simulator

실행 파일:

```text
poselink_viewer
```

현재 stack:

| 영역 | 기술 | 역할 |
|---|---|---|
| Window | GLFW | window, event, OpenGL context |
| Rendering | OpenGL Core profile | scene draw |
| Loader | GLAD | OpenGL function loading |
| Math | GLM | matrix/quaternion |
| ECS | Flecs | Entity/Component/System 관리 |
| Build | CMake | target 구성 |

현재 `RenderSystem`은 Flecs module/system으로 등록되고 `world.progress(dt)`에서 실행된다.

향후 Viewer에 필요한 기술:

| 단계 | 예정 기술 | 용도 |
|---|---|---|
| UDP Object Pose | OS UDP socket wrapper | Pose 수신 |
| Simulation Object | Flecs Entity + `Transform` | tracked object 표현 |
| Robot Model | mesh + joint/link description | 다관절 모델 구성 |
| FK/IK | 자체 C++ math 또는 검증된 linear algebra 보조 | kinematics 계산 |
| Diagnostics | Dear ImGui 선택 | viewer metrics/debug overlay |

Dear ImGui는 Viewer가 이미 graphics application이기 때문에 필요 시 diagnostics 용도로만 검토한다.

---

## 4. Vision Node

목표 실행 파일:

```text
poselink_vision_node
```

목표 stack:

| 영역 | 기술 | 역할 |
|---|---|---|
| Camera / CV | OpenCV | `VideoCapture`, calibration, ArUco, `solvePnP` |
| Calibration | ChArUco | camera intrinsic/distortion 추정 |
| Runtime Tracking | ArUco + `solvePnP` | known object의 6DoF Pose 추정 |
| Network | UDP | object pose publish |
| Serialization | custom binary protocol | ABI/endianness 독립 wire format |
| Runtime UI | 없음 | headless producer 유지 |
| Debug Preview | OpenCV HighGUI 선택 | 개발 중 marker/axis 확인 |

Vision Node는 다음을 링크하지 않는 것을 기본 원칙으로 한다.

```text
OpenGL
GLFW
GLAD
Flecs
Dear ImGui
Qt
MFC
```

---

## 5. Vision Node GUI 검토 결과

### Dear ImGui

장점:
- C++ 친화적
- 실시간 diagnostics에 적합
- GLFW/OpenGL backend 존재

단점:
- Vision Node에 window/graphics lifecycle이 새로 생김
- `cv::Mat` preview를 texture로 올리는 추가 경로 필요

결론:

```text
Vision Node: 사용하지 않음
Viewer diagnostics: 필요 시 사용 가능
```

### Qt

장점:
- 복잡한 desktop tool, calibration wizard, persistent settings에 적합

단점:
- 현재 범위에는 dependency/framework 비중이 큼

결론:

```text
향후 별도 운영 도구가 필요할 때 재검토
```

### MFC

장점:
- Windows native industrial application에 적합

단점:
- Windows 전용
- 현재 cross-platform 목표와 충돌

결론:

```text
사용하지 않음
```

### OpenCV HighGUI

장점:
- OpenCV dependency 안에서 camera frame을 즉시 확인 가능

단점:
- 제품 GUI가 아님

결론:

```text
--preview 같은 개발용 optional 기능으로만 사용
```

---

## 6. Robot Model / Kinematics 기술 선택

Robot 단계는 아직 구현되지 않았으므로 library를 확정하지 않는다.

필요 정보:

```text
Robot Base
Joint origin
Joint axis
Joint limits
Link mesh
End Effector frame
```

초기 후보:

- UR5/UR5e 계열
- Franka Panda
- xArm 계열

선정 기준:

1. simulation에 사용할 mesh/robot description을 합법적으로 구할 수 있는가
2. joint origin/axis/limit 정보가 공개되어 있는가
3. 6DoF grasp target을 표현하기 적절한 자유도를 갖는가
4. 직접 FK 결과를 외부 reference와 비교할 수 있는가

### URDF

Robot model 데이터 원본으로 URDF를 사용할 수 있다. 다만 처음부터 full URDF parser를 만드는 것은 필수로 두지 않는다.

초기 구현은 특정 robot 하나에 필요한 joint/link parameter만 명시적으로 읽거나 config로 고정할 수 있다.

### FK / IK

FK는 직접 구현해 transform chain을 이해하는 것을 우선한다.

IK는 다음 단계에서 선택한다.

- analytic IK: 선택한 robot에 적절하고 구현 범위가 감당 가능할 때
- numerical IK: Jacobian 기반, pseudo-inverse 또는 Damped Least Squares 후보

현재는 solver를 확정하지 않는다.

---

## 7. Network Stack

UDP 선택 이유:

```text
Object Pose = 지속적으로 갱신되는 상태
```

오래된 packet을 반드시 복구하는 것보다 최신 상태 반영이 중요하다.

다만 UDP를 선택했다고 reliability 문제를 무시하지 않는다. 마지막 단계에서 sequence/timestamp와 network impairment를 추가해 loss/reorder/jitter를 관측한다.

---

## 8. Binary Protocol

초기 wire format은 명시적 serialization을 사용한다.

피하는 방식:

```cpp
sendto(socket, &poseStruct, sizeof(poseStruct), ...);
```

이유:

- padding
- alignment
- compiler ABI
- endianness

프로토콜의 exact byte layout은 `docs/protocol.md`를 source of truth로 사용한다.

---

## 9. 좌표계 / 수학

프로젝트에서 필요한 수학 영역:

- 3D vector/matrix
- quaternion
- rigid transform
- homogeneous coordinate
- coordinate frame conversion
- FK transform chain
- Jacobian / numerical IK
- position/orientation error metric

Viewer의 GLM 타입을 domain/robot core 전체에 강제하지 않는다. Robot kinematics module을 만들 때 API 경계를 별도로 결정한다.

---

## 10. 단계별 dependency 도입

| 단계 | 새 dependency/기술 |
|---|---|
| Synthetic Pose → Cube | 현재 OpenGL/GLM/Flecs |
| UDP Object Pose | OS socket API, serialization |
| ArUco Object Detection | OpenCV |
| Simulation Object | 기존 Viewer/Flecs |
| Robot Model | robot description + mesh resource |
| FK | transform-chain math |
| Grasp Pose | frame composition |
| IK | Jacobian/solver math |
| Tracking / Attach | 기존 scene/kinematics |
| Network experiment | `tc netem` 또는 별도 UDP proxy |

새 library는 해당 단계에 실제 필요가 생길 때 추가한다.

---

## 11. 참고 자료

- Flecs: https://www.flecs.dev/flecs/
- OpenGL Wiki: https://wikis.khronos.org/opengl/Main_Page
- GLFW: https://www.glfw.org/docs/latest/
- GLM: https://github.com/g-truc/glm
- OpenCV ArUco: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- OpenCV solvePnP: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
- Dear ImGui backends: https://github.com/ocornut/imgui/blob/master/docs/BACKENDS.md
- ROS URDF tutorials: https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/URDF-Main.html
- Modern Robotics: https://modernrobotics.northwestern.edu/
