# PoseLink Project Background

## 1. 프로젝트가 해결하려는 문제

PoseLink의 최종 목표는 **원격 카메라에서 특정 물체의 6DoF Pose를 추정하고, 해당 Pose를 시뮬레이터로 전달한 뒤 가상 다관절 로봇팔이 그 물체를 집도록 하는 것**이다.

단순히 카메라 Pose를 화면에 표시하는 프로젝트가 아니다. 프로젝트가 연결하려는 핵심 경로는 다음과 같다.

```text
Physical Object
→ Camera Observation
→ Object 6DoF Pose
→ Network Transfer
→ Simulation Object
→ Grasp Pose
→ IK
→ Joint Angles
→ FK
→ Robot Link Transforms
→ Grasp
```

이 경로를 작은 C++ 코드베이스에서 직접 구성하면서 다음 문제를 분리해서 다룬다.

- Vision: 물체의 공간 상태를 어떻게 얻는가
- Network: 그 상태를 다른 process로 어떻게 전달하는가
- Graphics: 수신한 상태를 simulation world에 어떻게 표현하는가
- Robotics: object pose에서 grasp pose를 어떻게 만들고 robot joint를 어떻게 계산하는가
- Robustness: network delay/jitter/loss가 target pose와 grasp 안정성에 어떤 영향을 주는가

---

## 2. 왜 known object부터 시작하는가

초기 범위는 임의의 사물을 인식하고 자동으로 grasp point를 찾는 문제가 아니다.

```text
Known Object
+
Known Marker
+
Known Grasp Offset
```

을 사용한다.

예를 들어 물체에 ArUco marker를 부착하고 marker와 object frame의 상대 변환을 미리 알고 있으면:

```text
T_camera_object
=
T_camera_marker × T_marker_object
```

를 계산할 수 있다.

그리고 object frame에서 미리 정의한 grasp pose `T_object_grasp`를 사용하면:

```text
T_base_grasp
=
T_base_camera
× T_camera_object
× T_object_grasp
```

를 IK의 target으로 사용할 수 있다.

이렇게 범위를 제한하면 object detection, coordinate transform, robot kinematics를 검증하면서도 일반적인 grasp planning이나 physics simulation까지 문제를 확장하지 않는다.

---

## 3. 왜 Synthetic Pose부터 시작하는가

실제 Camera + ArUco부터 시작하면 오차 원인이 한 번에 섞인다.

```text
Camera Noise
Marker Detection Error
solvePnP Error
Coordinate Conversion Error
Network Error
Rendering Error
```

그래서 첫 단계에서는 deterministic한 Synthetic Pose를 Viewer에 직접 적용한다.

```text
Synthetic Pose
→ Transform
→ Flecs Entity
→ RenderSystem
→ OpenGL Cube
```

이 단계는 최종 기능이 아니라 **Pose가 생성된 뒤 Viewer까지 전달되는 가장 작은 수직 경로를 검증하는 기준점**이다.

그 다음 UDP를 연결하면:

```text
Synthetic Pose
→ UDP
→ Viewer
```

로 바뀌므로 network 문제만 추가해서 확인할 수 있다.

---

## 4. 왜 Vision Node와 Simulator를 분리하는가

한 process에서:

```text
Camera
→ Pose
→ Robot
```

를 처리하면 실제 원격 시스템에서 생기는 network boundary가 사라진다.

PoseLink는 의도적으로 두 process를 둔다.

```text
Vision Node
     ↓ UDP
Viewer / Simulator
```

Vision Node는 다음까지만 책임진다.

```text
Camera
→ Detection
→ Object Pose
→ Serialization
→ UDP Publish
```

Viewer / Simulator는 다음을 책임진다.

```text
UDP Receive
→ Object Transform
→ Grasp Target
→ Robot Kinematics
→ Rendering
```

이 분리는 Vision과 Simulation을 독립적으로 테스트할 수 있게 한다.

---

## 5. 왜 물체 Pose와 Grasp Pose를 구분하는가

물체 중심 좌표로 end-effector를 이동한다고 해서 물체를 잡을 수 있는 것은 아니다.

예:

```text
Object Pose
= 물체의 위치와 방향

Grasp Pose
= End Effector가 실제로 도달해야 하는 위치와 방향
```

따라서 PoseLink에서는 object frame 기준 grasp offset을 명시적으로 둔다.

```text
Object Pose
     ↓
Grasp Pose Generator
     ↓
Target End-Effector Pose
```

초기에는 물체 종류별로 하나의 known grasp pose만 둔다. 임의 형상에서 자동 grasp 후보를 생성하는 기능은 기본 범위에서 제외한다.

---

## 6. 왜 FK를 IK보다 먼저 구현하는가

IK는 target pose를 만족하는 joint angle을 찾는 문제다. 하지만 candidate joint angle이 실제로 어떤 end-effector pose를 만드는지 계산하려면 FK가 먼저 필요하다.

```text
Joint Angles
→ FK
→ Current End-Effector Pose
→ Target Pose와 Error 계산
→ IK Update
```

따라서 구현 순서는 다음처럼 고정한다.

```text
Robot Model
→ FK
→ Grasp Pose
→ IK
```

이 순서를 지키면 robot hierarchy/axis/origin 오류와 IK solver 오류를 분리해서 확인할 수 있다.

---

## 7. 왜 Flecs를 Viewer에 사용하는가

최종 Viewer에는 단순 Cube 외에 다음 scene object가 생긴다.

```text
Tracked Object
Robot Base
Robot Links
End Effector
Debug Axis
```

각 object를 깊은 상속 구조로 만들기보다:

```text
Entity
+ Transform
+ Renderable
+ optional role component
```

형태로 구성한다.

현재 `RenderSystem`은 `Transform + Renderable` 조합을 query해 렌더링한다. 향후 robot hierarchy에는 Flecs relationship을 검토할 수 있지만, FK/IK 계산 자체를 Flecs에 종속시킬 필요는 없다.

---

## 8. 왜 OpenGL을 직접 사용하는가

기존 game engine을 사용하면 robot model을 빠르게 화면에 띄울 수 있지만 다음 경계가 가려진다.

```text
Pose
→ Transform
→ Model Matrix
→ Shader
→ Draw
```

PoseLink에서는 OpenGL을 직접 사용해 coordinate system과 transform hierarchy를 코드 수준에서 확인한다.

이는 robot link의 world transform, end-effector pose, grasp target을 시각적으로 검증하는 데도 유용하다.

---

## 9. 왜 network 실험은 마지막에 하는가

Network jitter/loss는 프로젝트의 중요한 검증 항목이지만 robot grasp path가 완성되기 전에 먼저 최적화하면 무엇이 흔들리는지 기준이 없다.

따라서 먼저 ideal/local condition에서:

```text
Object Pose
→ Grasp Pose
→ IK
→ FK
→ Attach
```

가 정상 동작하는지 확인한다.

그 다음 network impairment를 넣어 다음을 비교한다.

```text
Immediate Pose
vs
Buffered / Interpolated Pose
```

측정 대상은 단순 화면 jitter뿐 아니라 최종적으로:

- target pose age
- end-effector target error
- grasp 성공/실패 조건
- queue/buffer 상태

까지 확장할 수 있다.

---

## 10. 기본 범위에서 하지 않는 것

초기 완료 범위에는 다음을 넣지 않는다.

- markerless object detector
- neural network 기반 6DoF estimation
- 일반 물체 grasp planning
- collision-free path planner
- 물리 기반 gripper contact simulation
- robot dynamics / torque control
- ROS2 integration
- multi-camera sensor fusion

초기 grasp는 **kinematic grasp**다. End Effector가 위치/회전 허용 오차 안에 들어오면 object를 end-effector에 attach하여 성공으로 처리한다.

---

## 11. 프로젝트 진행 순서

1. **Synthetic Pose → Cube** — 현재 단계
2. UDP Object Pose
3. ArUco Object Detection
4. Simulation에 Object 생성
5. Robot Model 추가
6. FK 구현
7. Grasp Pose 정의
8. IK 구현
9. End Effector → Grasp Pose 추종
10. Grasp 성공 시 Object attach
11. Network jitter/loss 실험

각 단계는 이전 단계의 correctness를 기준으로 다음 변수를 하나씩 추가한다.

---

## 12. 완료 후 한 문장

> PoseLink는 원격 카메라가 추정한 known object의 6DoF Pose를 UDP로 전달하고, 가상 다관절 로봇팔이 object-relative grasp pose를 IK/FK로 추종해 집는 과정을 시뮬레이션하며, 마지막으로 network jitter/loss가 grasp 안정성에 미치는 영향을 검증하는 C++ 프로젝트다.
