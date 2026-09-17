# GraspLink Project Background

## 1. 프로젝트가 해결하려는 문제

GraspLink의 목표는 **가상 환경에서 4DoF 로봇팔이 목표 물체에 접근하고 파지하는 전체 과정을 C++로 직접 구현하고 검증하는 것**이다.

```text
Synthetic Object
→ Grasp Pose
→ IK
→ Joint State
→ FK
→ Robot Link Transform
→ End Effector
→ Gripper
→ Object Attach
```

프로젝트 범위를 Simulator 내부로 제한해 외부 센서나 네트워크 문제와 로봇 kinematics 문제를 섞지 않는다.

---

## 2. 왜 Simulation-only로 제한하는가

이 프로젝트의 핵심 학습 대상은 다음이다.

- robot coordinate frame
- joint hierarchy
- forward kinematics
- inverse kinematics
- end-effector transform
- gripper mount
- grasp state transition
- real-time rendering/update loop

외부 MCU, RTOS, Camera, UDP까지 동시에 포함하면 오류 원인이 너무 넓어진다. 따라서 입력은 deterministic synthetic state로 고정하고 로봇 시뮬레이션 자체의 correctness를 먼저 완성한다.

---

## 3. 왜 4DoF인가

초기 범위는 다음 serial chain으로 고정한다.

```text
J1 Base yaw
J2 Shoulder pitch
J3 Elbow pitch
J4 Wrist pitch
```

Gripper open/close는 별도 state다.

이 구조는 full 6DoF industrial manipulator보다 범위가 작아 FK/IK를 직접 구현하고 시각적으로 검증하기 적절하다. 초기 IK는 XYZ position과 wrist pitch를 대상으로 한다.

---

## 4. 왜 FK를 IK보다 먼저 구현하는가

IK 결과를 검증하려면 joint angle에서 실제 End Effector pose를 계산할 기준이 먼저 필요하다.

```text
Joint Angles
→ FK
→ End Effector Pose
```

그 다음:

```text
Target Grasp Pose
→ IK
→ Joint Angles
→ FK
→ Error 확인
```

순서로 solver를 검증한다.

---

## 5. Robot asset과 운동학을 왜 분리하는가

CAD→GLB 변환 파일은 시각적으로는 정상이어도 joint hierarchy, pivot, parent-child 관계가 사라질 수 있다. 따라서 mesh node 구조를 kinematics의 source of truth로 사용하지 않는다.

```text
Rendering Asset
≠
Robot Kinematic Description
```

RobotDescription은 joint origin/axis/limit과 link 관계를 별도로 가진다. GLB는 계산된 link transform을 받아 그리는 자산으로 사용한다.

---

## 6. Gripper를 별도 asset으로 두는 이유

로봇 본체 GLB와 gripper GLB는 합칠 필요가 없다.

```text
T_world_gripper
=
T_world_ee × T_ee_gripper
```

고정 mount offset만 정의하면 End Effector에 독립 asset을 연결할 수 있다. 이후 jaw animation을 추가하더라도 robot arm FK와 분리해서 관리할 수 있다.

---

## 7. Grasp 범위

초기 grasp는 physics/contact simulation이 아니라 kinematic condition으로 판정한다.

```text
End Effector / Gripper가 Grasp Pose tolerance에 진입
AND
Gripper Close
→ Grasp Success
→ Object Attach
```

충돌력, 마찰, dynamics는 기본 범위에 포함하지 않는다.

---

## 8. 왜 OpenGL/Flecs를 유지하는가

OpenGL을 사용해 link, target, gripper, frame axis를 직접 시각화한다.

```text
Kinematics Result
→ Transform
→ Model Matrix
→ Renderer
```

Flecs는 scene entity/component와 rendering system scheduling에 사용한다. FK/IK 계산은 rendering layer와 분리한다.

---

## 9. 프로젝트 완료 범위

1. Synthetic Target/Object
2. 4DoF Robot Model
3. FK
4. End Effector / Grasp Pose
5. Gripper Mount
6. IK
7. Joint Update / Tracking
8. Kinematic Grasp
9. Object Attach / Release
10. correctness 및 runtime measurement

---

## 10. 기본 범위에서 하지 않는 것

- MCU / RTOS / Embedded firmware
- UDP / socket communication
- Camera / ArUco / markerless detection
- ROS2
- 실로봇 제어
- collision-free motion planning
- rigid-body/contact physics
- torque/dynamics control

---

## 11. 프로젝트 한 문장

> GraspLink는 C++/OpenGL 기반 가상 환경에서 4DoF 로봇팔의 FK/IK와 gripper transform을 직접 구현하고, 목표 물체 추종부터 kinematic grasp와 attach까지 검증하는 로봇 시뮬레이션 프로젝트다.
