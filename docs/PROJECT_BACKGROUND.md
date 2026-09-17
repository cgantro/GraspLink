# GraspLink Project Background

## 1. 프로젝트가 해결하려는 문제

GraspLink의 목표는 **가상 환경에서 HCR-12A 기반 6DoF 로봇팔이 목표 물체에 접근하고 파지하는 전체 과정을 C++로 직접 구현하고 검증하는 것**이다.

```text
Synthetic Object
→ 6D Grasp Pose
→ 6DoF IK
→ Joint State q1..q6
→ FK
→ Robot Link Transform
→ End Effector
→ 2F85 Gripper
→ Object Attach
```

프로젝트 범위를 Simulator 내부로 제한해 외부 센서나 네트워크 문제와 로봇 kinematics 문제를 섞지 않는다.

---

## 2. 왜 Simulation-only로 제한하는가

이 프로젝트의 핵심 학습 대상은 다음이다.

- robot coordinate frame
- 6-axis joint hierarchy
- forward kinematics
- inverse kinematics
- end-effector 6D pose
- gripper mount
- grasp state transition
- real-time rendering/update loop

외부 MCU, RTOS, Camera, UDP까지 동시에 포함하면 오류 원인이 너무 넓어진다. 따라서 입력은 deterministic synthetic state로 고정하고 로봇 시뮬레이션 자체의 correctness를 먼저 완성한다.

---

## 3. 왜 6DoF인가

사용할 HCR-12A 모델은 J1~J6으로 구성된 6축 serial manipulator다. 프로젝트의 kinematics도 실제 모델 구조에 맞춰 6DoF로 정의한다.

```text
Base
→ J1
→ J2
→ J3
→ J4
→ J5
→ J6
→ End Effector
→ Gripper
```

Gripper open/close는 6개 robot joint와 별도 state다.

6DoF를 사용하면 End Effector의 XYZ 위치뿐 아니라 orientation까지 포함한 grasp pose를 다룰 수 있다. IK는 position과 orientation을 함께 만족하는 joint configuration을 구하는 문제로 정의한다.

---

## 4. 왜 FK를 IK보다 먼저 구현하는가

IK 결과를 검증하려면 joint angle에서 실제 End Effector pose를 계산할 기준이 먼저 필요하다.

```text
q1..q6
→ FK
→ End Effector 6D Pose
```

그 다음:

```text
Target 6D Grasp Pose
→ IK
→ q1..q6
→ FK
→ Position / Orientation Error 확인
```

순서로 solver를 검증한다.

---

## 5. Robot asset과 운동학을 왜 분리하는가

STEP→GLB 변환 시 CAD assembly hierarchy와 local transform은 보존한다. 하지만 CAD 조립 트리와 robot kinematic chain은 목적이 다르므로 동일한 구조라고 가정하지 않는다.

```text
Rendering Asset
≠
Robot Kinematic Description
```

RobotDescription은 J1~J6 origin/axis/limit과 parent-child link 관계를 별도로 가진다. GLB는 계산된 link transform을 받아 그리는 자산으로 사용한다.

---

## 6. Gripper를 별도 asset으로 두는 이유

로봇 본체 HCR-12A GLB와 2F85 Gripper GLB는 분리한다.

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
End Effector / Gripper가 6D Grasp Pose tolerance에 진입
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
2. HCR-12A 6DoF Robot Model
3. FK
4. End Effector / 6D Grasp Pose
5. 2F85 Gripper Mount
6. 6DoF IK
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

> GraspLink는 C++/OpenGL 기반 가상 환경에서 HCR-12A 6DoF 로봇팔의 FK/IK와 2F85 gripper transform을 직접 구현하고, 6D grasp pose 추종부터 kinematic grasp와 attach까지 검증하는 로봇 시뮬레이션 프로젝트다.
