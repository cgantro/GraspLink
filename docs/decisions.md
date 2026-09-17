# 설계 결정과 범위

## 1. 확정 결정

### D-01 프로젝트는 Simulation-only로 유지

GraspLink는 단일 C++ Simulator process로 제한한다.

기본 범위에서 제외:

```text
Embedded Controller
UDP / Network
Camera / Vision
External Robot
```

### D-02 Robot Arm은 HCR-12A 기반 6DoF + 2F85 Gripper

```text
Base
→ J1
→ J2
→ J3
→ J4
→ J5
→ J6
→ End Effector
→ 2F85 Gripper
```

J1~J6은 revolute joint로 관리하고 Gripper open/close는 별도 actuator/state로 관리한다.

### D-03 Robot asset과 Kinematics 분리

STEP→GLB 변환에서는 CAD assembly hierarchy와 local transform을 보존한다. 다만 GLB/CAD node tree를 6축 kinematic hierarchy의 source of truth로 사용하지 않는다.

```text
RobotDescription
→ J1~J6 origin / axis / limit / parent-child

Mesh Asset
→ rendering geometry / CAD assembly
```

### D-04 Robot GLB와 Gripper GLB는 분리 유지

그리퍼는 End Effector에 고정 mount transform으로 연결한다.

```text
T_world_gripper = T_world_ee × T_ee_gripper
```

### D-05 FK를 IK보다 먼저 구현

6축 Robot hierarchy와 joint frame을 검증한 뒤 IK를 구현한다.

### D-06 IK target은 6D End Effector Pose

IK target은 XYZ position과 orientation을 포함한다.

```text
Target 6D Pose
→ IK
→ q1..q6
→ FK
→ Position / Orientation Error
```

### D-07 Object Pose와 Grasp Pose를 분리

```text
T_world_grasp = T_world_object × T_object_grasp
```

Object 중심과 End Effector target은 같다고 가정하지 않는다.

### D-08 초기 grasp는 kinematic attach

물리 접촉 simulation은 기본 범위에서 제외한다. 위치/정렬 오차와 gripper state가 조건을 만족할 때 Object Attach로 파지를 표현한다.

### D-09 Flecs는 Scene/Rendering에 사용

Flecs는 entity/component/system 관리에 사용한다. FK/IK solver를 ECS에 강하게 결합하지 않는다.

### D-10 입력은 Deterministic Synthetic State

외부 입력 대신 Simulator 내부에서 target/object trajectory를 생성한다. 같은 조건에서 같은 결과를 재현할 수 있어야 한다.

### D-11 성능 수치는 Baseline 이후 결정

구현 전 임의 숫자를 성과 목표처럼 두지 않는다.

---

## 2. 현재 로드맵

1. Synthetic Target/Object
2. HCR-12A / 2F85 Asset 정리
3. 6DoF RobotDescription
4. FK
5. Gripper Mount / 6D Grasp Pose
6. 6DoF IK
7. Joint Tracking
8. Kinematic Grasp
9. Object Attach / Release
10. Verification / Measurement

---

## 3. 미결정 사항

### Robot Model 단계

- HCR-12A CAD assembly의 각 visual group을 Link1~6에 어떻게 매핑할지
- RobotDescription을 C++ 상수/config 중 어디에 둘지
- J1~J6 origin/axis/limit 기준값
- HCR-12A Tool frame과 2F85 mount offset

### IK 단계

- analytic vs numerical solver
- numerical 사용 시 Jacobian/DLS 세부 방식
- position/orientation error weight
- convergence threshold
- joint limit 처리

### Grasp 단계

- `T_object_grasp`
- `T_ee_gripper`
- position/orientation threshold
- attach 이후 transform ownership

### Verification 단계

- baseline trajectory
- 반복 횟수
- solver/frame timing 측정 방식

---

## 4. 기본 완료 범위

```text
Synthetic Object
→ 6D Grasp Pose
→ 6DoF IK
→ Joint Tracking q1..q6
→ FK
→ HCR-12A / 2F85 Visualization
→ Kinematic Grasp
→ Object Attach / Release
→ Deterministic Verification
```

---

## 5. 기본 범위에서 제외

- ESP32 / MCU / RTOS
- GPIO / ADC / I2C
- UDP / network protocol
- Camera / OpenCV / ArUco
- markerless object detection
- ROS2
- 실로봇 제어
- collision-free motion planning
- rigid-body/contact physics
- robot dynamics / torque control

---

## 6. 설계 원칙

- 현재 필요한 책임만 구현한다.
- Kinematics와 rendering을 분리한다.
- CAD assembly hierarchy와 6DoF kinematic hierarchy를 분리한다.
- Asset 형식과 robot joint definition을 분리한다.
- FK correctness를 IK보다 먼저 고정한다.
- synthetic input으로 재현 가능한 테스트를 만든다.
- 구현 상태와 예정 상태를 문서에서 구분한다.
