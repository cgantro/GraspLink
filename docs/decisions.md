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

### D-02 Robot Arm은 4DoF + Gripper

```text
J1 Base yaw
J2 Shoulder pitch
J3 Elbow pitch
J4 Wrist pitch
Gripper Open/Close
```

Gripper는 4DoF와 별도 state로 관리한다.

### D-03 Robot asset과 Kinematics 분리

GLB node 구조를 joint hierarchy의 source of truth로 사용하지 않는다.

```text
RobotDescription
→ joint origin / axis / limit / parent-child

Mesh Asset
→ rendering geometry
```

### D-04 Robot GLB와 Gripper GLB는 분리 유지 가능

그리퍼는 End Effector에 고정 mount transform으로 연결한다.

```text
T_world_gripper = T_world_ee × T_ee_gripper
```

### D-05 FK를 IK보다 먼저 구현

Robot hierarchy와 joint frame을 검증한 뒤 IK를 구현한다.

### D-06 Object Pose와 Grasp Pose를 분리

```text
T_world_grasp = T_world_object × T_object_grasp
```

Object 중심과 End Effector target은 같다고 가정하지 않는다.

### D-07 초기 grasp는 kinematic attach

물리 접촉 simulation은 기본 범위에서 제외한다. 위치/정렬 오차와 gripper state가 조건을 만족할 때 Object Attach로 파지를 표현한다.

### D-08 Flecs는 Scene/Rendering에 사용

Flecs는 entity/component/system 관리에 사용한다. FK/IK solver를 ECS에 강하게 결합하지 않는다.

### D-09 입력은 Deterministic Synthetic State

외부 입력 대신 Simulator 내부에서 target/object trajectory를 생성한다. 같은 조건에서 같은 결과를 재현할 수 있어야 한다.

### D-10 성능 수치는 Baseline 이후 결정

구현 전 임의 숫자를 성과 목표처럼 두지 않는다.

---

## 2. 현재 로드맵

1. Synthetic Target/Object
2. 4DoF Robot Asset / Description
3. FK
4. Gripper Mount / Grasp Pose
5. IK
6. Joint Tracking
7. Kinematic Grasp
8. Object Attach / Release
9. Verification / Measurement

---

## 3. 미결정 사항

### Robot Model 단계

- CAD/GLB를 link 단위로 어떻게 전처리할지
- RobotDescription을 C++ 상수/config 중 어디에 둘지
- 실제 joint origin/axis/limit 기준 자료

### IK 단계

- analytic vs numerical solver
- numerical 사용 시 Jacobian/DLS 세부 방식
- convergence threshold
- joint limit 처리

### Grasp 단계

- `T_object_grasp`
- `T_ee_gripper`
- position/alignment threshold
- attach 이후 transform ownership

### Verification 단계

- baseline trajectory
- 반복 횟수
- solver/frame timing 측정 방식

---

## 4. 기본 완료 범위

```text
Synthetic Object
→ Grasp Pose
→ 4DoF IK
→ Joint Tracking
→ FK
→ Robot / Gripper Visualization
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
- Asset 형식과 robot joint definition을 분리한다.
- FK correctness를 IK보다 먼저 고정한다.
- synthetic input으로 재현 가능한 테스트를 만든다.
- 구현 상태와 예정 상태를 문서에서 구분한다.
