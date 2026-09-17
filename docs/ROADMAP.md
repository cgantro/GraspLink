# GraspLink Simulation Roadmap

## Goal

프로젝트의 최소 완성 경로는 다음과 같다.

```text
Synthetic Target/Object
→ HCR-12A 6DoF Robot Model
→ FK
→ 6D Grasp Pose
→ 6DoF IK
→ Joint Update
→ End Effector Pose Tracking
→ 2F85 Gripper
→ Object Attach / Release
```

외부 장치, 네트워크, Camera/Vision 기능은 로드맵에 포함하지 않는다.

---

## Phase 1 — Simulation Scene Baseline

### 목표
현재 OpenGL/Flecs Viewer 안에서 target object와 reference axis를 안정적으로 렌더링한다.

### 완료 기준
- deterministic synthetic target을 배치할 수 있다.
- object transform과 world axis를 확인할 수 있다.
- frame delta time과 scene update 순서가 고정된다.

---

## Phase 2 — Robot Asset / Kinematic Description

### 목표
HCR-12A 6DoF 로봇을 link 단위로 표현할 수 있는 asset과 joint 정보를 정리한다.

### 작업
- STEP→GLB 변환 시 CAD assembly hierarchy/local transform 보존
- HCR-12A Base / Link1~6 visual group 식별
- J1~J6 origin/axis/limit 정의
- 실제 6축 serial kinematic hierarchy 정의
- End Effector frame 정의
- 2F85 Gripper asset 분리 유지
- `T_ee_gripper` mount offset 정의

### 완료 기준
reference configuration에서 모든 link와 gripper가 의도한 위치에 표시된다.

---

## Phase 3 — Forward Kinematics

### 목표
`q1..q6` joint angle로 모든 link와 End Effector의 world transform을 계산한다.

### 완료 기준
- J1~J6을 하나씩 움직였을 때 올바른 축으로 회전한다.
- child link가 parent transform을 올바르게 누적한다.
- reference configuration을 재현한다.
- End Effector 6D pose를 계산할 수 있다.

---

## Phase 4 — Grasp Pose / Gripper Mount

### 목표
Object frame에서 6D grasp target과 gripper 장착 관계를 명시한다.

```text
T_world_grasp = T_world_object × T_object_grasp
T_world_gripper = T_world_ee × T_ee_gripper
```

### 완료 기준
- Object와 Grasp frame을 debug axis로 확인할 수 있다.
- End Effector에 gripper가 일정한 mount offset으로 따라간다.

---

## Phase 5 — Inverse Kinematics

### 목표
reachable target에 대해 6DoF joint configuration을 계산한다.

```text
Target 6D Grasp Pose
→ IK
→ q1..q6
→ FK(q)
→ End Effector Pose
```

### 완료 기준
position/orientation error가 정한 tolerance 안에 들어온다. unreachable/non-convergent target은 명시적 실패로 처리한다.

---

## Phase 6 — Joint Update / Tracking

### 목표
IK 결과로 즉시 teleport하지 않고 joint state를 시간에 따라 갱신한다.

### 완료 기준
- joint speed/step 제한이 있다.
- frame rate 변화에도 update가 안정적이다.
- target pose 이동 시 End Effector가 position/orientation을 추종한다.

---

## Phase 7 — Grasp / Attach / Release

### 목표
kinematic grasp state machine을 완성한다.

```text
OPEN
→ APPROACHING
→ READY
→ CLOSED / ATTACHED
→ RELEASED
```

### 완료 기준
- position/orientation threshold 밖에서는 attach되지 않는다.
- 조건 만족 + gripper close에서 attach된다.
- attach 후 object가 gripper relative transform을 유지한다.
- release 시 world pose가 비정상적으로 튀지 않는다.

---

## Phase 8 — Verification / Measurement

### 측정 후보
- FK reference error
- IK position/orientation error
- IK convergence / failure count
- grasp success/failure
- target 도달 시간
- frame time / FPS
- solver time

정확한 목표 수치는 baseline을 측정한 뒤 정한다.

---

## 기본 범위에서 제외

- ESP32 / MCU / Zephyr RTOS
- GPIO / ADC / I2C / OLED
- UDP / socket communication
- Camera / ArUco / OpenCV
- network jitter/loss 실험
- ROS2
- 실로봇 제어
- collision-free motion planning
- rigid-body/contact physics
- torque/dynamics simulation
