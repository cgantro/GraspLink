# GraspLink Acceptance Criteria

## 1. 목적

각 시뮬레이션 단계가 어느 상태에서 완료된 것으로 판단되는지 정의한다. 임의의 성능 수치를 먼저 정하지 않고 correctness를 확보한 뒤 baseline을 측정한다.

---

## 2. 공통 원칙

- 구현 상태와 문서가 일치한다.
- 같은 입력에서 재현 가능한 결과가 나온다.
- 좌표계와 단위가 명시되어 있다.
- NaN/Inf와 invalid state를 scene에 적용하지 않는다.
- kinematics와 rendering의 책임이 분리되어 있다.
- CAD assembly hierarchy와 kinematic hierarchy를 구분한다.
- Release/Debug 측정 결과를 섞지 않는다.

---

## 3. 단계별 완료 기준

### 1. Synthetic Target/Object

- Simulator 내부에서 target/object transform을 생성할 수 있다.
- 같은 seed/time input에서 같은 trajectory를 재현할 수 있다.
- object transform을 debug axis로 확인할 수 있다.

### 2. Robot Model

- HCR-12A Base, Link1~6, End Effector가 식별된다.
- J1~J6의 origin/axis/limit가 정의된다.
- STEP→GLB 변환 asset에서 assembly hierarchy와 local transform이 보존된다.
- CAD assembly tree와 6축 kinematic description을 분리한다.
- reference configuration에서 모든 link가 의도한 위치에 렌더링된다.

### 3. FK

- `q1..q6` 입력으로 각 link world transform을 계산한다.
- J1~J6을 하나만 움직였을 때 해당 joint axis를 기준으로 회전한다.
- child link가 parent transform을 누적한다.
- End Effector의 position/orientation pose를 계산할 수 있다.
- reference configuration을 deterministic test로 검증한다.

### 4. Gripper Mount / Grasp Pose

- `T_ee_gripper`가 명시되어 있다.
- `T_object_grasp`가 명시되어 있다.
- 2F85 gripper가 End Effector를 일정한 offset으로 따라간다.
- object가 움직이면 6D grasp frame이 동일한 object-relative offset을 유지한다.

### 5. IK

- reachable 6D target pose에서 `q1..q6` joint configuration을 찾는다.
- `IK → FK` 재계산 결과의 position/orientation error가 tolerance 안에 들어온다.
- unreachable/non-convergent target을 명시적 실패로 반환한다.
- joint limit 정책이 존재한다.

### 6. Joint Tracking

- IK 결과를 frame 단위 joint state에 반영한다.
- joint speed/step 제한을 적용한다.
- `dt` 변화에도 비정상 점프가 없다.
- target이 reachable한 동안 End Effector가 target pose를 추종한다.

### 7. Grasp / Attach / Release

- position/orientation alignment error를 계산한다.
- 성공 threshold가 명시되어 있다.
- threshold와 gripper close 조건을 만족할 때만 attach한다.
- attach 후 object가 gripper-relative transform을 유지한다.
- release 시 object world pose가 불필요하게 튀지 않는다.

### 8. Runtime Verification

- solver time과 frame time을 측정할 수 있다.
- 반복 실행에서 state가 무제한 증가하지 않는다.
- 동일 trajectory에서 IK failure/grasp result를 재현할 수 있다.

---

## 4. 비기능 요구사항

### Ownership / Lifetime

- Window/OpenGL context보다 GPU resource가 먼저 파괴된다.
- resource owner가 명확하다.
- 종료 시 dangling task/thread를 남기지 않는다.

### Dependency Boundary

금지:

```text
kinematics → OpenGL
kinematics → Flecs scene lifecycle
renderer → IK solver internals
```

### Observability

최소한 다음 상태를 확인할 수 있어야 한다.

- current joint state `q1..q6`
- End Effector 6D pose
- target/grasp 6D pose
- IK success/failure
- position/orientation error
- gripper/grasp state
- frame/solver timing

---

## 5. 수치 목표 설정 방법

```text
Correctness 구현
→ Baseline 측정
→ 병목/오차 원인 확인
→ 만족 기준 설정
→ 같은 환경에서 재측정
```

측정 후보:

- FK reference error
- IK position error
- IK orientation error
- IK solve time
- target 도달 시간
- Viewer frame time
- grasp success/failure

---

## 6. 프로젝트 완료 정의

다음 경로가 하나의 deterministic simulation으로 동작하고 단계별 검증 근거가 남아 있을 때 완료로 본다.

```text
Synthetic Object
→ 6D Grasp Pose
→ 6DoF IK
→ Joint Update q1..q6
→ FK
→ HCR-12A / 2F85 Visualization
→ Kinematic Grasp
→ Object Attach / Release
```

외부 Embedded/Network/Vision 기능은 완료 정의에 포함하지 않는다.
