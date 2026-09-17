# Testing and Verification

> Automated tests are limited to simulation-domain behavior: HCR-12A nominal FK/Jacobian/DLS IK and grasp ownership. External transport/protocol/device tests are not part of the project.

## 1. 목적

GraspLink는 시뮬레이션 단계별로 오류 원인을 분리해 검증한다.

---

## 2. 전체 검증 순서

```text
Synthetic Target/Object
→ HCR-12A Robot Model
→ 6DoF FK
→ 2F85 Gripper Mount / 6D Grasp Pose
→ 6DoF IK
→ Joint Tracking
→ Attach / Release
→ Runtime Measurement
```

---

## 3. Synthetic Target/Object

- X/Y/Z 이동이 예상 축으로 적용되는지 확인한다.
- orientation 변화가 예상 회전축으로 적용되는지 확인한다.
- 같은 입력에서 같은 target pose를 재현한다.
- reference axis를 함께 그려 frame 방향을 확인한다.

---

## 4. Robot Model

체크리스트:

- HCR-12A Base / Link1~6 식별
- J1~J6 origin 확인
- J1~J6 joint axis 확인
- J1~J6 joint limit 확인
- End Effector frame 확인
- GLB mesh scale/unit 확인
- CAD assembly hierarchy/local transform 보존 확인
- CAD assembly tree와 kinematic tree 분리 확인
- 2F85 Gripper mount frame 확인

---

## 5. FK

### Reference Configuration

고정 `q1..q6` angle set을 사용해 예상 link/EE transform과 비교한다.

### Single-Joint Test

```text
J1 only
J2 only
J3 only
J4 only
J5 only
J6 only
```

확인:

- 올바른 축으로 회전하는가
- child link가 함께 움직이는가
- parent link가 역으로 움직이지 않는가
- J1~J6 누적 transform 후 End Effector pose가 일관적인가

### Invariant

- NaN/Inf 없음
- rigid transform에 의도하지 않은 scale/shear 없음
- quaternion/rotation이 유효함

---

## 6. Gripper Mount / Grasp Pose

```text
T_world_gripper = T_world_ee × T_ee_gripper
T_world_grasp   = T_world_object × T_object_grasp
```

- End Effector 이동 시 2F85 gripper offset이 일정하다.
- Object 이동/회전 시 grasp frame offset이 일정하다.
- 두 frame을 debug axis로 동시에 확인할 수 있다.

---

## 7. IK

### Reachable Target

```text
Target 6D Pose
→ IK
→ q1..q6
→ FK(q)
→ End Effector 6D Pose
```

FK 결과의 position/orientation error가 설정한 tolerance 안에 들어오는지 확인한다.

### Unreachable Target

workspace 밖 target 또는 수렴 불가능한 orientation에서 solver가 실패 상태를 반환하고 scene에 invalid joint 값을 적용하지 않는지 확인한다.

### Numerical Solver 검증

- iteration limit
- damping
- convergence threshold
- position/orientation error weight
- singularity 근처 동작
- joint limit 처리

을 명시한다.

---

## 8. Joint Tracking

- frame `dt` 변화에 따른 update를 확인한다.
- 각 joint의 speed/step limit를 초과하지 않는다.
- 큰 `dt` 입력에서 비정상 점프를 방지한다.
- 움직이는 synthetic 6D target pose를 안정적으로 추종한다.

---

## 9. Grasp / Attach / Release

검증:

- threshold 밖에서는 attach되지 않는다.
- 조건 만족 시 한 번만 attach transition이 발생한다.
- attach 순간 object world pose가 튀지 않는다.
- attach 후 gripper-relative transform을 유지한다.
- release 후 world ownership이 정상 복귀한다.

---

## 10. Asset Loader

GLB/mesh loader에 대해 다음을 확인한다.

- 잘못된 header/chunk를 명시적으로 reject한다.
- unsupported vertex/index layout을 구분한다.
- mesh bounds와 scale을 확인할 수 있다.
- STEP→GLB 변환 asset의 assembly hierarchy와 local transform을 읽을 수 있다.
- CAD node hierarchy를 kinematics 정보로 암묵적으로 사용하지 않는다.

---

## 11. Memory / Lifetime

반복 실행에서 확인:

- scene/state 무제한 증가 없음
- GPU resource가 context보다 먼저 파괴됨
- 종료 시 dangling thread/task 없음

가능하면 AddressSanitizer/UndefinedBehaviorSanitizer를 사용한다.

---

## 12. Runtime Measurement

Release build에서 기록 후보:

- Viewer frame time / FPS
- 6DoF FK update time
- IK solve time
- IK iteration count
- End Effector position error
- End Effector orientation error
- grasp success/failure
- target 도달 시간

측정 조건과 입력 trajectory를 함께 기록한다.
