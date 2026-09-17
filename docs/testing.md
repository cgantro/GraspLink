# Testing and Verification

> v1 automated test는 graphics 없이 protocol/sequence, HCR-12A nominal FK/DLS IK와 grasp
> ownership을 검증한다. ArUco/camera/network impairment test는 현재 범위에서 제외한다.

## 1. 목적

GraspLink는 시뮬레이션 단계별로 오류 원인을 분리해 검증한다. 외부 장치/네트워크/Vision은 테스트 범위에 포함하지 않는다.

---

## 2. 전체 검증 순서

```text
Synthetic Target/Object
→ Robot Model
→ FK
→ Gripper Mount / Grasp Pose
→ IK
→ Joint Tracking
→ Attach / Release
→ Runtime Measurement
```

---

## 3. Synthetic Target/Object

- X/Y/Z 이동이 예상 축으로 적용되는지 확인한다.
- 같은 시간/seed에서 같은 target을 생성한다.
- reference axis를 함께 그려 frame 방향을 확인한다.

---

## 4. Robot Model

체크리스트:

- Base / Link1~4 식별
- J1~J4 origin 확인
- joint axis 확인
- joint limit 확인
- End Effector frame 확인
- GLB mesh scale/unit 확인
- Gripper mount frame 확인

각 link에 debug axis를 표시해 hierarchy 오류를 확인한다.

---

## 5. FK

### Reference Configuration

고정 joint angle set을 사용해 예상 link/EE transform과 비교한다.

### Single-Joint Test

```text
J1 only
J2 only
J3 only
J4 only
```

확인:

- 올바른 축으로 회전하는가
- child link가 함께 움직이는가
- parent link가 역으로 움직이지 않는가

### Invariant

- NaN/Inf 없음
- rigid transform에 의도하지 않은 scale/shear 없음
- quaternion/rotation이 유효함

---

## 6. Gripper Mount / Grasp Pose

검증 식:

```text
T_world_gripper = T_world_ee × T_ee_gripper
T_world_grasp   = T_world_object × T_object_grasp
```

- End Effector 이동 시 gripper offset이 일정하다.
- Object 이동 시 grasp frame offset이 일정하다.
- 두 frame을 debug axis로 동시에 확인할 수 있다.

---

## 7. IK

### Reachable Target

```text
Target
→ IK
→ q
→ FK(q)
→ End Effector
```

FK 결과가 설정한 tolerance 안에 들어오는지 확인한다.

### Unreachable Target

workspace 밖 target에서 solver가 실패 상태를 반환하고 scene에 invalid joint 값을 적용하지 않는지 확인한다.

### Numerical Solver를 사용할 경우

- iteration limit
- damping
- convergence threshold
- singularity 근처 동작
- joint limit 처리

를 명시한다.

---

## 8. Joint Tracking

- frame `dt` 변화에 따른 update를 확인한다.
- joint speed/step limit를 초과하지 않는다.
- 큰 `dt` 입력에서 비정상 점프를 방지한다.
- 움직이는 synthetic target을 안정적으로 추종한다.

---

## 9. Grasp / Attach / Release

성공 조건 예:

```text
position error <= threshold
AND
alignment error <= threshold
AND
gripper == close
```

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
- CAD 변환 asset의 node hierarchy를 kinematics 정보로 암묵적으로 사용하지 않는다.

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
- FK update time
- IK solve time
- IK iteration count
- End Effector error
- grasp success/failure
- target 도달 시간

측정 조건과 입력 trajectory를 함께 기록한다.
