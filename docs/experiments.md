# 실험 계획

## 1. 목적

GraspLink의 실험은 외부 network 상태가 아니라 **로봇 kinematics, tracking, grasp 조건, runtime 성능**을 검증한다.

기본 경로:

```text
Synthetic Object
→ Grasp Pose
→ IK
→ Joint Update
→ FK
→ End Effector
→ Grasp / Attach
```

---

## 2. 실험 전제

입력은 deterministic synthetic trajectory를 사용한다.

이유:

```text
Known Ground Truth
→ 같은 입력 반복 가능
→ IK / Tracking / Grasp 영향 분리
```

각 실험은 seed, trajectory, build type, runtime을 기록한다.

---

## 3. FK 검증

비교 항목:

- reference configuration
- single-joint rotation
- multi-joint configuration
- End Effector world pose

측정 후보:

- position error
- orientation error
- invalid transform count

---

## 4. IK 실험

독립 변수 후보:

| 변수 | 의미 |
|---|---|
| target position | workspace 내 위치 |
| wrist pitch | J4 목표 |
| initial joint state | solver 시작점 |
| iteration limit | 최대 반복 횟수 |
| damping | numerical IK 안정화 계수 |
| convergence threshold | 종료 조건 |

측정:

- convergence success/failure
- iteration count
- solve time
- `IK → FK` End Effector error
- joint limit violation 여부

---

## 5. Tracking 실험

Synthetic object trajectory 예:

- static target
- straight line
- circle/arc
- direction change
- reachable/unreachable boundary

조절 변수:

- joint speed limit
- joint step limit
- frame `dt`
- target movement speed

측정:

- End Effector position error
- tracking error variation
- target 도달 시간
- IK failure count

---

## 6. Grasp 실험

조절 변수:

- position threshold
- alignment threshold
- gripper close timing
- object motion speed

측정:

- grasp success/failure
- success condition 도달 시간
- attach 시 object pose discontinuity
- attach 후 relative transform drift

---

## 7. Runtime 성능

Release build에서 기록:

- frame time / FPS
- FK update time
- IK solve time
- scene entity count
- 반복 실행 시 memory 증가 여부

렌더링 시간과 solver 시간을 가능한 한 분리해 측정한다.

---

## 8. 비교 원칙

한 번에 여러 변수를 동시에 바꾸지 않는다.

```text
Baseline
→ 변수 하나 변경
→ 동일 trajectory 반복
→ 결과 비교
```

시각적으로 부드러워 보인다는 평가만 사용하지 않고 수치와 trace를 함께 기록한다.

---

## 9. 기록해야 할 환경

```text
OS
CPU
GPU
Compiler
Build Type
Robot Model
Trajectory
Initial Joint State
IK Parameters
Random Seed
Run Duration
```

---

## 10. 완료 시 답할 수 있어야 하는 질문

1. FK가 정의한 joint hierarchy를 정확히 반영하는가
2. IK가 reachable target에서 어느 오차로 수렴하는가
3. unreachable/singularity 근처에서 어떻게 실패하는가
4. joint speed 제한이 tracking error에 어떤 영향을 주는가
5. grasp threshold가 성공 시점과 안정성에 어떤 영향을 주는가
6. 현재 frame time에서 가장 큰 비용은 rendering인지 solver인지
