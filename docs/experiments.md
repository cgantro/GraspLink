# 실험 계획

## 1. 목적

Network experiment는 프로젝트 로드맵의 마지막 단계다.

먼저 ideal/local condition에서 다음 경로가 정상 동작해야 한다.

```text
Object Pose
→ Simulation Object
→ Grasp Pose
→ IK
→ FK
→ End Effector Tracking
→ Object Attach
```

그 뒤 delay/jitter/loss/reorder를 추가하여 **network impairment가 object target과 grasp 성능에 어떤 영향을 주는지** 측정한다.

---

## 2. 실험 전제

정량 network 실험은 우선 `SyntheticPoseSource`를 사용한다.

이유:

```text
Synthetic Ground Truth
→ Vision noise 제거
→ Network / Buffer / IK 영향 분리
```

실제 ArUco 입력 실험은 별도로 수행하고 결과를 섞지 않는다.

---

## 3. 비교 정책

### A. Ideal / Local Baseline

network impairment 없음.

목적:
- robot model/FK/IK/grasp 자체의 baseline 확보

### B. Immediate Pose

network impairment가 있지만 최신 수신 Pose를 즉시 적용.

### C. Buffered Interpolation

동일 impairment trace에서 timestamp buffer와 LERP/SLERP 적용.

A/B/C를 같은 trajectory와 seed로 비교한다.

---

## 4. 독립 변수

구체적인 최종 숫자는 baseline 측정 전에 성과 기준으로 고정하지 않는다.

조절 변수:

| 변수 | 의미 |
|---|---|
| pose rate | Object Pose 송신 빈도 |
| one-way delay | 기본 network delay |
| jitter | delay 변동 |
| packet loss | packet 유실 비율 |
| reorder | 도착 순서 변경 |
| buffer delay | buffered policy의 의도적 지연 |
| trajectory | 정지/직선/곡선/방향전환 등 synthetic motion |

실험 profile은 한 번에 모든 조합을 수행하지 않고 baseline에서 한 변수씩 변화시키는 방식을 우선한다.

---

## 5. 측정 지표

### Sender / Network

- generated pose count
- sent packet count
- receive packet count
- receive interval
- loss/reorder/duplicate count
- packet age를 측정할 수 있는 조건이면 pose age

### Object Target

Synthetic ground truth가 있을 때:

```text
position error
=
||p_ground_truth - p_object||
```

rotation error는 quaternion angular distance로 계산한다.

### Robot

- IK convergence 여부
- IK iteration count 또는 solve time
- end-effector position error
- end-effector orientation error
- grasp condition 도달 시간
- grasp success/failure

### System

- Viewer frame time/FPS
- PoseBuffer occupancy
- queue drop count
- stale/underrun count
- memory가 장시간 단조 증가하는지 여부

---

## 6. Motion Stability

"덜 흔들린다"는 육안 평가만 사용하지 않는다.

후보 metric:

- ground truth trajectory 대비 object position error
- frame-to-frame object velocity variation
- end-effector target error variation
- grasp target에 진입/이탈하는 횟수

어떤 metric을 최종 사용했는지는 결과 문서에 식과 함께 고정한다.

---

## 7. Network impairment 도구

### Linux

`tc netem` 사용 후보.

적용 전 현재 qdisc/interface를 확인하고 실험 후 원복한다.

### Windows

필요하면 UDP proxy tool을 구현한다.

목표 기능:

- delay
- jitter
- loss
- reorder
- deterministic seed

현재 `tools/network` 아래 구현은 완료된 것으로 간주하지 않는다.

---

## 8. Clock 주의사항

동일 머신의 별도 process에서는 monotonic timestamp 비교를 쉽게 검증할 수 있다.

서로 다른 PC의 `std::chrono::steady_clock` 값은 epoch가 같다고 가정할 수 없다.

따라서 clock synchronization 또는 offset estimation 없이 sender timestamp를 이용해 원격 E2E latency를 주장하지 않는다.

대신 가능한 지표:

- receiver-side arrival interval
- sequence-based loss/reorder
- simulation object stability
- end-effector error

---

## 9. 기록해야 할 환경

모든 정량 결과에는 최소 다음을 함께 기록한다.

```text
OS
CPU
GPU
Compiler
Build Type
Camera (실제 Vision 실험 시)
Camera Resolution/FPS
Robot Model
Pose Rate
Network Profile
Buffer Policy
Random Seed
Run Duration
```

Release build 여부를 명시한다.

---

## 10. 결과 해석 원칙

Buffered Interpolation이 안정성을 높여도 pose age가 과도하게 증가하면 무조건 좋은 결과가 아니다.

최종 비교는 항상:

```text
Target Stability
vs
Added Latency
```

을 함께 본다.

IK가 실패했을 때도 network 문제인지, unreachable target인지, solver 문제인지 구분해서 기록한다.

---

## 11. 실험 완료 조건

최종 프로젝트에서 다음 질문에 수치와 trace를 근거로 답할 수 있어야 한다.

1. ideal condition에서 robot이 grasp target에 도달하는가
2. network jitter/loss가 object target과 end-effector error를 어떻게 변화시키는가
3. buffering/interpolation이 어떤 error를 줄이고 얼마나 지연을 추가하는가
4. grasp 성공 조건이 network 상태에 따라 어떻게 달라지는가
5. 현재 구조에서 가장 큰 한계는 무엇인가

이 문서는 결과가 아니라 실험 계약이다.
