# Viewer Pose Buffer와 Interpolation

## 1. 이 문서의 위치

PoseBuffer와 interpolation은 프로젝트의 첫 핵심 기능이 아니라 **로드맵 11단계 Network jitter/loss 실험에서 사용하는 robustness 기능**이다.

먼저 다음 ideal path가 완성되어야 한다.

```text
Object Pose
→ Simulation Object
→ Grasp Pose
→ IK
→ FK
→ End Effector Tracking
→ Object Attach
```

그 뒤 network impairment를 넣고 target pose의 시간축을 안정화한다.

---

## 2. 비교할 두 정책

### Immediate

가장 최근 수신한 유효 Object Pose를 바로 simulation target에 적용한다.

장점:
- 추가 buffering latency가 없음

단점:
- arrival interval jitter가 object/grasp target 움직임에 직접 나타남

### Buffered Interpolation

```text
renderTime = now - bufferDelay
```

를 기준으로 해당 시점을 감싸는 Pose A/B를 찾고 보간한다.

```text
A -------- renderTime -------- B
```

장점:
- 불규칙한 packet arrival을 일정한 rendering timeline으로 재구성 가능

단점:
- 의도적으로 latency 추가

---

## 3. 보간 방식

Position:

```text
LERP
p(t) = (1-t)pA + t pB
```

Orientation:

```text
Quaternion SLERP
```

필수 처리:

- input quaternion normalize
- `q`와 `-q`가 같은 회전임을 고려
- shortest path 선택
- output normalize

초기에는 target time이 buffer 범위 밖이면 endpoint hold를 사용하고 prediction/extrapolation은 넣지 않는다.

---

## 4. Robot grasp pipeline에서의 적용 위치

우선 적용 위치는 **수신 Object Pose와 Simulation Object 사이**다.

```text
UDP Receiver
→ PoseBuffer
→ Interpolated Object Pose
→ Simulation Object
→ Grasp Pose
→ IK
```

이렇게 하면 IK solver 자체를 network arrival timing에 직접 노출하지 않는다.

향후 비교 실험에서:

```text
Immediate Object Pose
vs
Buffered Object Pose
```

를 같은 robot pipeline에 넣는다.

---

## 5. `PoseBuffer` 요구사항 — 예정

현재 `modules/streaming/include/PoseBuffer.h`, `.cpp`는 빈 골격이다.

구현 시 요구:

- bounded sample count
- timestamp 기준 정렬 또는 명시적 late packet 정책
- target time을 감싸는 sample pair 조회
- stale sample 제거
- empty / single sample 처리
- duplicate timestamp 정책
- invalid sample 정책

무제한 queue를 허용하지 않는다.

---

## 6. 측정 대상

마지막 실험에서 최소 다음을 비교한다.

### Network

- packet receive interval
- loss
- reorder
- duplicate
- pose age

### Object Target

- immediate/buffered object pose 차이
- ground truth가 있을 때 position/orientation error

### Robot

- grasp target과 end-effector 간 position error
- orientation error
- IK convergence failure 여부
- grasp success 조건 도달 여부

즉 단순히 "화면이 덜 흔들린다"가 아니라 robot target에 미치는 영향을 측정한다.

---

## 7. Buffer Delay 결정

`bufferDelay` 값은 현재 문서에서 임의로 최종값을 정하지 않는다.

절차:

```text
Baseline 측정
→ delay/jitter profile 정의
→ bufferDelay 후보 비교
→ target stability 개선과 추가 pose age를 함께 기록
→ 최종 정책 선택
```

결과는 `experiments.md` 계약에 따라 기록한다.

---

## 8. 제외 범위

초기 robustness 실험에서 제외:

- Kalman Filter
- velocity prediction
- dead reckoning
- adaptive jitter buffer
- clock synchronization protocol

필요성은 기본 interpolation 결과를 측정한 뒤 판단한다.
