# PoseLink Acceptance Criteria

## 1. 목적

이 문서는 각 로드맵 단계가 **어느 상태에서 완료된 것으로 판단되는지**를 정의한다.

성능 수치는 실제 baseline을 측정하기 전에는 임의의 성과값으로 확정하지 않는다. 구현 후 측정 가능한 항목과 합격 조건을 먼저 고정하고, 실제 환경에서 얻은 수치로 target을 갱신한다.

---

## 2. 공통 원칙

모든 단계는 다음을 만족해야 한다.

- 구현 상태가 문서와 일치한다.
- 실패 조건에서 crash보다 명시적 error handling을 우선한다.
- 데이터 구조가 무제한으로 증가하지 않는다.
- 좌표계와 단위가 문서에 정의되어 있다.
- 같은 입력에 재현 가능한 검증 절차가 있다.
- Release build 성능 결과와 Debug build 결과를 섞지 않는다.

---

## 3. 단계별 완료 기준

### 1. Synthetic Pose → Cube

목표:

```text
Synthetic Pose
→ Pose
→ Transform
→ Flecs Entity
→ RenderSystem
→ OpenGL
```

완료 기준:

- synthetic position 변화가 Cube translation에 반영된다.
- synthetic quaternion 변화가 Cube rotation에 반영된다.
- 같은 시간 입력에서 같은 Pose가 생성된다.
- `SyntheticPoseSource`가 OpenGL/Flecs 타입에 의존하지 않는다.
- `Renderer`/`RenderSystem`이 Pose source를 직접 알지 않는다.

프로젝트 진행 상태에서는 이 단계를 완료로 관리한다.

> 현재 GitHub master snapshot에는 synthetic source 연결 코드가 아직 반영되지 않은 상태다. 이 문서의 완료 표시는 현재 작업 단계 기준이며, repository source가 갱신되면 다시 코드 기준으로 확인한다.

---

### 2. UDP Object Pose

목표:

```text
Synthetic Sender Process
→ Encode
→ UDP
→ Decode
→ Viewer Object
```

완료 기준:

- sender/viewer가 별도 process로 실행된다.
- encode/decode round-trip에서 position/orientation이 허용 오차 내에서 보존된다.
- malformed/unsupported packet이 scene state를 손상시키지 않는다.
- loopback UDP에서도 local synthetic motion과 동일한 trajectory가 재현된다.
- socket/resource가 정상 종료된다.

---

### 3. ArUco Object Detection

완료 기준:

- camera open/close가 정상 동작한다.
- calibration data를 읽을 수 있다.
- known marker를 검출한다.
- `solvePnP`로 object pose를 생성한다.
- marker translation/rotation 방향이 simulation의 좌표계 규약과 일치한다.
- detection 실패 상태를 구분한다.
- Synthetic source와 같은 UDP object pose path를 재사용한다.

정확도 수치는 camera/marker/조명 조건을 기록한 baseline 후 확정한다.

---

### 4. Simulation에 Object 생성

완료 기준:

- 수신 Object Pose가 별도 tracked object entity의 `Transform`으로 반영된다.
- object position/orientation이 camera test와 일관된다.
- reference axis/object를 이용해 좌표계 방향을 확인할 수 있다.
- network/vision code가 `Renderer`에 직접 들어가지 않는다.

---

### 5. Robot Model 추가

완료 기준:

- 선택한 robot model의 출처와 license를 기록한다.
- base, link, joint, end-effector frame이 식별된다.
- 각 joint의 origin/axis/limit 정보가 정의된다.
- link mesh와 kinematic description을 분리해서 관리한다.
- 모든 link를 simulation에 렌더링할 수 있다.

---

### 6. FK 구현

완료 기준:

- joint angle 입력으로 각 link world transform을 계산한다.
- zero/reference configuration이 외부 reference 또는 robot description과 일치한다.
- joint 하나씩 움직였을 때 예상 axis로 회전한다.
- end-effector pose를 계산할 수 있다.
- parent-child transform accumulation이 unit/integration test로 검증된다.

IK 구현 전에 FK 결과를 고정한다.

---

### 7. Grasp Pose 정의

완료 기준:

- object frame을 명확히 정의한다.
- `T_object_grasp`를 명시한다.
- `T_base_camera`, `T_camera_object`, `T_object_grasp`를 조합해 `T_base_grasp`를 계산한다.
- grasp target frame을 debug visualization으로 확인할 수 있다.
- object가 움직이면 grasp target도 같은 relative offset을 유지한다.

---

### 8. IK 구현

완료 기준:

- target end-effector pose를 입력받는다.
- reachable target에서 solver가 joint configuration을 찾는다.
- FK로 다시 계산한 end-effector pose가 target과 설정한 tolerance 안에 들어온다.
- unreachable/non-convergent target을 실패 상태로 반환한다.
- joint limit 정책이 명시되어 있다.

position/orientation tolerance의 최종 수치는 robot model과 solver baseline 후 확정한다.

---

### 9. End Effector → Grasp Pose 추종

완료 기준:

- moving object의 grasp pose가 매 update 계산된다.
- IK 결과가 joint state에 반영된다.
- FK 결과로 robot link transform이 갱신된다.
- target이 reachable한 동안 end-effector가 grasp frame을 추종한다.
- solver failure 시 NaN/폭주 joint 값으로 scene이 손상되지 않는다.

---

### 10. Grasp 성공 시 Object Attach

초기 grasp는 kinematic attach다.

완료 기준:

- position error와 orientation error를 계산한다.
- 성공 threshold가 config/constant로 명시되어 있다.
- threshold를 동시에 만족할 때만 grasp success로 전이한다.
- attach 이후 object가 end-effector relative transform을 유지한다.
- release 기능을 구현한다면 attach 전 world pose가 불필요하게 튀지 않는다.

물리 접촉력/충돌 기반 성공 판정은 기본 범위가 아니다.

---

### 11. Network jitter/loss 실험

완료 기준:

- ideal / immediate / buffered policy를 같은 synthetic trajectory에서 비교한다.
- delay/jitter/loss/reorder 조건을 재현할 수 있다.
- packet/pose buffer가 bounded 상태를 유지한다.
- network metric과 end-effector/grasp metric을 함께 기록한다.
- buffering이 안정성에 주는 이득과 추가 latency를 함께 제시한다.
- 최종 buffer/interpolation 정책을 측정 결과로 설명한다.

---

## 4. 비기능 요구사항

### NFR-01 Ownership / Lifetime

- Window/OpenGL context보다 GPU resource가 먼저 파괴된다.
- worker thread를 detach한 채 process를 종료하지 않는다.
- socket/camera/resource는 RAII 또는 명시적 owner가 존재한다.

### NFR-02 Bounded State

향후 queue/buffer가 생기면 maximum capacity를 반드시 가진다.

```text
Network Queue
PoseBuffer
Debug Log Buffer
```

등이 처리 지연으로 무제한 증가해서는 안 된다.

### NFR-03 Dependency Boundary

금지:

```text
common → OpenGL/OpenCV/Flecs
vision → Renderer
transport → Viewer
kinematics → OpenGL
Vision Node → Flecs/OpenGL
```

### NFR-04 Observability

기능 단계가 늘어날 때 최소한 다음 상태를 확인할 수 있어야 한다.

- current Object Pose
- packet count/error state
- IK convergence state
- current end-effector error
- grasp state

방법은 log/debug overlay/test output 중 단계에 맞게 선택한다.

### NFR-05 Reproducibility

정량 실험에는 다음을 기록한다.

- hardware/OS
- compiler/build type
- camera/robot model
- input trajectory
- network profile
- seed
- runtime duration

---

## 5. 수치 목표를 정하는 방법

아직 구현되지 않은 단계에 임의 숫자를 성과 목표처럼 넣지 않는다.

각 기능은 다음 순서로 target을 만든다.

```text
Correctness 구현
→ Baseline 측정
→ 병목/오차 원인 확인
→ 만족 기준 설정
→ 같은 환경에서 재측정
```

예:

```text
IK position error
network pose age
Viewer frame time
ArUco stationary jitter
```

같은 값은 실제 구현/환경이 준비된 뒤 `측정 조건 + baseline + target` 형태로 갱신한다.

---

## 6. 프로젝트 완료 정의

기본 프로젝트는 다음 전체 경로가 동작하고 단계별 검증 근거가 남아 있을 때 완료로 본다.

```text
Remote Camera / Synthetic Object
→ 6DoF Object Pose
→ UDP
→ Simulation Object
→ Grasp Pose
→ IK
→ FK
→ Robot Arm Visualization
→ Kinematic Grasp
→ Network Robustness Evaluation
```

단순히 robot이 화면에서 움직이는 것으로 완료하지 않는다. Object frame, grasp frame, IK/FK correctness와 network 조건별 결과를 설명할 수 있어야 한다.
