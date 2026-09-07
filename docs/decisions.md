# 설계 결정과 범위

## 1. 확정 결정

### D-01 Vision Node와 Viewer/Simulator 분리

두 프로그램을 별도 process로 유지한다.

```text
Vision Node
→ UDP
→ Viewer / Simulator
```

이유: 실제 network boundary와 process lifetime을 검증하기 위해서다.

### D-02 Vision Node는 headless producer

Vision Node는 OpenGL/Flecs GUI application으로 만들지 않는다.

개발 중 preview가 필요하면 OpenCV HighGUI를 선택적으로 사용한다.

### D-03 초기 인식 대상은 known object

초기에는 markerless detector나 일반 grasp planning을 하지 않는다.

```text
Known Object
+ Known ArUco Marker
+ Known Grasp Offset
```

구조로 제한한다.

### D-04 물체 Pose와 Grasp Pose를 분리

```text
Object Pose
≠
End-Effector Target Pose
```

`T_object_grasp`를 별도로 정의하고 IK target을 계산한다.

### D-05 FK를 IK보다 먼저 구현

Robot model hierarchy와 joint axis/origin을 검증한 뒤 IK를 구현한다.

### D-06 초기 grasp는 kinematic attach

물리 엔진 기반 접촉 simulation은 기본 범위에서 제외한다.

End Effector가 위치/회전 성공 조건을 만족하면 object를 end-effector transform에 부착한다.

### D-07 Viewer에만 Flecs 사용

Flecs는 scene object와 rendering system 관리에 사용한다.

Vision, transport, protocol, kinematics core 전체를 ECS에 종속시키지 않는다.

### D-08 UDP 사용

Object Pose는 지속적으로 갱신되는 상태이므로 오래된 packet 복구보다 최신성에 더 높은 우선순위를 둔다.

loss/reorder 문제는 마지막 network robustness 단계에서 직접 측정한다.

### D-09 명시적 binary serialization

C++ struct memory를 그대로 송신하지 않는다.

padding, alignment, ABI, endianness 차이를 피하기 위해 field 단위로 encode/decode한다.

### D-10 Network impairment는 마지막 단계

먼저 ideal/local condition에서 robot grasp pipeline을 완성한 뒤 delay/jitter/loss를 주입한다.

---

## 2. 현재 진행 로드맵

1. **Synthetic Pose → Cube** — 현재 단계
2. UDP Object Pose
3. ArUco Object Detection
4. Simulation에 Object 생성
5. Robot Model 추가
6. FK 구현
7. Grasp Pose 정의
8. IK 구현
9. End Effector → Grasp Pose 추종
10. Grasp 성공 시 Object attach
11. Network jitter/loss 실험

이 순서는 기능을 나열한 것이 아니라 **오류 원인을 한 단계씩 분리하기 위한 구현 순서**다.

---

## 3. 미결정 사항

다음 항목은 실제 구현 단계에 들어가기 전에 결정한다.

### UDP Object Pose 전

- packet exact byte layout
- wire quaternion field order
- timestamp/sequence를 단계 2부터 넣을지 여부
- object ID 포함 여부
- Windows/Linux socket wrapper API

### Robot Model 전

- 사용할 robot model
- robot description 원본(URDF/config 등)
- mesh format
- joint/link data representation
- Flecs hierarchy 사용 여부

### IK 전

- analytic vs numerical solver
- numerical IK 사용 시 Jacobian method
- position/orientation error weighting
- convergence condition
- joint limit 처리

### Grasp Attach 전

- position success threshold
- orientation success threshold
- attach 이후 object transform ownership

### Network Experiment 전

- buffer/interpolation을 robot target 앞에 둘지 object transform 단계에 둘지
- buffer delay 후보
- target/end-effector error metric
- 실험 반복 횟수와 baseline

---

## 4. 기본 완료 범위

```text
Synthetic Pose
→ UDP Object Pose
→ ArUco Object Pose
→ Simulation Object
→ Robot Model
→ FK
→ Grasp Pose
→ IK
→ End-Effector Tracking
→ Kinematic Object Attach
→ Network Impairment Evaluation
```

---

## 5. 기본 범위에서 제외

- markerless object detection
- ML-based grasp generation
- arbitrary-object grasp planning
- collision-free motion planning
- rigid-body/contact physics
- robot dynamics / torque control
- ROS2
- multi-camera sensor fusion
- production security/authentication

이 기능들은 핵심 경로가 완성된 뒤 별도 확장으로만 검토한다.

---

## 6. 설계 원칙

- 구현 전에 추상화를 늘리지 않는다.
- 현재 단계에서 실제로 필요한 dependency만 추가한다.
- 외부 입력, network, kinematics, rendering의 책임 경계를 유지한다.
- 수치 목표는 baseline 측정 없이 임의로 성과처럼 작성하지 않는다.
- 구현 상태와 예정 상태를 문서에서 구분한다.
