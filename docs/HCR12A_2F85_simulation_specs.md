# HCR-12A + Robotiq 2F-85 Simulation Specification

> 기준일: 2026-10-04  
> 현재 runtime asset: `assets/HCR12A_2F-85.glb`  
> 목적: HCR-12A + 2F-85의 장비 규격, 프로젝트 기구학 기준, GLB-specific 좌표를 구분해 시뮬레이션 source of truth를 명확히 한다.

## 1. 먼저 구분해야 하는 데이터 종류

이 문서의 숫자는 모두 제조사 공식값이 아니다.

| 분류 | 의미 |
|---|---|
| `MANUFACTURER` | 제조사 매뉴얼/제품/프로토콜이 정의한 값 |
| `CAD-DERIVED` | STEP/CAD의 실제 체결 feature에서 구한 값 |
| `KINEMATIC-REFERENCE` | 공개 URDF/Xacro 등 시뮬레이션 기구학 모델 기준 |
| `ASSET-DERIVED` | 현재 controller-ready GLB 좌표계에 맞춰 변환한 값 |

상세 provenance는 `docs/MODEL_DATA_PROVENANCE.md`를 기준으로 한다.

---

## 2. Hanwha HCR-12A 기본 규격 — MANUFACTURER

| 항목 | 값 |
|---|---:|
| 자유도 | 6 DOF |
| 정격 payload | 12 kg |
| Reach / work radius | 1300 mm |
| 관절 형태 | J1~J6 revolute |

현재 runtime에서 사용하는 관절 limit/속도:

| Joint | Limit [deg] | Limit [rad] | Max speed [deg/s] | Max speed [rad/s] |
|---|---:|---:|---:|---:|
| J1 | -180 ~ 180 | -3.141593 ~ 3.141593 | 130 | 2.268928 |
| J2 | -165 ~ 135 | -2.879793 ~ 2.356194 | 130 | 2.268928 |
| J3 | -85 ~ 245 | -1.483530 ~ 4.276057 | 200 | 3.490659 |
| J4 | -190 ~ 190 | -3.316126 ~ 3.316126 | 200 | 3.490659 |
| J5 | -170 ~ 170 | -2.967060 ~ 2.967060 | 200 | 3.490659 |
| J6 | -360 ~ 360 | -6.283185 ~ 6.283185 | 200 | 3.490659 |

현재 `Hcr12a.h`는 degree/degree-per-second 값을 SI runtime 단위인 rad/rad/s로 저장한다.

> HCR revision/문서에 따라 J2/J3 범위가 다르게 표기된 자료가 있으므로 실제 장비와 1:1 대응할 때는 해당 controller/manual revision을 다시 확인한다.

---

## 3. HCR 기계 pivot — CAD-DERIVED

현재 J1~J6 pivot은 mesh 중심이나 임의 추정점이 아니다.
STEP assembly에서 서로 맞물리는 원통형 체결 feature의 중심이 양쪽 부품에서 일치하는지 확인해 결정했다.

| Joint | Bind pivot [m] | Physical/runtime axis |
|---|---|---|
| J1 | `(0.0000, 0.1985, 0.00000)` | `+Y (0,1,0)` |
| J2 | `(0.1535, 0.3100, 0.10000)` | `+X (1,0,0)` |
| J3 | `(0.1095, 0.9100, 0.10000)` | `+X (1,0,0)` |
| J4 | `(0.0000, 1.0150, 0.23350)` | `+Z (0,0,1)` |
| J5 | `(-0.1385, 1.0150, 0.69100)` | `+X (1,0,0)` |
| J6 | `(0.0000, 1.0150, 0.85475)` | `+Z (0,0,1)` |

ToolFrame bind world position:

```text
(0.0000, 1.0150, 0.9145) m
```

원본 CAD의 mm 좌표를 runtime에서는 meter로 정규화했다.

---

## 4. HCR controller-ready GLB contract — ASSET-DERIVED

현재 `assets/HCR12A_2F-85.glb`는 다음 contract를 사용한다.

```text
J1~J6 moving node bind rotation = identity
J1 axis = +Y
J2 axis = +X
J3 axis = +X
J4 axis = +Z
J5 axis = +X
J6 axis = +Z
```

이전 중간 모델에서 사용했던 ±90° bind quaternion 보정은 현재 contract가 아니다.
Arm visual transform은 mesh vertex/normal에 bake해 bind pose 외형을 유지한다.

`RobotTransformAdapter`는 다음 방식으로 상태를 화면에 반영한다.

```text
RobotState q [rad]
+
JointSpecification.axis
→ angleAxis(q, axis)
→ Joint local rotation
```

Pivot translation은 GLB hierarchy에 이미 존재하므로 매 frame 코드에서 다시 더하지 않는다.

---

## 5. Robotiq 2F-85 제조사 수준 규격 — MANUFACTURER

대표 장비 규격:

| 항목 | 값 |
|---|---:|
| Maximum opening | 85 mm |
| Grasp force | 20~235 N |
| Finger speed | 20~150 mm/s |
| Nominal payload | 5 kg |
| Mass | 약 0.925 kg |
| Position resolution | 약 0.4 mm |
| Supply | 24 V DC 계열 |

2F-85는 rotational adaptive / under-actuated gripper다.
손가락은 단순 prismatic slider처럼 좌우로 평행 이동하지 않고 knuckle/linkage 회전으로 열리고 닫힌다.

### 장치 command/state

| 값 | 의미 | 범위 |
|---|---|---:|
| rPR | Position Request | `0=open`, `255=closed` |
| rSP | Speed Request | `0..255` |
| rFR | Force Request | `0..255` |
| gPO | Actual/encoder position 계열 값 | `0..255` |
| gOBJ | Object/motion status | device status code |

중요:

```text
rPR = raw command
rPR != mm
rPR != rad
```

0~255와 실제 opening은 제조사 문서에서 quasi-linear 관계로 다뤄지므로, 정밀 실기기 일치를 원하면 추후 LUT/측정 보정이 필요하다.

---

## 6. 2F-85 free-space kinematic reference — KINEMATIC-REFERENCE

현재 프로젝트는 공개 2F-85 kinematic model의 master/mimic 개념을 자유 공간 자세 기준으로 사용한다.

```text
nominal master closed q = 0.7929 rad ≈ 45.43°
```

이 값은 **Robotiq 내부 motor shaft angle이 아니다.**
시뮬레이션 linkage의 master joint reference다.

Free-space mimic 관계:

| Joint | Master multiplier | nominal range [rad] |
|---|---:|---:|
| LeftOuterKnuckleJoint | +1 | 0 ~ +0.8 |
| RightOuterKnuckleJoint | -1 | -0.8 ~ 0 |
| LeftInnerKnuckleJoint | +1 | 0 ~ +0.8 |
| RightInnerKnuckleJoint | -1 | -0.8 ~ 0 |
| LeftFingerTipJoint | -1 | -0.8 ~ 0 |
| RightFingerTipJoint | +1 | 0 ~ +0.8 |

고정 mimic은 **물체와 접촉하기 전 자유공간 opening/closing**을 표현한다.
접촉 후 passive adaptive motion까지 이 표만으로 재현하지 않는다.

---

## 7. 현재 GLB의 2F-85 pivot/axis — ASSET-DERIVED

현재 `assets/HCR12A_2F-85.glb`에 맞춘 Gripper-local frame 값:

```text
joint axis    = (0, 0, -1)
joint plane Z = 0.0934257339 m
```

| Joint | Gripper-local pivot [m] |
|---|---|
| LeftOuterKnuckleJoint | `(-0.03060114, 0.05490452, 0.0934257339)` |
| RightOuterKnuckleJoint | `(+0.03060114, 0.05490452, 0.0934257339)` |
| LeftInnerKnuckleJoint | `(-0.01270000, 0.06142000, 0.0934257339)` |
| RightInnerKnuckleJoint | `(+0.01270000, 0.06142000, 0.0934257339)` |
| LeftFingerTipJoint | `(-0.06775864, 0.09832620, 0.0934257339)` |
| RightFingerTipJoint | `(+0.06775864, 0.09832620, 0.0934257339)` |

이 값은 공개 kinematic frame을 현재 GLB geometry에 대응한 **프로젝트 asset 좌표**다.
다른 2F-85 CAD/GLB에 그대로 사용할 수 있는 제조사 universal coordinate가 아니다.

---

## 8. 현재 Gripper hierarchy

과거의 `LeftFingerJoint / RightFingerJoint` 두-node 모델은 폐기됐다.
현재 구조:

```text
Gripper
├─ GripperMesh
├─ LeftOuterKnuckleJoint
│  └─ LeftOuterKnuckle
│     ├─ LeftOuterKnuckleMesh
│     └─ LeftFinger
│        ├─ LeftFingerMesh
│        └─ LeftFingerTipJoint
│           └─ LeftFingerTip
│              └─ LeftFingerTipMesh
├─ LeftInnerKnuckleJoint
│  └─ LeftInnerKnuckle
│     └─ LeftInnerKnuckleMesh
├─ RightOuterKnuckleJoint
│  └─ RightOuterKnuckle
│     ├─ RightOuterKnuckleMesh
│     └─ RightFinger
│        ├─ RightFingerMesh
│        └─ RightFingerTipJoint
│           └─ RightFingerTip
│              └─ RightFingerTipMesh
└─ RightInnerKnuckleJoint
   └─ RightInnerKnuckle
      └─ RightInnerKnuckleMesh
```

Moving gripper joints의 bind rotation도 identity다.

---

## 9. Gripper geometry/동작 검증

현재 linkage geometry 기준 자유 공간 검증:

```text
OPEN gap   ≈ 85.00 mm
CLOSED gap ≈ 0.97 mm
```

원본 finger geometry의 triangle 수는 rigid component 분할 후에도 보존됐다.

```text
Left  12,952 -> 12,952
Right 12,952 -> 12,952
```

현재 runtime GLB에는 baked open/close animation이 없다.
GLB는 geometry/hierarchy/pivot만 제공하고 동작은 향후 C++ controller가 만든다.

---

## 10. 현재 코드 상태

### 구현됨

```text
IRobotController
IGripperController contract
RobotSpecification / GripperSpecification
Hcr12a model specification
TwoF85 model specification
SimRobotController : IRobotController
RobotTransformAdapter
Joint angle limit 검증
Joint max velocity 기반 target 추종
software Stop
```

### 아직 미구현/후순위

```text
Fixed Control Loop
Acceleration limiting (검증된 max acceleration 값 필요)
FK
IK
SimGripperController
Hardware Robot backend
Hardware Gripper backend
Jolt Physics
Contact-based adaptive grasp
Watchdog / E-Stop state / Zero Offset
```

그리퍼 runtime 제어는 현재 의도적으로 뒤로 미뤘다. 먼저 Robot/Hardware/Simulation 공통 interface와 simulation robot backend 구조를 고정한다.

---

## 11. 향후 gripper runtime 흐름

자유 공간:

```text
rPR 0..255
    ↓
SimGripperController
    ↓
model-specific rPR -> master q mapping
    ↓
masterMultiplier
    ↓
6 linkage Joint local rotation
```

초기에는 endpoint가 맞는 단순 mapping으로 시작할 수 있지만 실제 장비 정밀 대응이 필요하면 256-entry LUT 또는 측정 기반 calibration을 사용한다.

접촉 이후:

```text
rPR / rFR
    ↓
Physics motor / constraints
    ↓
Collision / contact
    ↓
under-actuated passive adaptation
    ↓
object grasp
```

---

## 12. Source of truth

모델 규격 코드:

```text
modules/robotics/include/robotics/models/hanwha/Hcr12a.h
modules/robotics/include/robotics/models/robotiq/TwoF85.h
```

수치 provenance:

```text
docs/MODEL_DATA_PROVENANCE.md
```

GLB 구조:

```text
docs/HCR12A GLB 정규화.md
```

Controller architecture:

```text
docs/ARCHITECTURE.md
docs/CONTROLLER_INTERFACE.md
```
