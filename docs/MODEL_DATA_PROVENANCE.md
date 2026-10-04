# Model Data Provenance

## 목적

`modules/robotics/include/robotics/models/`에 있는 숫자는 모두 같은 종류의 데이터가 아니다.
제조사 공식 규격, CAD에서 직접 추출한 기계 형상, 공개 kinematic reference, 현재 GLB에 맞춰 변환한 asset-specific 값이 함께 존재한다.

이 구분을 하지 않으면 `TwoF85.h`의 pivot이나 `0.7929 rad` 같은 값을 Robotiq 공식 모터 사양으로 오해하기 쉽다.
따라서 모델 상수와 관련 문서에서는 아래 provenance 분류를 명시한다.

## 분류

| 분류 | 의미 | 예시 |
|---|---|---|
| `MANUFACTURER` | 제조사 매뉴얼/제품 명세/장치 protocol이 직접 정의한 값 | HCR 관절 limit·속도, 2F-85 rPR/rSP/rFR 범위 |
| `CAD-DERIVED` | STEP/CAD의 실제 체결 형상과 축에서 계산한 값 | HCR J1~J6 기계 pivot 중심 |
| `KINEMATIC-REFERENCE` | 공개 URDF/Xacro 등 기구학 모델에서 가져온 시뮬레이션 기준 | 2F-85 master closed angle, mimic 관계 |
| `ASSET-DERIVED` | 위 데이터를 현재 controller-ready GLB 좌표계에 맞춰 변환한 값 | 2F-85 Gripper-local pivot, local -Z axis, joint plane Z |

`KINEMATIC-REFERENCE`와 `ASSET-DERIVED`는 제조사의 내부 모터 설계값을 의미하지 않는다.

---

## HCR-12A

코드 위치:

```text
modules/robotics/include/robotics/models/hanwha/Hcr12a.h
```

### 관절 limit / 최대속도 — MANUFACTURER

현재 runtime specification은 HCR-A(2G) 계열에 사용한 다음 값을 SI 단위로 정규화한다.

| Joint | limit [deg] | runtime limit [rad] | max [deg/s] | runtime [rad/s] |
|---|---:|---:|---:|---:|
| J1 | -180 ~ 180 | -3.141593 ~ 3.141593 | 130 | 2.268928 |
| J2 | -165 ~ 135 | -2.879793 ~ 2.356194 | 130 | 2.268928 |
| J3 | -85 ~ 245 | -1.483530 ~ 4.276057 | 200 | 3.490659 |
| J4 | -190 ~ 190 | -3.316126 ~ 3.316126 | 200 | 3.490659 |
| J5 | -170 ~ 170 | -2.967060 ~ 2.967060 | 200 | 3.490659 |
| J6 | -360 ~ 360 | -6.283185 ~ 6.283185 | 200 | 3.490659 |

변환은 개념적으로 다음과 같다.

```text
rad   = deg * pi / 180
rad/s = deg/s * pi / 180
```

### J1~J6 pivot — CAD-DERIVED

Pivot은 mesh bounding-box 중심이나 임의의 회전점이 아니다.
HCR-12A STEP assembly에서 서로 맞물리는 원통형 체결 feature의 중심을 양쪽 부품에서 대조해 결정했다.
원본 CAD mm 좌표를 runtime에서는 meter로 저장한다.

| Joint | bind pivot [m] | physical axis |
|---|---|---|
| J1 | `(0.0000, 0.1985, 0.00000)` | `+Y` |
| J2 | `(0.1535, 0.3100, 0.10000)` | `+X` |
| J3 | `(0.1095, 0.9100, 0.10000)` | `+X` |
| J4 | `(0.0000, 1.0150, 0.23350)` | `+Z` |
| J5 | `(-0.1385, 1.0150, 0.69100)` | `+X` |
| J6 | `(0.0000, 1.0150, 0.85475)` | `+Z` |

ToolFrame bind 위치는 `(0, 1.0150, 0.9145)` m다.

### controller-ready axis — CAD-DERIVED + ASSET-DERIVED

물리 회전축 방향은 CAD에서 확인했지만, 코드의 `axis`는 현재 controller-ready GLB의 Joint local frame에서 사용하는 단위축이다.
현재 GLB는 J1~J6 moving node의 bind rotation을 identity로 만들었으므로 physical axis와 runtime local axis가 직접 일치한다.

---

## Robotiq 2F-85

코드 위치:

```text
modules/robotics/include/robotics/models/robotiq/TwoF85.h
```

### rPR / rSP / rFR — MANUFACTURER

`0..255`는 Robotiq 장치가 사용하는 command 값이다.

```text
rPR  position request  0=open, 255=closed
rSP  speed request     0..255
rFR  force request     0..255
```

이 값은 meter나 radian이 아니다.
향후 Hardware backend는 장치 register에 매핑하고, Simulation backend는 모델별 기구학 상태로 변환한다.

### 0.7929 rad / mimic 관계 — KINEMATIC-REFERENCE

`kTwoF85NominalClosedMasterRadians = 0.7929`는 공개 2F-85 kinematic model의 free-space nominal closed master joint 기준값이다.

중요한 해석:

```text
0.7929 rad = simulation linkage master joint angle
0.7929 rad != Robotiq 내부 motor shaft angle
```

`masterMultiplier = ±1`도 하나의 actuator command에서 좌우 knuckle/fingertip을 연동하기 위한 free-space mimic 관계다.
물체 접촉 이후 실제 2F-85의 under-actuated adaptive motion까지 고정 mimic으로 표현하는 값은 아니다.

### pivot / local -Z / joint plane — ASSET-DERIVED

다음 값은 현재 `assets/HCR12A_2F-85.glb`에 맞춘 프로젝트 좌표다.

```text
joint plane Z = 0.0934257339 m
joint axis    = (0, 0, -1) in Gripper local frame
```

Pivot:

| Joint | Gripper-local pivot [m] |
|---|---|
| LeftOuterKnuckleJoint | `(-0.03060114, 0.05490452, 0.0934257339)` |
| RightOuterKnuckleJoint | `(+0.03060114, 0.05490452, 0.0934257339)` |
| LeftInnerKnuckleJoint | `(-0.01270000, 0.06142000, 0.0934257339)` |
| RightInnerKnuckleJoint | `(+0.01270000, 0.06142000, 0.0934257339)` |
| LeftFingerTipJoint | `(-0.06775864, 0.09832620, 0.0934257339)` |
| RightFingerTipJoint | `(+0.06775864, 0.09832620, 0.0934257339)` |

이 좌표들은 공개 kinematic frame을 기존 gripper geometry에 정렬해 만든 값이므로 **다른 2F-85 GLB에 그대로 적용하면 안 된다.**
Asset이 바뀌면 다시 검증해야 한다.

---

## Runtime 변환 경계

HCR-12A:

```text
MANUFACTURER / CAD
      ↓ normalize
Hcr12a.h [m, rad, rad/s]
      ↓
SimRobotController -> RobotState q [rad]
      ↓
RobotTransformAdapter
      q + Joint-local axis
      -> angleAxis(q, axis)
      -> current ECS Euler [rad]
      ↓
Flecs / GLB
```

2F-85는 아직 runtime controller를 구현하지 않았다.
향후 자유공간 동작은 다음 경계를 사용한다.

```text
rPR [0..255]
      ↓
SimGripperController
      ↓ model-specific mapping
master q [rad]
      ↓ masterMultiplier
6 linkage Joint local rotations
```

접촉 이후는 Jolt Physics/contact/constraint 계층에서 처리한다.

## 작성 규칙

앞으로 model constant를 추가할 때 다음을 지킨다.

1. 주석에 `MANUFACTURER`, `CAD-DERIVED`, `KINEMATIC-REFERENCE`, `ASSET-DERIVED` 중 출처 성격을 적는다.
2. 숫자에는 단위를 적는다.
3. 좌표는 어느 frame인지 적는다.
4. 변환된 값이라면 원 단위와 runtime 단위를 함께 기록한다.
5. 공개 simulation model의 값을 제조사 공식 기계/모터 사양처럼 서술하지 않는다.
6. asset-specific 값은 대상 GLB 이름과 좌표계 contract를 함께 기록한다.
