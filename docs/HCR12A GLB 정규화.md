# HCR-12A + Robotiq 2F-85 Controller-ready GLB 정규화

> 현재 대상 자산: `assets/HCR12A_2F-85.glb`  
> 기준: 2026-10-04  
> 목적: CAD 형상을 유지하면서 HCR-12A J1~J6와 2F-85 linkage를 런타임 Controller가 직접 구동할 수 있는 GLB 구조로 만든다.

## 1. 현재 자산 contract

현재 runtime GLB는 다음 원칙을 사용한다.

1. HCR J1~J6 moving node의 **bind rotation은 identity**다.
2. 관절 회전축은 Joint node에 미리 quaternion 보정을 넣는 대신 모델 specification의 local axis로 정의한다.
3. Arm visual geometry는 기존 bind pose가 바뀌지 않도록 vertex에 transform을 bake한다.
4. Gripper는 단순 좌/우 translation finger가 아니라 outer/inner knuckle와 fingertip joint로 분리한다.
5. Runtime GLB에는 **그리퍼 open/close baked animation을 넣지 않는다.** 움직임은 C++ controller와 기구학 계산이 만든다.
6. 물체 접촉 이후 2F-85의 under-actuated adaptive motion은 후속 Physics 작업이며 아직 구현하지 않았다.

과거 문서의 `BindTransform * JointRotation` 전제와 `LeftFingerJoint/RightFingerJoint` 두 개만 사용하는 구조는 현재 자산 기준으로 폐기됐다.

---

## 2. HCR-12A hierarchy

Arm의 논리 hierarchy는 다음과 같다.

```text
RobotRoot
└─ Base
   ├─ BaseMesh
   └─ J1
      └─ Link1
         ├─ Link1Mesh
         └─ J2
            └─ Link2
               ├─ Link2Mesh
               └─ J3
                  └─ Link3
                     ├─ Link3Mesh
                     └─ J4
                        └─ Link4
                           ├─ Link4Mesh
                           └─ J5
                              └─ Link5
                                 ├─ Link5Mesh
                                 └─ J6
                                    └─ Link6
                                       ├─ Link6Mesh
                                       └─ ToolFrame
                                          └─ Gripper
```

CAD 내부의 Housing/Cover/Bolt 같은 상세 assembly hierarchy는 런타임 로봇 제어 구조에서 제거하거나 visual mesh에 bake했다.

---

## 3. HCR J1~J6 pivot과 axis

Pivot은 mesh 중심이 아니다. STEP assembly에서 서로 맞물리는 원통형 체결 feature의 중심을 양쪽 부품에서 대조해 얻은 기계 회전 중심이다.
원본 mm 좌표를 runtime에서 meter로 사용한다.

| Joint | Bind pivot [m] | Runtime local axis |
|---|---|---|
| J1 | `(0.0000, 0.1985, 0.00000)` | `+Y (0,1,0)` |
| J2 | `(0.1535, 0.3100, 0.10000)` | `+X (1,0,0)` |
| J3 | `(0.1095, 0.9100, 0.10000)` | `+X (1,0,0)` |
| J4 | `(0.0000, 1.0150, 0.23350)` | `+Z (0,0,1)` |
| J5 | `(-0.1385, 1.0150, 0.69100)` | `+X (1,0,0)` |
| J6 | `(0.0000, 1.0150, 0.85475)` | `+Z (0,0,1)` |

ToolFrame bind world position은 `(0, 1.0150, 0.9145)` m다.

이 값의 출처 분류는 `docs/MODEL_DATA_PROVENANCE.md`를 따른다.

---

## 4. 왜 moving Joint bind rotation을 Identity로 바꿨는가

이전 중간 모델에서는 물리축을 모두 local Z처럼 다루기 위해 J1/J2/J3/J5 등에 약 ±90° bind quaternion을 넣는 방식도 사용했다.
그 방식은 시각적으로는 맞을 수 있지만 런타임에서 다음 두 종류 회전을 동시에 이해해야 한다.

```text
asset bind rotation
×
command joint rotation
```

그 결과 Inspector에서 `w≈-0.707`, `z≈0.707` 같은 값이 보이고, 잘못된 local axis를 전달하면 회전이 사선처럼 보이기 쉬웠다.

현재 자산은 다음처럼 단순화했다.

```text
J1..J6 bind rotation = identity
Joint position       = 실제 기계 pivot
Joint axis           = 모델 specification에 명시
```

따라서 arm 회전은 다음 의미가 된다.

```text
q [rad]
+
joint pivot + local axis
→ RobotKinematics
→ Joint local rotation / LinkPose
→ RobotTransformAdapter / RobotPhysicsAdapter
```

현재 `RobotTransformAdapter`도 이 asset contract를 검사한다. J1~J6 bind quaternion이 identity 회전을 나타내지 않으면 controller-ready GLB가 아니라고 판단한다. `q`와 `-q`는 같은 회전으로 취급한다.

---

## 5. Visual transform bake

Joint frame을 identity bind로 정리하면서 외형이 움직이면 안 된다.
따라서 기존 visual node에 들어 있던 CAD 정렬 transform을 mesh vertex/normal에 미리 반영했다.

개념적으로 기존:

```text
ParentWorld * VisualLocal * Vertex
```

를 다음으로 바꾼다.

```text
ParentWorld * Identity * (VisualLocal * Vertex)
```

즉 **world bind pose는 유지하면서 visual node transform만 단순화**한다.

Arm visual bind-pose 검증에서 최대 world-space vertex 오차는 약 `1.192e-7 m`로 floating-point 오차 수준이었다.

---

## 6. Robotiq 2F-85 현재 hierarchy

현재 gripper는 단순 `LeftFingerJoint / RightFingerJoint` 두 개가 아니다.
다음 six-joint free-space linkage 구조를 사용한다.

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

Moving gripper joint도 bind rotation은 identity다.
현재 GLB에 맞춘 local rotation axis는 `-Z`다.

---

## 7. Gripper geometry 분할

기존 runtime asset의 좌/우 finger mesh는 여러 실제 rigid part가 하나의 mesh로 합쳐진 상태였다.
현재 controller-ready 자산에서는 원본 finger geometry의 connected component를 rigid body 단위로 분류해 다음 그룹으로 재구성했다.

```text
outer knuckle
finger
inner knuckle
finger tip
```

Triangle을 임의로 잘라내거나 형상을 단순화하지 않았다.
검증 결과 좌우 각각 원본 triangle 수와 재구성 triangle 수가 동일했다.

```text
Left  : 12,952 -> 12,952
Right : 12,952 -> 12,952
```

공개 2F-85 kinematic frame을 현재 gripper geometry에 매핑한 것이므로 pivot/plane/axis 좌표는 **asset-specific**이다. 제조사 내부 motor 설계 좌표로 해석하면 안 된다.

---

## 8. Gripper animation 제거

검증 단계에서는 `Open -> Close -> Open` animation을 넣어 linkage가 실제로 접히는지 확인했지만, 현재 runtime 자산에서는 제거했다.

현재 contract:

```text
GLB
= geometry + hierarchy + pivot/frame

C++ Controller
= command/state + joint rotation
```

현재 자유 공간 제어 흐름은 다음과 같다.

```text
rPR 0..255
   ↓
SimGripperController
   ↓
GripperState.closureFraction [0,1]
   ↓ GripperKinematics
master linkage q [rad]
   ↓
mimic relation
   ↓
6개 gripper Joint local rotation
   ↓ GripperTransformAdapter
GLB 관절 / 기존 7개 Kinematic proxy
```

`rPR=0`은 fully open, `255`는 fully closed지만, `rPR` 자체는 radian이나 mm가 아니다.

---

## 9. 2F-85 free-space 검증

현재 geometry와 linkage frame으로 확인한 nominal free-space 결과:

```text
OPEN   fingertip inner gap ≈ 85.00 mm
CLOSED fingertip inner gap ≈ 0.97 mm
nominal closed master q    ≈ 0.7929 rad
```

`0.7929 rad`는 공개 kinematic reference의 master linkage 기준각이며 실제 내부 motor shaft angle이 아니다.

실제 2F-85는 under-actuated 구조다. 물체가 먼저 한 phalanx에 접촉하면 이후 joint 관계가 고정 mimic animation과 달라질 수 있다.
따라서 접촉 이후 동작은 향후 Jolt Physics의 contact/constraint/motor force 계층에서 처리한다.

---

## 10. 런타임 변환 경로

Arm은 다음 순서로 움직인다.

```text
Hcr12a.h
  axis / limit / velocity
        ↓
SimRobotController
  RobotState.q [rad]
        ↓
RobotKinematics
  q + pivot / axis
  → joint rotation + link pose
        ↓
RobotTransformAdapter
  quaternion → ECS unit quaternion
        ↓
Entity::SetLocalRotation
        ↓
TransformSystemModule
  Local = T * R * S
  World = ParentWorld * Local
        ↓
Renderer
```

Pivot translation은 GLB hierarchy에 이미 들어 있다. `RobotTransformAdapter`가 다시 더하지 않고, `RobotKinematics`가 동일한 bind pivot을 사용해 base-frame LinkPose를 계산한다.

glTF `[x,y,z,w]`는 로더에서 GLM 생성자 `(w,x,y,z)`로 옮긴다. `NodeData`·Entity는 quaternion을 보관하고 행렬 합성 전에 검증·정규화한다. 영 quaternion·NaN 등 비유한 입력은 거부하며 Euler로 왕복 변환하지 않는다. 자세 저장과 행렬은 float, 관절각은 rad 스칼라를 유지한다. 현재 계약은 [Architecture](ARCHITECTURE.md)와 [그리퍼 런타임 설계](GRIPPER_RUNTIME_DESIGN.md)를 참고한다.

---

## 11. 코드의 source of truth

모델 상수:

```text
modules/robotics/include/robotics/models/hanwha/Hcr12a.h
modules/robotics/include/robotics/models/robotiq/TwoF85.h
```

Controller:

```text
modules/robotics/include/robotics/core/IRobotController.h
modules/robotics/include/robotics/backends/simulation/SimRobotController.h
```

Viewer 반영:

```text
modules/scene/include/viewer/robotics/RobotTransformAdapter.h
modules/scene/src/robotics/RobotTransformAdapter.cpp
```

수치 출처/변환 성격:

```text
docs/MODEL_DATA_PROVENANCE.md
```

이 문서보다 코드/자산이 변경되면 위 source-of-truth를 먼저 확인하고 문서를 함께 갱신한다.
