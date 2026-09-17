# HCR12A GLB 런타임 구조 정규화

## 1. 작업 배경

로봇 시뮬레이터에 6축 로봇팔 HCR12A 모델을 적용하기 위해 GLB 파일을 TinyGLTF로 로드했다.

기존에 사용하던 단순 로봇 모델은 각 관절과 Mesh가 비교적 단순한 계층으로 구성되어 있어, Node Transform을 순회하며 바로 렌더링 및 관절 제어에 사용할 수 있었다.

반면 HCR12A 모델은 CAD Assembly를 기반으로 생성된 GLB였기 때문에 런타임 제어에 필요한 관절 구조 외에도 Housing, Cover, Bolt, Connector 등 다수의 세부 부품이 각각 별도의 Node와 Mesh로 존재했다.

원본 모델은 다음과 같은 특성을 가지고 있었다.

* Node: 168개
* Mesh: 128개
* J1~J6 관절 Node 존재
* Gripper 및 좌·우 Finger Joint 존재
* CAD 부품별로 다수의 중첩 Node 존재
* 하나의 Link Visual이 여러 단계의 Transform을 거쳐 구성됨
* Mesh 내부가 다수의 Primitive로 세분화됨

이 구조를 그대로 런타임에서 처리할 경우 로봇 제어 코드가 CAD 모델의 내부 구조에 강하게 의존하는 문제가 발생했다.

---

## 2. 문제

### 2.1 Mesh와 관절 Node의 구조 차이

단순 테스트 모델은 대부분 다음과 같은 형태였다.

```text
Base
└─ Arm0
   └─ Arm1
      └─ Arm2
         └─ EndEffector
```

각 Node가 직접 Mesh를 가지고 있어 Node와 렌더링 객체를 거의 일대일로 대응시킬 수 있었다.

HCR12A는 구조가 달랐다.

```text
RobotRoot
└─ BaseLink
   ├─ BaseVisual
   │  └─ CAD Assembly
   │     ├─ Housing
   │     ├─ Cover
   │     ├─ Bolt
   │     └─ ...
   │
   └─ J1
      └─ Link1
         ├─ Link1Visual
         │  └─ CAD Assembly
         │     └─ ...
         └─ J2
```

`J1`, `J2`, `Link1`과 같은 제어상 중요한 Node는 Mesh를 직접 가지고 있지 않았고, 실제 형상은 여러 단계 아래의 CAD Node에 분산되어 있었다.

따라서 Mesh만을 기준으로 Node를 처리할 경우 관절 계층이 유실될 수 있었다.

---

## 3. 초기 접근

처음에는 TinyGLTF 로더가 원본 GLB의 모든 구조를 그대로 처리하도록 구현하는 방향을 검토했다.

이를 위해서는 다음 요소를 모두 런타임에서 처리해야 했다.

* Mesh가 없는 Transform Node
* Node 간 Transform 누적
* glTF TRS 변환
* Quaternion 변환
* CAD Assembly 계층
* 다수의 Primitive
* 관절 기본 Transform
* 관절 회전 Transform
* Gripper 및 Finger 계층

이 방법은 범용 glTF Viewer를 구현할 때는 적절하지만, 현재 프로젝트의 목적과는 차이가 있었다.

프로젝트에서 필요한 정보는 다음과 같이 제한적이었다.

```text
J1
J2
J3
J4
J5
J6
Gripper
LeftFingerJoint
RightFingerJoint
```

즉 프로젝트의 핵심은 GLB 내부 CAD 구조를 해석하는 것이 아니라, 6축 로봇의 관절을 제어하고 FK/IK 결과를 시뮬레이션에 반영하는 것이었다.

따라서 런타임 로더를 복잡하게 만드는 대신 입력 모델 자체를 런타임에 적합한 형태로 정규화하기로 했다.

---

## 4. 목표 구조

런타임 GLB는 다음 구조를 기준으로 정의했다.

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
                                       └─ Gripper
                                          ├─ GripperMesh
                                          ├─ LeftFingerJoint
                                          │  └─ LeftFingerMesh
                                          └─ RightFingerJoint
                                             └─ RightFingerMesh
```

구조 설계 기준은 다음과 같다.

1. J1~J6 관절 Node를 명시적으로 유지한다.
2. Link와 Joint의 부모-자식 관계를 유지한다.
3. 각 Link의 시각적 형상은 하나의 Mesh Node 아래로 모은다.
4. CAD 내부 Assembly Node는 런타임 구조에서 제거한다.
5. Gripper Body와 두 Finger를 독립적인 제어 단위로 유지한다.
6. 관절을 넘어서는 Mesh 병합은 수행하지 않는다.

---

## 5. Transform 정규화

원본 모델에서는 하나의 Mesh가 다음과 같은 여러 Transform을 거쳐 배치될 수 있었다.

```text
J2
└─ Link2
   └─ Visual
      └─ Assembly
         └─ Part
            └─ Mesh
```

따라서 Mesh의 최종 위치는 다음 Transform의 누적으로 결정된다.

```text
Mmesh =
MLink2
× MVisual
× MAssembly
× MPart
× MMesh
```

런타임에서 이 계층을 모두 유지하는 대신, 관절 아래의 Visual 계층 Transform을 Mesh Vertex에 미리 반영했다.

즉 다음 구조를

```text
Link2
└─ Visual
   └─ Assembly
      └─ Part
         └─ Mesh
```

다음과 같이 변경했다.

```text
Link2
├─ Link2Mesh
└─ J3
```

CAD 계층에 포함되어 있던 Transform은 Vertex 좌표에 Bake했기 때문에 런타임에서는 Link와 Joint Transform만 처리하면 된다.

---

## 6. 관절 Transform 보존

J1~J6의 기존 Local Transform은 제거하지 않았다.

원본 GLB에 정의된 관절 Transform은 단순한 초기 자세가 아니라 각 Link의 좌표계를 정의하는 기준 Transform이기 때문이다.

따라서 런타임에서는 관절 Transform을 덮어쓰는 방식이 아니라 기준 Transform에 관절 회전을 추가하는 방식으로 처리하도록 했다.

개념적으로 다음 형태이다.

```text
JointLocal =
BindTransform
× JointRotation
```

이를 통해 GLB에서 정의된 관절 위치와 축 기준을 유지하면서 FK 또는 IK를 통해 계산된 관절 각도를 추가할 수 있도록 했다.

---

## 7. Mesh 단위 병합

1차 정규화에서는 CAD Node들을 제거하고 각 Link의 형상을 하나의 Mesh Node 아래로 모았다.

정규화 결과는 다음과 같다.

```text
기존
168 Nodes
128 Meshes

↓

1차 정규화

27 Nodes
10 Meshes
```

이 단계에서 로봇의 논리 계층은 단순해졌지만 Mesh 내부에는 여전히 다수의 Primitive가 남아 있었다.

---

## 8. Primitive 병합

정규화 이후 Mesh 내부를 확인한 결과 총 8,625개의 Primitive가 존재했다.

이는 CAD 변환 과정에서 작은 형상 단위가 각각 Primitive로 분리된 결과였다.

렌더링 시 Primitive마다 별도의 처리 과정이 발생하기 때문에 동일 Material을 사용하는 Primitive끼리 Vertex와 Index Buffer를 병합했다.

병합 기준은 다음과 같다.

```text
동일 Link
+
동일 Material
=
하나의 Primitive로 병합
```

다만 서로 다른 관절에 속하는 Mesh는 병합하지 않았다.

예를 들어 다음 병합은 허용하지 않았다.

```text
Link2Mesh
+
Link3Mesh
```

이 경우 J3 회전 시 두 형상을 독립적으로 움직일 수 없기 때문이다.

---

## 9. 최종 결과

Primitive 병합 후 모델 구조는 다음과 같이 변경됐다.

| 항목        |  원본/중간 구조 |     최종 구조 |
| --------- | --------: | --------: |
| Node      |       168 |        27 |
| Mesh      |       128 |        10 |
| Primitive |     8,625 |        40 |
| Vertex    |   257,727 |   257,727 |
| Index     |   913,980 |   913,980 |
| 파일 크기     | 약 18.2 MB | 약 8.05 MB |

Primitive 수는 약 99.5% 감소했다.

Vertex와 Index의 총 개수는 유지했기 때문에 형상의 단순화나 Polygon 제거를 수행한 것은 아니다. 구조와 Buffer 배치만 런타임에 적합하게 변경했다.

---

## 10. 검증

정규화 과정에서 형상이 변경되지 않았는지 확인하기 위해 변환 전후의 World-space Bounding Box를 비교했다.

변환 전:

```text
min = [-0.201087968, 0, -0.189019069]
max = [ 0.296439128, 1.09120760, 1.06359168]
```

변환 후:

```text
min = [-0.201087967, 0, -0.189019069]
max = [ 0.296439128, 1.09120760, 1.06359168]
```

Floating-point 오차 수준을 제외하면 동일한 범위를 유지했다.

추가로 다음 항목을 확인했다.

* Vertex 개수 유지
* Index 개수 유지
* Index 범위 유효성 확인
* J1~J6 계층 유지
* Gripper 계층 유지
* Left/Right Finger 독립 Node 유지
* 기본 자세의 전체 형상 위치 유지

---

## 11. 런타임 처리 방식

정규화 이후 로봇 제어 코드에서는 CAD 내부 구조를 알 필요가 없다.

필요한 Node만 이름으로 조회한다.

```cpp
J1
J2
J3
J4
J5
J6

LeftFingerJoint
RightFingerJoint
```

렌더링에서는 각 Link에 연결된 Mesh를 부모 Transform에 따라 렌더링하고, 관절 제어에서는 J1~J6의 Local Transform만 변경한다.

따라서 로봇 제어 계층과 시각적 Mesh 구조를 분리할 수 있다.

```text
Robot Control

J1
 ↓
J2
 ↓
J3
 ↓
J4
 ↓
J5
 ↓
J6
 ↓
Gripper
```

```text
Rendering

BaseMesh
Link1Mesh
Link2Mesh
Link3Mesh
Link4Mesh
Link5Mesh
Link6Mesh
GripperMesh
FingerMesh
```

---

## 12. 설계 결과

이번 작업에서는 복잡한 입력 데이터를 모두 런타임 로직에서 처리하는 대신, 입력 자산을 프로젝트 목적에 맞는 중간 표현으로 정규화했다.

이를 통해 TinyGLTF는 GLB의 범용 CAD 구조를 해석하는 역할에서 벗어나 다음 역할만 수행하도록 단순화할 수 있었다.

```text
GLB Load
→ Robot Node 탐색
→ Mesh Buffer 생성
→ Joint Transform 갱신
→ Render
```

결과적으로 3D 자산의 내부 구조와 로봇 제어 코드 간 결합을 줄였고, 이후 FK·IK·Grasp와 같은 로봇 기능은 CAD 모델의 세부 Node 구조와 관계없이 J1~J6 기준으로 구현할 수 있는 구조를 만들었다.
