# Physics와 Flecs 연결

이 문서는 장면의 Entity가 Jolt 물체가 되는 과정, 그리고 두 쪽의 위치가 매 물리 tick마다 어떻게 맞춰지는지를 설명한다. 모듈마다 책임을 나눠 둔 이유는 PhysicsWorld가 장면이나 렌더러에 얽매이지 않게 하기 위해서다.

## 책임 경계

| 영역 | 책임 |
| --- | --- |
| `modules/physics` | Jolt 초기화, Body·접촉 snapshot·constraint 수명, 고정 스텝, pose 조회·변경 |
| `modules/robotics` | Robot/Gripper Controller 상태, 팔 FK·DLS IK·직선 이동, 그리퍼 master/mimic Local 회전 계산 |
| `modules/scene` | Flecs Entity·Scene과 Local/World Transform 계산 |
| `modules/model` | GLB를 읽은 CPU 측 모델 데이터 |
| `modules/viewer` | OpenGL 창·렌더링, GPU 자산과 화면 표현 |
| `modules/simulation` | 물리 설정 컴포넌트, Entity와 Body 연결, 로봇 충돌 프록시, 시뮬레이션 Scene 구성 |
| `ViewerApp` | 모듈을 만들고 실행 순서를 연결하는 Composition Root |

`PhysicsWorld`는 Flecs와 OpenGL을 모른다. `PhysicsSystemModule`은 `modules/simulation`에서 Flecs 설정을 Jolt Body로 연결한다. `PhysicsBodyHandle`은 Physics API 내부의 불투명 핸들이며, 앱과 Entity 생성 코드는 이를 보관하지 않는다.

이 문서는 fixed tick, Transform 좌표계와 physics-body 수명 세부 계약을 다룬다. 상위 모듈 의존성은 [Architecture](ARCHITECTURE.md), 후보 self-collision 허용 쌍은 [Self-Collision](SELF_COLLISION.md), Controller API 상태는 [Controller Interface](CONTROLLER_INTERFACE.md)를 참고한다.

물리 객체는 Entity에 설정 컴포넌트를 붙여 선언한다.

```cpp
entity.set<RigidBody>(RigidBody{BodyMotionType::Dynamic})
    .set<Colliders>(Colliders{{physics_colliders::Box({0.06F, 0.06F, 0.06F})}});
```

`Colliders`는 Box 또는 Convex Hull 모양을 하나의 Entity/rigid body에 하나 이상 묶는다. Box는 바닥처럼 크기와 위치가 정해진 단순 형상에 쓰고, Convex Hull은 로봇 link와 그리퍼처럼 GLB 정점에서 만든 볼록 형상에 쓴다. `PhysicsSystemModule`은 두 설정이 모두 있는 Entity의 Body를 만든다. 설정이 바뀌거나 제거되면 기존 Body를 정리한다. Entity가 사라질 때 Flecs가 private binding component를 제거하고, observer가 연결된 Body를 삭제한다.

## 변환과 물리 좌표

`TransformSystemModule::UpdateWorldTransforms()`가 Entity 계층의 Local/World 행렬을 계산하는 단일 경로다. Scene-driven Static·Kinematic은 이 결과를 Jolt에 보낸다. Physics-driven Dynamic은 SceneRoot 또는 항등 grouping 조상만 허용해 World≈Local로 다룬다. Jolt의 World 위치·회전을 Local에 직접 기록하므로 부모 역행렬이나 Local 행렬 분해를 사용하지 않는다. 조상 각각이 항등이어야 하며 서로 상쇄되는 부모 변환도 허용하지 않는다.

Entity의 `Rotation, Local`은 단위 `glm::quat`이다. FK quaternion은 정규화해 Kinematic Entity에 직접 저장하며, Dynamic 결과도 Jolt quaternion을 직접 저장한다. Euler 왕복 변환은 없다. `Rotation`과 `ComposeLocalMatrix`는 영 quaternion·NaN 등 비유한 회전을 거부하고 단위 정규화한다. `q`와 `-q`는 같은 자세다. GLB 회전 배열 `[x,y,z,w]`는 로더에서 GLM 생성자 `(w,x,y,z)`로 변환한다. Entity pose와 행렬의 float 정밀도는 유지하며, 제어 관절각은 rad 스칼라로 전달한다.

Physics가 있는 Entity와 모든 조상 Entity는 unit scale이어야 한다. 단, Static Environment collider는 Entity 자신의 시각 scale을 허용한다. Floor Mesh의 authored scale은 Render 표현에만 쓰고, collider 크기와 offset은 meter 단위로 직접 지정한다. Jolt rigid body pose에는 scale이 포함되지 않는다.

4 ms 고정 tick은 아래 순서로 진행된다. `PhysicsSystemModule::Step()` 안에서 PrePhysicsSync, Jolt step, PostPhysicsSync가 이어진다. 충돌 guard는 Controller 갱신과 FK 적용 뒤, physics step 전에 현재 후보 자세를 검사한다.

```text
GripperGraspAdapter::BeforePhysicsStep: 이전 파지 해제 / Body handle 수명 확인
→ RobotCollisionGuard::CaptureSafeJointPose
→ RobotController / GripperController Update
→ RobotKinematics: RobotState → joint rotation / link pose
→ RobotTransformAdapter: joint rotation → GLB Joint Entity
→ RobotPhysicsAdapter: link pose → Kinematic collision Entity
→ GripperKinematics: 연속 closureFraction → master/mimic Local 회전
→ GripperTransformAdapter: bind 회전과 결합 → GLB 관절 Local 회전
→ TransformSystemModule: Local → World matrix
→ RobotCollisionGuard::RestoreSafePoseIfOverlapping: 후보 자세 검사; 겹치면 직전 안전 관절각으로 복원하고 오류 code 기록 및 Controller Idle 전환
→ PhysicsSystemModule::Step
   ├─ PrePhysicsSync: Body 설정 재구성 / Static·Kinematic Scene 목표 → Jolt
   ├─ PhysicsWorld Step
   └─ PostPhysicsSync: Dynamic Jolt pose → Entity Local 위치·회전 직접 저장
→ GripperGraspAdapter::AfterPhysicsStep: 접촉 상태 / 개폐 정지 / 같은 물체 양쪽 파지 검사
→ TransformSystemModule: World matrix 재갱신
```

화면 갱신 전에도 `TransformSystemModule::UpdateWorldTransforms()`가 실행된다. 따라서 RenderSystem은 최신 Local Transform으로 계산한 World 행렬을 읽으며, Physics는 render FPS와 독립된 fixed step에서만 진행한다.

`IsSceneDriven`은 Static·Kinematic, `IsPhysicsDriven`은 Dynamic을 분류한다. Static은 Scene World 목표가 변경될 때만 `SetBodyTransform`으로 배치한다. Kinematic은 목표가 같아도 실제 도달 전에는 `MoveKinematic`을 계속해 이동 속도를 만든다. 도달 후 `StopKinematic`을 한 번 호출해 Step 뒤 남는 속도를 제거하고, 이후 같은 목표의 자세 전달은 생략한다. 도달 판정은 위치 1 μm·quaternion 성분 1e-6 이내이며 quaternion 부호 차이를 허용한다. Dynamic은 PostPhysicsSync에서 Jolt 결과만 읽어 Local 위치·회전을 교체하고 Scale은 유지한다.

## 로봇 자세와 충돌 프록시

`RobotKinematics`는 앞 관절까지 누적한 회전을 반영해 bind pivot의 위치를 계산하고, 관절 축과 Controller 각도로 각 링크의 회전과 로봇 기준 위치를 구한다. IK는 TCP 목표를 풀 때 ToolFrame에 고정된 tool 변환을 적용한다. `MovePose`는 현재 관절각을 시작값으로 Damped Least Squares IK를 풀고 결과를 `MoveJoint`에 전달한다. `MoveLinear`는 TCP의 직선 위치와 quaternion 최단 회전 경로를 표본으로 나누어 실행 전에 IK를 계산한 뒤, 저장한 관절 표본을 고정 제어 주기마다 보간한다. IK 반복과 관절 경로 검사는 `LinearPathPlanningJob`이 작은 작업 단위로 나누며, Viewer는 렌더 루프에서 제한된 시간 동안 이를 진행한다. 이때 Jolt와 ECS를 작업자 스레드에서 읽지 않도록 시뮬레이션 갱신을 잠시 멈추고 화면 렌더링은 계속한다. 실행 중에는 설정 속도를 넘는 TCP 추종 오차가 생기는 특이 자세에서만 IK를 다시 계산한다. ToolFrame이 있으면 Controller는 모델에서 계산한 `tcpPoseValid=true`를 보고하지만, 이는 하드웨어 측정값이 아니다. Viewer adapter는 GLB 계층에 관절 회전을 적용하고 Simulation adapter는 Robot Base의 고정 Environment collider를 포함해 별도 Kinematic 충돌 Entity를 갱신한다. 렌더 메시와 Jolt body ID는 서로 별도로 유지한다.

HCR-12A collider는 GLB 재질 메시의 삼각형 연결로 나눈 부품별로 생성한다. 같은 위치의 seam 정점도 연결해 부품을 판별한다. 4 cm 미만 부품은 제외하고, 나머지는 삼각형 중심을 관절 좌표계 기준 16 cm 셀로 묶는다. 삼각형은 셀 경계에서 자르지 않는다. 1 cm 미만 크기 셀과 부피가 없는 hull 입력도 제외한다. 각 셀의 Convex Hull은 오목한 부분이나 셀 경계 사이를 메울 수 있다. 다음 가동 관절 아래와 `Gripper` geometry는 이 adapter의 hull 생성 대상이 아니다.

`ConfigureTwoF85Colliders`는 고정 `GripperMesh`, outer knuckle+finger compound, inner knuckle, fingertip을 각각 한 mesh 기반 Convex Hull로 만들어 총 일곱 Kinematic proxy로 둔다. 정점은 owning authored Gripper/joint 원점 기준이며, 해당 proxy는 원본 joint Entity의 child라 fixed-step World transform에서 자동으로 따라간다. 개폐 자세의 기준은 `SimGripperController`의 유효한 연속 `GripperState.closureFraction`이다. `GripperKinematics`가 master/mimic Local 회전을 계산하고 `GripperTransformAdapter`가 bind 회전과 결합해 원본 관절에 적용한다. 시작 형상은 약 85 mm open gap을 보존한다. `GripperGraspAdapter`가 손끝 접촉으로 개폐를 멈추고 양쪽 접촉이 같은 Dynamic 물체를 반대 방향에서 향하면 본체에 고정 constraint를 연결한다. 실제 접촉력, 개별 손가락 적응과 torque 동역학은 계산하지 않는다. 매핑과 속도의 시뮬레이션 가정은 [그리퍼 런타임 설계](GRIPPER_RUNTIME_DESIGN.md)를 참고한다.

`Robot`과 `Gripper`는 Environment 및 DynamicObject와 충돌한다. Robot 링크끼리, Gripper part끼리, Robot–Gripper 사이 충돌은 제외해 현재 프록시의 자기 충돌을 줄인다. 부착 상태별 필터는 없다.

현재 로봇 물리는 controller가 계산한 자세를 따르는 Kinematic 충돌 프록시다. Jolt Kinematic Body는 목표를 장애물 앞에서 막지 않는다. Viewer는 매 고정 갱신에서 Robot과 Gripper의 목표 충돌 형상을 Jolt 형상 검사로 Environment와 비교하고 겹치면 물리 목표를 보내기 전에 직전 안전 관절각으로 복원한다. 이는 각 고정 갱신의 목표 자세를 검사하며 전체 이동 경로를 미리 계획해 장애물을 돌아가지는 않는다. 로봇 관절 torque, 관성, 동역학 기반 grasp는 구현하지 않았다.

## 바닥과 디버그 객체

Floor의 `plane.glb` Mesh와 Static Box Collider는 하나의 model root Entity가 소유한다. GLB 평면은 root scale 3으로 X/Z ±3 m 범위이며, Collider도 half-extents `{3, 0.02, 3}` m로 맞춘다. Collider 윗면은 수치 오차 방지를 위해 시각 평면보다 5 mm 위에 둔다. GLB를 Jolt triangle mesh로 변환하지 않는다.

`apps/simulator`의 `PickPlaceScenario`는 미션에서 사용할 상자와 더 넓은 배치 목표의 시각·물리 Entity를 만든다. 위치와 yaw 회전 표본은 `grasplink::simulation::scenario`에서 생성하며, 같은 seed와 함수 호출 순서를 주면 같은 표본을 다시 만들 수 있다. 별도의 50개 낙하 상자 데모나 자동 관절 동작 옵션은 없다.

`GuiModule`은 ImGui context·GLFW/OpenGL backend와 프레임·입력 capture 수명을 관리한다. `ViewerApp`이 `BeginFrame`과 `EndFrame` 사이에 `GripperPanel`, `PhysicsDebugPanel`, `ColliderOverlay`를 명시적으로 호출한다. `PhysicsDebugPanel`은 `Show configured colliders` 체크박스 상태를 보관하고 `ColliderOverlay`가 ECS shape의 Box 외곽선과 Convex Hull 투영을 layer별 색으로 표시한다. Overlay는 World query와 화면 선 캐시를 소유하며 Camera는 Draw 동안만 빌린다. 첫 표시·resize 때 즉시 갱신하고 이후 100 ms 간격으로 캐시를 갱신한다. Jolt 내부 shape를 조회하거나 깊이를 검사하지 않으며 Camera 이동도 다음 갱신 전까지 이전 투영으로 보일 수 있다.

`PrefabFactory`는 primitive가 하나인 Node의 Mesh component를 Node Entity에 직접 붙인다. 여러 primitive인 경우에만 primitive별 Render child Entity를 만든다.

## 객체 수명과 확장 범위

`ViewerApp`이 `PhysicsWorld`를 소유하고 `PhysicsSystemModule` 및 어댑터보다 먼저 생성한다. `Scene`은 실행 중 하나만 있고 생성될 때 SceneRoot를 만들며 파괴될 때 자식 계층을 제거한다. 종료할 때 Overlay의 World query와 패널, 어댑터와 Scene Entity를 먼저 정리하고, `PhysicsSystemModule`을 해제한 다음 Flecs World와 `PhysicsWorld`를 파괴한다. Flecs Entity의 제거 observer가 Physics Body도 삭제한다. `GuiModule` backend와 `AssetManager`의 GPU Mesh 참조도 OpenGL Context를 정리하기 전에 해제한다. `ModelResource`는 CPU 모델 데이터만 보관한다.

Collider는 하나의 Body 안에 Box와 Convex Hull을 여러 개 둘 수 있다. Collider mass properties와 center-of-mass 처리는 Jolt 내부 책임이며, 공개 API는 ECS Entity와 같은 Body 원점의 pose를 주고받는다. Collider 설정 변경은 pending Entity 목록으로 모아 다음 PrePhysicsSync에서 한 번 반영한다. 모든 Body의 Dynamic 조상은 금지하며 Dynamic 자체의 비항등 조상도 지원하지 않는다. 실행 중 scale·부모 계약이 깨지면 연결 Body를 제거하고, 계약이 복구되면 현재 Scene World 자세로 재생성한다.

## 다음 구현 단계

1. 접촉력과 마찰을 이용하는 그리퍼 적응 및 파지 안정성 계산을 검토한다.
2. 충돌을 미리 예측해 지면과 물체를 피해 가는 로봇 경로 계획을 검토한다.
3. 필요성이 확인되면 joint constraint와 관절 동역학을 추가한다.
4. 물리 proxy를 SceneRoot 아래 평평한 계층으로 두는 구조는 장기 검토 사항이며 아직 구현하지 않았다. 현재 팔·그리퍼 proxy의 부모 계층과 API는 유지한다.
