# Physics / Flecs Integration

## 책임 경계

| 영역 | 책임 |
| --- | --- |
| `modules/physics` | Jolt 초기화, Body 생성·삭제, 고정 스텝, pose 조회·변경 |
| `modules/robotics` | RobotState와 모델 명세를 사용한 순기구학 계산 |
| `modules/viewer` | Flecs Scene, 화면 Transform, GLB·OpenGL 표현 |
| `modules/simulation` | 물리 설정 컴포넌트, Entity와 Body 연결, 로봇 충돌 프록시, 시뮬레이션 Scene 구성 |
| `ViewerApp` | 모듈을 만들고 실행 순서를 연결하는 Composition Root |

`PhysicsWorld`는 Flecs와 OpenGL을 모른다. `PhysicsSystemModule`은 `modules/simulation`에서 Flecs 설정을 Jolt Body로 연결한다. `PhysicsBodyHandle`은 Physics API 내부의 불투명 핸들이며, 앱과 Entity 생성 코드는 이를 보관하지 않는다.

물리 객체는 Entity에 설정 컴포넌트를 붙여 선언한다.

```cpp
entity.set<RigidBody>(RigidBody{BodyMotionType::Dynamic})
    .set<Colliders>(Colliders{{physics_colliders::Box({0.06F, 0.06F, 0.06F})}});
```

`Colliders`는 Box, Cylinder, Sphere, Convex Hull 여러 개를 하나의 Entity/rigid body에 묶는다. `PhysicsSystemModule`은 두 설정이 모두 있는 Entity의 Body를 만든다. 설정이 바뀌거나 제거되면 기존 Body를 정리한다. Entity가 사라질 때 Flecs가 private binding component를 제거하고, observer가 연결된 Body를 삭제한다.

## Transform과 물리 좌표

`TransformSystemModule::UpdateWorldTransforms()`가 Entity 계층의 Local/World 행렬을 계산하는 단일 경로다. Physics는 이 결과를 읽어 Jolt에 보낸다. Dynamic Body의 Jolt pose는 Entity 부모의 cached World 행렬 역행렬을 적용해 Local Transform으로 되돌린다.

Physics가 있는 Entity와 모든 조상 Entity는 unit scale이어야 한다. 단, Static Environment collider는 Entity 자신의 시각 scale을 허용한다. Floor Mesh의 authored scale은 Render 표현에만 쓰고, collider 크기와 offset은 meter 단위로 직접 지정한다. Jolt rigid body pose에는 scale이 포함되지 않는다.

고정 스텝 흐름은 다음과 같다.

```text
RobotController Update
→ RobotKinematics: RobotState → joint rotation / link pose
→ RobotTransformAdapter: joint rotation → GLB Joint Entity
→ RobotPhysicsAdapter: link pose → Kinematic collision Entity
→ TransformSystemModule: Local → World matrix
→ PhysicsSystemModule: Kinematic Entity → Jolt
→ PhysicsWorld Step
→ PhysicsSystemModule: Dynamic Jolt pose → Entity Local Transform
```

화면 갱신 전에도 `TransformSystemModule::UpdateWorldTransforms()`가 실행된다. 따라서 RenderSystem은 최신 Local Transform으로 계산한 World 행렬을 읽으며, Physics는 render FPS와 독립된 fixed step에서만 진행한다.

## Robot pose와 충돌 프록시

`RobotKinematics`는 모델의 bind pivot 사이 차이를 이전 누적 회전으로 변환하고, joint axis와 controller 관절각으로 joint 회전 및 base-frame link pose를 계산한다. bind pivot은 계산 입력에 쓰인다. FK의 ToolFrame 결과는 controller의 `tcpPose` feedback과 별개이며, 현재 `SimRobotController`의 `tcpPoseValid`는 false다. Viewer 어댑터는 joint 회전을 GLB hierarchy에 적용하고, Simulation 어댑터는 link pose를 별도 Kinematic collision Entity에 적용한다. 화면 Mesh Entity와 Jolt Body의 ID를 같은 것으로 취급하지 않는다.

HCR-12A collider는 GLB 재질 메시의 삼각형 연결로 나눈 부품별로 생성한다. 같은 위치의 seam 정점도 연결해 부품을 판별한다. 4 cm 미만 부품은 제외하고, 나머지는 삼각형 중심을 관절 좌표계 기준 16 cm 셀로 묶는다. 삼각형은 셀 경계에서 자르지 않는다. 1 cm 미만 크기 셀과 부피가 없는 hull 입력도 제외한다. 각 셀의 Convex Hull은 오목한 부분이나 셀 경계 사이를 메울 수 있다. 다음 가동 관절 아래와 `Gripper` geometry는 이 adapter의 hull 생성 대상이 아니다.

`ConfigureTwoF85Colliders`는 고정 `GripperMesh`, outer knuckle+finger compound, inner knuckle, fingertip을 각각 한 mesh 기반 Convex Hull로 만들어 총 일곱 Kinematic proxy로 둔다. 정점은 owning authored Gripper/joint 원점 기준이며, 해당 proxy는 원본 joint Entity의 child라 fixed-step World transform에서 자동으로 따라간다. 현재 authored ECS joint가 pose source of truth다. 시작 형상은 약 85 mm open gap을 보존한다. 이 단계는 controller나 grasp 동역학을 공급하지 않는다.

`Robot`과 `Gripper`는 Environment 및 DynamicObject와 충돌한다. Robot 링크끼리, Gripper part끼리, Robot–Gripper 사이 충돌은 제외해 현재 프록시의 자기 충돌을 줄인다. 부착 상태별 필터는 없다.

현재 로봇 물리는 controller가 계산한 자세를 따르는 Kinematic 충돌 프록시다. Jolt joint constraint, 관절 torque, 관성, 동역학 기반 grasp는 구현하지 않았다.

## Floor와 디버그 객체

Floor의 `plane.glb` Mesh와 Static Box Collider는 하나의 model root Entity가 소유한다. GLB 평면은 root scale 3으로 X/Z ±3 m 범위이며, Collider도 half-extents `{3, 0.02, 3}` m로 맞춘다. Collider 윗면은 수치 오차 방지를 위해 시각 평면보다 5 mm 위에 둔다. GLB를 Jolt triangle mesh로 변환하지 않는다.

`apps/viewer`의 `DebugSceneSetup`은 0.12 m Cube Mesh와 Material을 50개 Entity가 공유하는 물리 데모 객체를 만든다. Box Collider half-extents는 0.06 m다. `--physics-demo`로 활성화할 수 있으며 모든 build type에서 선택 사항이다. 객체 생성은 앱의 데모 설정에 있고, 기본 Simulation Scene의 Floor 설정과는 분리돼 있다.

`modules/gui`의 `GuiModule`이 ImGui context·입력·panel 수명을 관리한다. `Show configured colliders`가 켜졌을 때 ECS에 설정된 Collider shape의 Box/Cylinder/Sphere 외곽선과 Convex Hull 투영을 layer별 색으로 표시한다. 선은 100 ms 간격으로 캐시되며 Jolt 내부 shape를 조회하거나 깊이를 검사하지 않는다. Camera가 이동해도 다음 갱신 전까지 이전 투영이 보일 수 있다.

`PrefabFactory`는 primitive가 하나인 Node의 Mesh component를 Node Entity에 직접 붙인다. 여러 primitive인 경우에만 primitive별 Render child Entity를 만든다.

## Lifetime과 확장 범위

`ViewerApp`이 `PhysicsWorld`를 소유하고 `PhysicsSystemModule` 및 어댑터보다 먼저 생성한다. Scene 전환은 이전 Scene의 `OnExit`와 root 정리 후 새 root를 만들고 `OnEnter`를 호출한다. 종료할 때 어댑터와 Scene Entity를 먼저 정리하고, `PhysicsSystemModule`을 해제한 다음 Flecs World와 `PhysicsWorld`를 파괴한다. Flecs Entity의 제거 observer가 Physics Body도 삭제한다. `ModelResource`와 Entity가 공유하는 GPU Mesh 참조도 OpenGL Context를 정리하기 전에 해제한다.

Collider는 하나의 Body 안에 Box/Cylinder/Sphere/Convex Hull을 여러 개 둘 수 있다. Collider mass properties와 center-of-mass 처리는 Jolt 내부 책임이며, 공개 API는 ECS Entity와 같은 Body 원점의 pose를 주고받는다. Collider 설정 변경은 pending Entity 목록으로 모아 다음 fixed step에서 한 번 반영한다. Dynamic Body의 조상에 Dynamic Body가 있으면 해당 물리 binding을 만들지 않는다.

## 다음 구현 단계

1. 충돌 wireframe과 접촉 결과를 보며 asset 기반 hull을 검증한다.
2. Gripper controller와 고정 tick pose 공급을 연결한다. 현재는 authored ECS joint가 proxy transform의 source of truth다.
3. 접촉 결과를 읽고 grasp 상태를 판정한다.
4. 필요성이 확인되면 joint constraint와 관절 동역학을 추가한다.
