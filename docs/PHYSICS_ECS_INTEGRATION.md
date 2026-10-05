# Physics / Flecs Integration

## 책임 경계

MiniBCG는 3D rigid-body simulation에 Jolt를 사용한다. Flecs는 Entity와 Component를 관리하는 ECS이며,
프로젝트의 Physics simulation을 대신하지 않는다.

| 영역 | 책임 |
| --- | --- |
| `modules/physics` | Jolt 초기화, rigid body 생성·삭제, simulation step, pose 조회·변경 |
| Viewer의 `PhysicsSystemModule` | Physics 설정 Component 해석, Entity와 Body 연결, 좌표 변환, Entity 제거 시 Body 정리 |
| `EntityFactory` | 필요한 Entity와 Render/Physics 설정 Component 구성 |
| `ViewerApp` | Window, Scene, Robot, Physics 객체의 생성 순서와 fixed-step 호출 연결 |

`PhysicsWorld`와 `PhysicsSystemModule`은 코드 중복을 나눈 두 구현이 아니다. `PhysicsWorld`는 Flecs를
모르는 Jolt 경계이며, `PhysicsSystemModule`은 Jolt API를 Entity의 Component contract에 연결한다.
둘을 합치면 Physics 모듈이 ECS에 의존하거나 ViewerApp이 Body 생성, lifetime, 좌표 동기화를 직접
구현해야 한다.

`PhysicsWorld`의 public API는 프로젝트 소유 `PhysicsBodyHandle`과 GLM pose/config를 사용한다. Jolt의
`JPH::BodyID`와 다른 Jolt 타입은 `modules/physics` 구현 안에만 둔다. Flecs와 OpenGL은 이 모듈의
dependency가 아니다.

## Entity physics 설정

사용 코드는 Body를 직접 만들지 않고 Entity에 설정을 붙인다.

```cpp
entity.set<RigidBody>(RigidBody{BodyMotionType::Dynamic})
      .set<BoxCollider>(BoxCollider{{0.5F, 0.5F, 0.5F}});
```

`RigidBody`는 Body의 motion type을 선언한다.

- `Static`: 생성한 World pose를 Jolt가 고정한다.
- `Kinematic`: Entity Transform이 pose를 결정하고 매 fixed step Jolt에 목표 pose를 전달한다.
- `Dynamic`: Jolt가 pose를 계산하고 매 fixed step 결과를 Entity에 반영한다.

`BoxCollider::halfExtentsMeters`는 collider 각 축의 반 크기 [m]다. `localPositionMeters`와
`localRotation`은 Entity 기준 collider 위치와 회전이다. Entity scale은 collider 크기에 자동 곱하지
않는다. Dynamic object와 Robot Link는 unit scale을 쓴다. Floor visual은 GLB의 scale을 유지하고,
collider는 별도로 지정한 World meter 크기를 사용한다.

integration 생성 시 기존 설정 Entity를 한 번 처리하고, 이후 두 설정 Component가 모두 존재하거나
갱신되면 observer가 `PhysicsWorld::CreateBox()`를 호출한다. 설정이 바뀌면
기존 binding을 제거해 Body를 정리한 뒤 새 설정으로 다시 만든다. 둘 중 하나가 제거되면 같은 Entity의
private `PhysicsBodyBinding`도 제거되고, binding 제거 observer가 실제 Jolt Body를 삭제한다. Entity가
destroy될 때 Flecs의 Component 제거 event도 같은 정리 경로를 사용한다.

`PhysicsBodyBinding`은 integration source 안에서만 정의한다. 게임 코드와 Factory는 handle을 보관하거나
Body lifetime을 수동으로 관리하지 않는다. PhysicsWorld가 예기치 않게 handle을 무효화한 경우 다음 fixed
step의 validity 확인에서 설정 Component를 읽어 Body를 다시 만든다.

## 좌표계와 fixed-step 순서

Jolt는 World pose를 사용하고 Scene Entity는 부모 기준 Local Transform을 사용한다. 따라서 두 방향을
같은 값 대입으로 처리하지 않는다.

1. Kinematic Entity의 최신 Local TRS와 parent hierarchy를 누적해 World pose를 만든다.
2. Entity-local collider 위치·회전을 더한 pose를 Jolt에 전달한다.
3. `PhysicsWorld::Step(fixedDeltaSeconds)`를 한 번 실행한다.
4. Dynamic Body의 World pose에서 collider-local transform을 되돌린다.
5. 부모 World matrix의 역행렬을 적용해 Entity Local position/rotation에 기록한다.
6. 다음 render frame의 `TransformSystemModule`이 변경된 Local 값으로 World matrix를 계산하고 Renderer가 읽는다.

고정 실행 순서:

```text
FixedControlLoop
→ RobotController Update
→ RobotTransformAdapter (RobotState → Joint Local Transform)
→ PhysicsSystemModule (Kinematic Entity → Physics)
→ PhysicsWorld Step
→ PhysicsSystemModule (Dynamic Physics → Entity Local Transform)
→ Render frame: TransformSystem / RenderSystem
```

Physics가 쓰는 계층 행렬은 `TransformSystemModule::CalculateWorldMatrix()`에서 현재 Local Component를
읽어 계산한다. 이전 Render Frame의 cached World matrix를 사용하지 않으므로, 그 fixed tick에서 Controller가
바꾼 로봇 관절도 Physics 목표 pose에 반영된다. 동일한 `ComposeLocalMatrix()`가 Renderer용 TransformSystem과
Physics 양쪽에서 쓰이므로 Local `T * R * S` 규칙이 따로 복제되지 않는다.

Dynamic Body를 scale 또는 shear가 있는 부모 아래 두는 것은 아직 지원 범위가 아니다. Jolt rigid body pose에
scale이 없고 현재 동기화가 위치·회전만 Local Component에 되돌린다.

## 현재 Scene 구성

### Floor

`plane.glb`가 단일 primitive인 경우 PrefabFactory가 MeshFilter와 MeshRenderer를 model root Entity에 붙인다.
같은 Entity에 Factory가 `Static RigidBody`와 단순 `BoxCollider`를 붙인다. 시각 Mesh와 Collider는 하나의
논리적 Floor Entity가 소유하지만 shape 자체는 서로 다른 표현이다. GLB Mesh를 Jolt triangle mesh로 변환하지
않는다. 현재 visible plane의 local 반 폭은 scale 적용 후 약 3 m이고 Collider 반 폭은 5 m다. Collider는
mesh보다 넓게 두며 높이 0.2 m, local y offset -0.1 m로 놓아 윗면을 바닥면 y=0에 맞춘다.

### Debug Box

`Mesh::CreateCube()`는 각 축 -0.5에서 +0.5까지의 unit cube를 만든다. Debug build의 Entity는 크기 배율 1과
half-extents `{0.5, 0.5, 0.5}`를 쓰므로 Render Cube와 Physics Box의 nominal 크기가 일치한다. 같은 Entity가
MeshFilter, MeshRenderer, Material/Shader, Transform, Dynamic Body와 collider 설정을 가진다.

### Robot Link

HCR-12A의 J1~J6 Joint Entity에 Kinematic Box Collider를 붙인다. 이 proxy는 Controller의 관절 hierarchy를
따라 이동하고 환경 collision에 참여한다. 현재 proxy는 link 형상의 근사치다. Jolt joint constraint, 모터
토크 기반 articulated dynamics, self-collision policy는 구현하지 않았다.

2F-85 Gripper의 controller spec과 GLB linkage는 있지만, 현재 Physics Component는 Gripper joints에 붙이지
않는다. Gripper collision, object contact, under-actuated adaptation과 grasp는 이후 단계다.

## Lifetime

`ViewerApp`이 `PhysicsWorld`를 먼저 만들고 이를 참조하는 `PhysicsSystemModule`을 만든다. Shutdown에서는
Scene을 제거하고 integration을 해제한 뒤 Flecs World를 비운다. 모든 Entity binding의 Body가 제거되는 동안
`PhysicsWorld`가 살아 있고, 마지막에 `PhysicsWorld`를 파괴한다.

`PhysicsSystemModule`은 Flecs World와 PhysicsWorld를 non-owning reference로 보관한다. 둘 다 integration보다
오래 살아야 한다. 지금 단계에서는 Flecs Entity와 Body의 양방향 lifetime observer를 두어 수명을 맞춘다.
새로운 Physics client가 생기면 이 소유 순서와 Entity 삭제 경로를 함께 갱신해야 한다.

## 다음 확장 순서

1. HCR 링크 Collider 치수와 축 정렬을 model/asset 기준으로 개선하고 self-collision 정책을 정한다.
2. 2F-85 Simulation controller가 여섯 linkage Joint의 free-space motion을 만들게 한다.
3. Gripper link Collider와 joint constraint를 추가한다.
4. Object contact를 측정해 under-actuated adaptation과 grasp 상태를 구현한다.
5. Dynamic Body와 scale이 있는 부모를 지원해야 할 때 collider scale 변환을 별도 설계한다.
