# Why GraspLink Uses Flecs

## 1. 도입 배경

초기 Viewer에서는 `Renderer`가 Cube resource와 draw call을 직접 관리해도 충분했다.

하지만 로봇 시뮬레이터에는 다음 객체가 생긴다.

```text
Target Object
Robot Base
Robot Links
End Effector
Gripper
Debug Axis
```

Renderer가 각 객체의 수명과 상태까지 직접 관리하면 rendering code와 scene-state 관리가 섞인다. 그래서 GraspLink에서는 **scene object 관리와 반복 처리 규칙만 Flecs로 분리**한다.

---

## 2. 현재 실제 구조

현재 코드에서 `ViewerApp`이 `flecs::world`를 소유한다.

```text
ViewerApp
├─ Window
├─ Renderer
├─ Camera
└─ flecs::world
   ├─ RenderContext
   ├─ Entity: Transform + Renderable
   └─ RenderSystem
```

주요 코드:

- `apps/viewer/ViewerApp.cpp`
- `apps/viewer/ViewerApp.h`
- `modules/viewer/include/RenderContext.h`
- `modules/viewer/include/components/Transform.h`
- `modules/viewer/include/components/Renderable.h`
- `modules/viewer/src/systems/RenderSystem.cpp`

---

## 3. `RenderContext`

`Renderer`와 `Camera`는 ECS component가 아니라 application-owned subsystem이다.

```text
ViewerApp owns Renderer / Camera
        ↓
world.set<RenderContext>()
        ↓
RenderSystem reads RenderContext
```

이 구조 덕분에 `Renderer`는 Flecs를 알 필요가 없다.

---

## 4. `RenderSystem`

현재 `RenderSystem`은 Flecs module/system으로 등록된다.

```text
world.progress(dt)
        ↓
RenderSystem
        ↓
Transform + Renderable query
        ↓
RenderContext
        ↓
Renderer::Draw
```

Renderer는 entity 이름, 개수, 생성 순서를 알지 않는다.

---

## 5. 왜 모든 기능을 Flecs System으로 만들지 않는가

Flecs를 프로젝트 전체 framework로 강제하지 않는다.

적합한 경우:

```text
같은 component 조합을 가진 여러 entity에
동일 규칙을 반복 적용
```

예:

```text
Transform + Renderable
→ RenderSystem
```

반면 RobotDescription, FK, IK, grasp 계산은 일반 C++ module로 먼저 구현하고 검증한다. 계산된 결과 transform만 scene entity에 반영한다.

---

## 6. Robot 단계에서의 Flecs 역할

예상 kinematic scene:

```text
Robot Base
└─ Link1
   └─ Link2
      └─ Link3
         └─ Link4
            └─ Link5
               └─ Link6
                  └─ End Effector
                     └─ 2F85 Gripper
```

향후 검토 대상:

- `RobotLink` tag/component
- `Joint` data component
- `ChildOf` relationship
- hierarchy traversal

다만 FK 계산을 Flecs hierarchy에 먼저 맞추지 않는다. HCR-12A의 6DoF kinematics를 일반 C++로 검증한 뒤 계산된 world transform을 Flecs entity에 반영한다. CAD assembly hierarchy는 visual asset 구성에 사용하되 kinematic hierarchy의 source of truth로 사용하지 않는다.

---

## 7. Resource ownership

`Renderable`은 `Mesh`, `Shader`, `Texture`를 `shared_ptr`로 참조한다. 여러 entity가 동일 GPU resource를 공유할 수 있다.

`ViewerApp::Shutdown()`에서는 world를 GPU context보다 먼저 정리해야 한다.

```text
world/component 파괴
→ Renderable reference 감소
→ GPU resource 파괴
→ 마지막에 Window/OpenGL context 파괴
```

---

## 8. 현재와 향후 기능 구분

| 기능 | 상태 |
|---|---|
| `flecs::world` ownership | 구현 |
| `RenderContext` world context | 구현 |
| `RenderSystem` module/system | 구현 |
| `world.progress(dt)` loop | 구현 |
| `Transform + Renderable` query | 구현 |
| HCR-12A Link1~6 entity | 예정 |
| 2F85 Gripper / Object role component | 예정 |
| Flecs hierarchy / relationship | 검토 예정 |
| FK/IK system | ECS 종속 없이 먼저 구현 |

---

## 9. 판단 기준

Flecs를 사용하는 목적은 다음 두 책임을 분리하는 것이다.

```text
Renderer
= OpenGL draw 책임

Flecs World/System
= 어떤 scene object를 반복 처리할지 결정
```

Robot kinematics는 이 둘과 독립된 계산 계층으로 유지한다.
