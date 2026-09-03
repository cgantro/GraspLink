# Why PoseLink Uses Flecs

## 1. 도입 전 문제

기존 Viewer는 `Renderer`가 Cube의 `Mesh`, `Shader`, `Texture`와 객체 변환 상태를 직접 보유했다. Object를 추가할 때마다 Renderer의 멤버와 렌더링 코드를 함께 늘려야 했고, Object 생성·제거의 책임도 Renderer에 섞였다. 별도의 `Scene`과 `SceneObject`를 직접 구현하기 시작하면 entity 식별, 수명, 조회, 순회를 다시 구현하게 된다.

## 2. 왜 flecs인가

PoseLink에는 서로 다른 속성을 조합하는 visual object, pose target, robot link, camera가 생길 수 있다. 이들을 고정된 `SceneObject` 필드로 표현하기보다 필요한 component를 entity에 조합하는 방식이 현재 Object 관리 문제에 맞는다. flecs는 그 entity 생성·제거와 component 조회를 제공한다.

## 3. 직접 Scene을 만드는 경우와의 비교

직접 Scene을 만들면 `Scene`, `SceneObject`, ObjectID, Object lifetime, lookup, iteration을 프로젝트가 소유한다. flecs를 사용하면 이 관리 경로는 `flecs::world`와 `flecs::entity`로 대체된다. 직접 구현이 부적절해서가 아니라, 이미 ECS와 유사한 관리 코드를 만들기 시작한 현재 범위에서는 검증된 ECS 라이브러리를 쓰는 편이 관리할 코드가 적다.

## 4. flecs 적용 범위

flecs는 entity 생성·제거, component 데이터, `Transform + Renderable` 조회만 맡는다. OpenGL resource 생성, Shader, Mesh, Texture, 네트워크 protocol, pose estimation은 기존 구현의 책임으로 남는다. flecs가 프로젝트 전체를 지배하는 프레임워크가 아니다.

## 5. 최종 구조

```text
ViewerApp
|- Window
|- Renderer
|- Camera
`- flecs::world
   |- CubeA: Transform + Renderable
   `- CubeB: Transform + Renderable

flecs::world
  -> Transform + Renderable query
  -> Renderer
  -> OpenGL
```

`ViewerApp`은 OpenGL context가 만들어진 뒤 world와 Cube 리소스를 생성한다. 종료 시에는 world를 Renderer와 Window보다 먼저 파괴하므로, `Renderable`이 공유하는 GPU resource가 OpenGL context가 살아 있는 동안 해제된다.

## 6. Trade-off

flecs를 도입하면 외부 dependency와 API 학습 비용이 추가된다. 작은 프로젝트에서는 직접 Scene을 관리하는 편이 더 단순할 수도 있다. 또한 ECS component가 `shared_ptr`로 참조하는 GPU resource의 생명주기 경계를 명확히 지켜야 한다. PoseLink에서는 Renderer의 Object 소유를 제거하고 entity 조합을 실제로 검증하는 이점이 이 비용보다 크다.
