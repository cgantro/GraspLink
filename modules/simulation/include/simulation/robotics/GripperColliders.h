#pragma once

class Entity;
class Scene;
struct ModelResource;

namespace grasplink::simulation
{
/**
 * @brief TwoF85 화면 메시에서 본체와 여섯 관절의 Kinematic 충돌 Body를 만든다.
 * @details Body는 Jolt가 위치와 충돌을 계산하는 물체다.
 * Proxy는 화면 모델 대신 충돌 계산에 쓰는 별도 Entity다.
 * 본체와 여섯 관절 Entity를 각각 일곱 Body의 부모로 사용한다.
 * 이름으로 지정된 메시 아홉 개에서 각자 하나의 ConvexHull을 만든다.
 * ConvexHull은 점을 둘러싼 볼록한 모양이므로 메시의 오목한 곳을 채울 수 있다.
 * 정점은 소유 Body나 관절 기준 [m]이고 proxy Local 위치와 회전은 항등이다.
 * Gripper 전체의 장면 배치는 원래 Entity 계층에 남아 각 proxy의 World 변환에 반영된다.
 * 앱은 Controller 상태에서 관절 회전을 계산하고 원본 관절에 적용한다.
 * Fixed Update가 World 변환을 갱신한 뒤 PhysicsSystemModule이 이를 Body 목표로 전달한다.
 * 이 함수는 모양만 만들며 접촉한 물체를 잡으려고 손가락을 멈추거나 움직이지 않는다.
 * 입력과 외피를 먼저 검증하므로 실패 시 일부 proxy만 남지 않는다.
 * @param scene root와 새 proxy Entity를 소유하는 Scene이다.
 * @param robotRoot Gripper 계층을 포함하는 생성된 로봇 루트 Entity다.
 * @param model TwoF85 노드 관계와 정점 [m]을 제공하는 로드된 GLB 자원이다.
 * @throws std::invalid_argument 필수 노드, 계층 변환 또는 충돌 모양이 유효하지 않을 때 발생한다.
 */
void ConfigureTwoF85Colliders(Scene& scene, const Entity& robotRoot, const ModelResource& model);
}
