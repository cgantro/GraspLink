#pragma once

class Entity;
class Scene;
struct ModelResource;

namespace grasplink::simulation
{
/**
 * @brief GLB 모델 정보로 TwoF85 본체와 관절의 Kinematic 충돌 프록시 7개를 구성한다.
 * @details 본체와 6개 관절의 ECS Entity를 각각 프록시 부모로 사용한다. 현재 모델의 이름이 지정된 메시 9개를
 * 각각 하나의 볼록 외피로 바꾸며 프록시의 Local 자세는 항등이다. 정점은 소유 Body/관절 좌표계 기준
 * [m]이고 GLB의 비항등 Gripper 장착 변환은 원본 계층에 남겨 실행 중 World 변환에 반영한다.
 * 자세의 기준은 원본 ECS 관절이다. Fixed Update가 World 변환을 갱신한 뒤 PhysicsSystemModule이 Kinematic
 * 목표를 동기화한다. 형상은 오목한 공간을 메울 수 있다. 그리퍼 FK·제어 백엔드·접촉 기반 파지는 구현하지 않는다.
 * 입력을 먼저 검증하고 프록시 7개의 형상을 모두 준비한 뒤 생성하므로 잘못된 모델은 부분 Entity를 남기지 않는다.
 * @param scene robotRoot와 새 프록시를 소유하는 Scene.
 * @param robotRoot Gripper 계층을 포함하는 생성된 로봇 Entity의 루트.
 * @param model TwoF85 노드·부모 관계와 [m] 정점을 제공하는 로드된 GLB 자원.
 * @throws std::invalid_argument root, 필수 node, Scene/GLB 계층, transform 또는 hull이 유효하지 않을 때.
 */
void ConfigureTwoF85Colliders(Scene& scene, const Entity& robotRoot, const ModelResource& model);
}
