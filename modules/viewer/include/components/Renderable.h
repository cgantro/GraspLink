#pragma once

#include <memory>

namespace PoseLink
{
class Mesh;
class Shader;
class Texture;

struct Renderable
{
    /*
        Entity가 화면에 그릴 수 있음을 나타내는 최소 component.

        Transform은 "어디에 그릴지"를, Renderable은 "무엇으로 그릴지"를 담당한다.
        Material/AssetManager처럼 아직 필요한 기능이 아닌 추상화는 추가하지 않는다.

        여러 Entity가 같은 Cube geometry와 shader, texture를 사용할 수 있으므로
        GPU resource의 소유권은 shared_ptr로 공유한다.
        Entity가 제거되어도 마지막 Renderable이 사라질 때만 resource가 해제된다.
    */
    std::shared_ptr<Mesh> mesh;
    std::shared_ptr<Shader> shader;
    std::shared_ptr<Texture> texture;
};
} // namespace PoseLink
