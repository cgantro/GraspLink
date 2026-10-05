#include "gui/overlays/ColliderOverlay.h"

#include "Camera.h"
#include "simulation/components/PhysicsComponents.h"
#include "components/TransformComponents.h"

#include <imgui.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace grasplink::gui
{
namespace
{
/** @brief World 형상을 투영한 화면 픽셀 선과 layer 색을 저장한다. */
struct ScreenLine
{
    ImVec2 from;
    ImVec2 to;
    ImU32 color = IM_COL32_WHITE;
};

/** @brief 설정 Shape의 Body-local 위치·회전을 행렬로 바꾼다. 크기는 Shape 치수 [m]에서 읽는다. */
glm::mat4 LocalShapeMatrix(const grasplink::physics::Transform& transform)
{
    return glm::translate(glm::mat4{1.0F}, transform.position) *
        glm::mat4_cast(glm::normalize(transform.rotation));
}

/** @brief World 점을 ImGui 화면 픽셀로 투영하고 Camera 뒤의 점은 제외한다. */
bool Project(const glm::vec3& position, const glm::mat4& viewProjection,
    const ImVec2& displaySize, ImVec2& result)
{
    const glm::vec4 clip = viewProjection * glm::vec4{position, 1.0F};
    // Camera 뒤나 투영 분모가 너무 작은 점을 거른다. near plane과 교차하는 선은 자르지 않는다.
    // 경계 부근에서는 일부 선이 생략되거나 화면 밖으로 크게 늘어날 수 있다.
    if (clip.w <= 0.001F)
        return false;

    // Clip 좌표를 w로 나눠 NDC [-1,1]로 바꾸고 y축을 뒤집어 ImGui 화면 픽셀로 옮긴다.
    const glm::vec3 ndc = glm::vec3{clip} / clip.w;
    result = {((ndc.x + 1.0F) * 0.5F) * displaySize.x,
        ((1.0F - ndc.y) * 0.5F) * displaySize.y};
    return true;
}

/** @brief Shape-local 선의 두 끝점을 World로 옮겨 화면 선 캐시에 넣는다. */
void DrawLine(std::vector<ScreenLine>& lines, const glm::vec3& from, const glm::vec3& to,
    const glm::mat4& shapeWorld, const glm::mat4& viewProjection,
    const ImVec2& displaySize, ImU32 color)
{
    ImVec2 screenFrom;
    ImVec2 screenTo;
    const glm::vec3 worldFrom = glm::vec3{shapeWorld * glm::vec4{from, 1.0F}};
    const glm::vec3 worldTo = glm::vec3{shapeWorld * glm::vec4{to, 1.0F}};
    if (Project(worldFrom, viewProjection, displaySize, screenFrom) &&
        Project(worldTo, viewProjection, displaySize, screenTo))
        lines.push_back({screenFrom, screenTo, color});
}

/** @brief 설정 Box의 12개 모서리를 화면 선으로 만든다. */
void DrawBox(std::vector<ScreenLine>& lines, const grasplink::physics::CollisionShapeDescription& shape,
    const glm::mat4& entityWorld, const glm::mat4& viewProjection,
    const ImVec2& displaySize, ImU32 color)
{
    // 설정된 반쪽 크기로 8 꼭짓점을 만들고 12개 모서리를 World→ViewProjection→화면으로 변환한다.
    const glm::mat4 world = entityWorld * LocalShapeMatrix(shape.localTransform);
    std::array<glm::vec3, 8> corners;
    for (unsigned int i = 0; i < corners.size(); ++i)
        corners[i] = {
            (i & 1U) ? shape.halfExtentsMeters.x : -shape.halfExtentsMeters.x,
            (i & 2U) ? shape.halfExtentsMeters.y : -shape.halfExtentsMeters.y,
            (i & 4U) ? shape.halfExtentsMeters.z : -shape.halfExtentsMeters.z};

    for (unsigned int i = 0; i < corners.size(); ++i)
        for (unsigned int axis = 0; axis < 3; ++axis)
            if ((i & (1U << axis)) == 0)
                DrawLine(lines, corners[i], corners[i | (1U << axis)], world,
                    viewProjection, displaySize, color);
}

/** @brief Local Y축 Cylinder의 위·아래 원과 세로선을 화면에 투영한다. */
void DrawCylinder(std::vector<ScreenLine>& lines, const grasplink::physics::CollisionShapeDescription& shape,
    const glm::mat4& entityWorld, const glm::mat4& viewProjection,
    const ImVec2& displaySize, ImU32 color)
{
    constexpr int Segments = 24;
    // Cylinder 축은 Local Y. 위·아래 원과 일정 간격의 세로 모서리로 원통을 선 근사한다.
    const glm::mat4 world = entityWorld * LocalShapeMatrix(shape.localTransform);
    for (int i = 0; i < Segments; ++i)
    {
        const float a = glm::two_pi<float>() * static_cast<float>(i) / Segments;
        const float b = glm::two_pi<float>() * static_cast<float>(i + 1) / Segments;
        const glm::vec3 aBottom{shape.radiusMeters * std::cos(a), -shape.halfHeightMeters,
            shape.radiusMeters * std::sin(a)};
        const glm::vec3 bBottom{shape.radiusMeters * std::cos(b), -shape.halfHeightMeters,
            shape.radiusMeters * std::sin(b)};
        const glm::vec3 aTop{aBottom.x, shape.halfHeightMeters, aBottom.z};
        const glm::vec3 bTop{bBottom.x, shape.halfHeightMeters, bBottom.z};
        DrawLine(lines, aBottom, bBottom, world, viewProjection, displaySize, color);
        DrawLine(lines, aTop, bTop, world, viewProjection, displaySize, color);
        if (i % 4 == 0)
            DrawLine(lines, aBottom, aTop, world, viewProjection, displaySize, color);
    }
}

/** @brief 설정 Sphere를 직교하는 세 원으로 근사해 화면에 투영한다. */
void DrawSphere(std::vector<ScreenLine>& lines, const grasplink::physics::CollisionShapeDescription& shape,
    const glm::mat4& entityWorld, const glm::mat4& viewProjection,
    const ImVec2& displaySize, ImU32 color)
{
    constexpr int Segments = 24;
    // 서로 직교하는 세 원으로 구의 설정 반지름을 읽기 쉽게 표시한다.
    const glm::mat4 world = entityWorld * LocalShapeMatrix(shape.localTransform);
    for (int plane = 0; plane < 3; ++plane)
    {
        for (int i = 0; i < Segments; ++i)
        {
            const float a = glm::two_pi<float>() * static_cast<float>(i) / Segments;
            const float b = glm::two_pi<float>() * static_cast<float>(i + 1) / Segments;
            glm::vec3 from{shape.radiusMeters * std::cos(a), shape.radiusMeters * std::sin(a), 0.0F};
            glm::vec3 to{shape.radiusMeters * std::cos(b), shape.radiusMeters * std::sin(b), 0.0F};
            if (plane == 1)
            {
                from = {from.x, 0.0F, from.y};
                to = {to.x, 0.0F, to.y};
            }
            else if (plane == 2)
            {
                from = {0.0F, from.x, from.y};
                to = {0.0F, to.x, to.y};
            }
            DrawLine(lines, from, to, world, viewProjection, displaySize, color);
        }
    }
}

/** @brief Hull 설정 정점의 투영 윤곽을 만든다. 실제 3D hull 모서리는 조회하지 않는다. */
void DrawConvexHull(std::vector<ScreenLine>& lines, const grasplink::physics::CollisionShapeDescription& shape,
    const glm::mat4& entityWorld, const glm::mat4& viewProjection,
    const ImVec2& displaySize, ImU32 color)
{
    const glm::mat4 world = entityWorld * LocalShapeMatrix(shape.localTransform);
    std::vector<ImVec2> points;
    points.reserve(shape.pointsMeters.size());
    for (const glm::vec3& point : shape.pointsMeters)
    {
        ImVec2 screen;
        if (Project(glm::vec3{world * glm::vec4{point, 1.0F}}, viewProjection, displaySize, screen))
            points.push_back(screen);
    }
    if (points.size() < 3) return;

    // 투영된 점의 2D 바깥 윤곽을 연결한다. 실제 3D convex hull의 모서리를 복원하지 않는다.
    std::sort(points.begin(), points.end(), [](const ImVec2& a, const ImVec2& b)
        { return a.x == b.x ? a.y < b.y : a.x < b.x; });
    auto cross = [](const ImVec2& a, const ImVec2& b, const ImVec2& c)
        { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); };
    std::vector<ImVec2> hull;
    hull.reserve(points.size() * 2);
    for (const ImVec2& point : points)
    {
        while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), point) <= 0.0F)
            hull.pop_back();
        hull.push_back(point);
    }
    const std::size_t lowerSize = hull.size();
    for (auto iterator = points.rbegin() + 1; iterator != points.rend(); ++iterator)
    {
        while (hull.size() > lowerSize && cross(hull[hull.size() - 2], hull.back(), *iterator) <= 0.0F)
            hull.pop_back();
        hull.push_back(*iterator);
    }
    if (hull.size() > 2) hull.pop_back();
    for (std::size_t i = 0; i < hull.size(); ++i)
        lines.push_back({hull[i], hull[(i + 1) % hull.size()], color});
}

/** @brief 패널 범례와 같은 Collision layer 색을 반환한다. */
ImU32 LayerColor(grasplink::physics::CollisionLayer layer)
{
    switch (layer)
    {
    case grasplink::physics::CollisionLayer::Environment: return IM_COL32(70, 230, 100, 255);
    case grasplink::physics::CollisionLayer::Robot: return IM_COL32(255, 110, 55, 255);
    case grasplink::physics::CollisionLayer::Gripper: return IM_COL32(210, 90, 255, 255);
    case grasplink::physics::CollisionLayer::DynamicObject: return IM_COL32(70, 210, 255, 255);
    }
    return IM_COL32_WHITE;
}

glm::mat4 AsMatrix(const TransformMatrix& matrix)
{
    return static_cast<const glm::mat4&>(matrix);
}

/** @brief Entity World 행렬에서 Physics Body 원점의 위치·회전만 추출한다. */
glm::mat4 PhysicsTransform(const glm::mat4& world)
{
    // Body pose처럼 위치·회전만 사용한다. Collider 크기는 설정된 [m] 값이며 Entity scale은 적용하지 않는다.
    glm::vec3 scale{1.0F};
    glm::quat rotation{1.0F, 0.0F, 0.0F, 0.0F};
    glm::vec3 position{0.0F};
    glm::vec3 skew{0.0F};
    glm::vec4 perspective{0.0F};
    if (!glm::decompose(world, scale, rotation, position, skew, perspective))
        return glm::mat4{1.0F};
    return glm::translate(glm::mat4{1.0F}, position) * glm::mat4_cast(glm::normalize(rotation));
}
}

struct ColliderOverlay::Impl
{
    // ECS 설정 Collider와 최신 World 행렬을 읽는다. Jolt가 실제 생성한 Shape의 변형/축약 결과는 조회하지 않는다.
    flecs::query<const RigidBody, const Colliders, const TransformMatrix> colliderQuery;
    std::vector<ScreenLine> collisionLines;
    std::chrono::steady_clock::time_point nextCollisionRefresh{};
    ImVec2 cachedDisplaySize{};
    bool wasVisible = false;

    explicit Impl(flecs::world& world)
        : colliderQuery(world.query_builder<const RigidBody, const Colliders, const TransformMatrix>()
            .term_at(2).second<World>().build())
    {
    }

    void Draw(const Camera& camera, bool visible)
    {
        if (visible)
        {
            const auto now = std::chrono::steady_clock::now();
            const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
            const bool resized = displaySize.x != cachedDisplaySize.x || displaySize.y != cachedDisplaySize.y;
            if (!wasVisible || resized || now >= nextCollisionRefresh)
            {
                RefreshCollisionLines(camera);
                cachedDisplaySize = displaySize;
                nextCollisionRefresh = now + std::chrono::milliseconds(100);
            }
            // Screen 좌표를 캐시하므로 Camera 이동도 다음 갱신까지 이전 투영을 쓴다.
            // Foreground draw list는 깊이 버퍼를 검사하지 않아 가려진 Collider 선도 보인다.
            ImDrawList* drawList = ImGui::GetForegroundDrawList();
            for (const ScreenLine& line : collisionLines)
                drawList->AddLine(line.from, line.to, line.color, 1.5F);
        }
        else
        {
            collisionLines.clear();
        }
        wasVisible = visible;
    }

    void RefreshCollisionLines(const Camera& camera)
    {
        collisionLines.clear();
        // Camera 투영 × View로 World 좌표를 clip 좌표로 보낸다. Entity scale은 PhysicsTransform에서 제외한다.
        const glm::mat4 viewProjection = camera.GetProjectionMatrix() * camera.GetViewMatrix();
        const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
        colliderQuery.each([&](flecs::entity, const RigidBody& rigidBody,
            const Colliders& colliders, const TransformMatrix& matrix)
        {
            const glm::mat4 entityWorld = PhysicsTransform(AsMatrix(matrix));
            const ImU32 color = LayerColor(rigidBody.collisionLayer);
            for (const auto& shape : colliders.shapes)
            {
                switch (shape.type)
                {
                case grasplink::physics::CollisionShapeType::Box:
                    DrawBox(collisionLines, shape, entityWorld, viewProjection, displaySize, color);
                    break;
                case grasplink::physics::CollisionShapeType::Cylinder:
                    DrawCylinder(collisionLines, shape, entityWorld, viewProjection, displaySize, color);
                    break;
                case grasplink::physics::CollisionShapeType::Sphere:
                    DrawSphere(collisionLines, shape, entityWorld, viewProjection, displaySize, color);
                    break;
                case grasplink::physics::CollisionShapeType::ConvexHull:
                    DrawConvexHull(collisionLines, shape, entityWorld, viewProjection, displaySize, color);
                    break;
                }
            }
        });
    }
};

ColliderOverlay::ColliderOverlay(flecs::world& world)
    : m_Impl(std::make_unique<Impl>(world))
{
}

ColliderOverlay::~ColliderOverlay() = default;

void ColliderOverlay::Draw(const Camera& camera, bool visible)
{
    m_Impl->Draw(camera, visible);
}

}
