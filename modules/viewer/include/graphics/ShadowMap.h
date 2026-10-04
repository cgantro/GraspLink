#pragma once

#include <cstdint>

/**
 * @brief Directional light shadow mapping용 depth texture와 framebuffer를 관리한다.
 *
 * @details
 * Shadow map은 light 관점에서 scene의 가장 가까운 depth를 texture에 기록한다.
 * Main pass에서는 현재 fragment의 light-space depth와 저장된 depth를 비교해 가려졌는지 판단한다.
 * Begin/End는 shadow pass 동안 바뀌는 framebuffer와 viewport를 저장/복원한다.
 *
 * @todo [FUTURE] shadow resolution과 polygon offset 값을 graphics settings로 이동한다.
 * @todo [FUTURE] 넓은 작업공간이 필요해지면 cascaded shadow map 또는 dynamic light frustum을 검토한다.
 */
/*
 * [추가 그래픽스 용어 설명]
 * - Directional Light: 태양처럼 모든 위치에 거의 같은 방향으로 들어오는 광원.
 * - Shadow Mapping: light가 보는 깊이를 texture로 저장한 뒤 main pass에서 가려짐을 판정하는 그림자 기법.
 * - Depth: 카메라/광원으로부터 얼마나 앞뒤에 있는지를 나타내는 값.
 * - Depth Texture: 색이 아니라 depth 값을 저장하는 texture.
 * - Light-space: World 좌표를 광원이 보는 좌표계/투영으로 변환한 공간.
 * - Viewport: framebuffer의 어느 pixel 영역에 렌더링할지 정하는 직사각형.
 * - Texture Unit: Shader가 여러 texture를 동시에 참조할 때 각각을 연결하는 slot 번호.
 * - Polygon Offset: shadow acne 같은 self-shadow artifact를 줄이기 위해 depth 값을 약간 밀어주는 기능.
 *
 * size는 정사각형 shadow map 한 변의 pixel 수다. 기본 2048은 2048x2048 depth texture를 뜻한다.
 */
class ShadowMap final
{
public:
    /** @brief 정사각형 depth map을 생성한다. */
    explicit ShadowMap(int size = 2048);

    /** @brief depth texture와 framebuffer를 해제한다. */
    ~ShadowMap();

    ShadowMap(const ShadowMap&) = delete;
    ShadowMap& operator=(const ShadowMap&) = delete;

    /** @brief shadow framebuffer/viewport를 활성화하고 depth buffer를 초기화한다. */
    void Begin();

    /** @brief 이전 framebuffer/viewport 상태를 복원한다. */
    void End();

    /** @brief 생성된 depth texture를 지정 texture unit에 바인딩한다. */
    void Bind(std::uint32_t slot) const;

private:
    // Shadow pass가 그릴 OpenGL framebuffer object ID.
    std::uint32_t m_Framebuffer = 0;

    // light 관점의 depth를 저장하는 OpenGL texture object ID.
    std::uint32_t m_DepthTexture = 0;

    // shadow map 한 변의 해상도 [pixel].
    int m_Size = 2048;

    // Shadow pass 전 사용하던 framebuffer를 End()에서 복원하기 위한 저장값.
    int m_PreviousFramebuffer = 0;

    // Shadow pass 전 viewport {x, y, width, height} [pixel]을 복원하기 위한 저장값.
    int m_PreviousViewport[4]{};
};
