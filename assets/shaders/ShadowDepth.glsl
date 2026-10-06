
/**
 * @file ShadowDepth.glsl
 * @brief 광원 카메라에서 본 표면 깊이를 이미지에 저장해 나중에 그림자 여부를 판정한다.
 * @details
 * 꼭짓점은 삼각형의 모서리 위치다. 모델 변환은 Mesh 자체 기준 좌표를 장면 전체 좌표로 옮기며 광원 카메라 변환은 광원 위치와 방향에서 본 좌표로 바꾼다.
 * GPU는 삼각형이 덮는 픽셀마다 광원 카메라 기준 깊이를 계산한다. 깊이는 실제 광원 거리나 기본 카메라 앞뒤값이 아니라 투영 방향에서의 앞뒤 순서다.
 * 이 단계는 색을 출력하지 않고 framebuffer의 깊이 저장소에 더 가까운 표면의 투영 깊이를 남긴다. Framebuffer는 실제 창이 아니라 GPU가 결과를 기록하는 저장소다.
 * 화면 색을 계산할 때 같은 광원 변환으로 깊이를 구해 이 이미지와 비교하면 광선이 다른 표면에 막혔는지 알 수 있다.
 */
#type vertex
#version 330 core

layout(location = 0) in vec3 a_Position;

uniform mat4 u_Model;
uniform mat4 u_LightSpaceMatrix;

void main()
{

    // Mesh 위치를 장면 좌표로 바꾼 뒤 광원에서 본 화면 좌표로 옮겨 깊이 기록 위치를 정한다.
    gl_Position =
        u_LightSpaceMatrix *
        u_Model *
        vec4(a_Position, 1.0);
}

#type fragment
#version 330 core

void main()
{
    // 색상은 출력하지 않는다. GPU가 픽셀마다 광원 카메라에서 본 표면 깊이를 깊이 저장소에 기록한다.
}
