// [추가 그래픽스 용어 설명]
// Shadow pass 전용 Shader다. 화면 색을 만드는 것이 아니라 light 관점의 depth만 기록한다.
// - Depth-only Pass: 색 계산 없이 "광원에서 가장 가까운 표면 깊이"만 저장하는 렌더링 단계.
// - Light-space Matrix: World 좌표를 광원이 보는 View/Projection 좌표로 변환하는 행렬.
// - Fragment Shader가 비어 있는 이유: color output이 필요 없고 framebuffer의 depth attachment만 사용하기 때문이다.

#type vertex
#version 330 core

layout(location = 0) in vec3 a_Position;

uniform mat4 u_Model;
uniform mat4 u_LightSpaceMatrix;

void main()
{
    // Mesh local position -> World -> Light clip space 순서로 변환한다.
    gl_Position =
        u_LightSpaceMatrix *
        u_Model *
        vec4(a_Position, 1.0);
}


#type fragment
#version 330 core

void main()
{
    // 명시적 color 출력은 없다. Rasterization 결과의 depth 값만 ShadowMap depth texture에 기록된다.
}
