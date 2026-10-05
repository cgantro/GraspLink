#pragma once

#include "assets/GraphicsTypes.h"

#include <cstdint>
#include <memory>

#include <glm/glm.hpp>

class VertexArray;
class VertexBuffer;
class IndexBuffer;

/**
 * @brief 정점 속성과 삼각형 연결 정보를 GPU에 올려 그리기에 제공한다.
 * @details
 * 정점은 위치만 있는 점이 아니라 법선과 UV 같은 표면 속성도 함께 가진다. 이 Mesh는 Vertex 구조체를
 * 정점마다 연속 저장하는 배치를 사용한다. VertexArray는 각 attribute의 위치·크기·byte 간격을 기억하고,
 * VertexBuffer는 위치/법선/UV 값을 보관하며, IndexBuffer는 어떤 정점 세 개가 삼각형 하나를 이룰지
 * 번호로 보관한다. index 배열은 정점을 재사용해 저장량을 줄이고, 면마다 법선이나 UV가 달라야 할 때는
 * 정점을 나누어 저장한다. 현재 Shader에서 위치는 모델 변환으로 World 좌표가 되고, 법선은 조명 방향과
 * 비교되어 표면 밝기에 영향을 주며, UV는 Base Color texture를 읽는 위치가 된다. 이 타입은 CPU 배열을
 * 생성 중 GPU buffer에 복사하므로 업로드 뒤 원본 배열의 수명과 독립적이다. GPU 자원을 파괴하는 마지막
 * 참조의 해제 시점에는 해당 OpenGL context가 현재 스레드에서 유효해야 한다.
 * Vertex에는 tangent도 있지만 현재 attribute 설정과 Shader 입력에 연결하지 않아 그 값은 렌더링에
 * 쓰이지 않는다. 따라서 현재 경로는 tangent 기반 normal mapping을 수행하지 않는다.
 */
class Mesh final
{
public:
    /**
     * @brief CPU 정점·index 배열을 GPU에 복사하고 정점 입력 형식을 설정한다.
     * @param vertices Vertex 배열 주소. 생성 중 복사하므로 호출자가 이후에도 보관할 필요는 없다.
     * @param vertexCount vertices의 Vertex 원소 개수이며 byte 수가 아니다.
     * @param indices 삼각형을 구성하는 정점 번호 배열 주소.
     * @param indexCount indices의 uint32 원소 개수이며 byte 수가 아니다.
     * @throws std::invalid_argument 배열 주소가 null이거나 해당 원소 개수가 0인 경우.
     * @throws std::runtime_error 정점 배열의 byte 크기가 VertexBuffer가 받는 범위를 넘는 경우.
     * @details 각 index 값은 vertices 안의 유효한 정점 번호여야 한다. 렌더러는 draw 시작 위치도 index
     * 원소 단위로 받으며, glDrawElements에는 uint32 원소 크기를 곱한 byte offset을 전달한다. 이 생성자는
     * 개별 index 범위나 삼각형을 이루는 개수인지를 검사하지 않으므로 호출자가 올바른 데이터를 제공해야 한다.
     */
    Mesh(
        const Vertex* vertices,
        std::uint32_t vertexCount,
        const std::uint32_t* indices,
        std::uint32_t indexCount);

    /** @brief 소유한 OpenGL 객체를 해제한다. 해당 객체를 만든 유효한 GL context가 필요하다. */
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    /**
     * @brief 이 Mesh의 VAO를 연결해 다음 draw가 사용할 정점 형식과 EBO를 선택한다.
     * @details VAO는 데이터 자체가 아니라 attribute의 읽는 법과 EBO 연결을 기억한다. 호출자는 draw 뒤
     * UnBind를 호출해 현재 VAO를 해제해야 한다.
     */
    void Bind() const;

    /** @brief 현재 VAO를 해제한다. */
    void UnBind() const;

    /** @brief EBO에 저장된 index 원소 개수를 반환한다. 단위: index 개수. */
    std::uint32_t GetIndexCount() const;

    /**
     * @brief Local 원점 중심의 정육면체 Mesh를 만든다.
     * @param sideLengthMeters 한 변 길이 [m]. 유한한 양수여야 한다.
     * @return 각 면의 법선과 UV 경계를 유지하도록 면마다 정점을 나눈 Mesh. 삼각형은 12개다.
     * @throws std::invalid_argument 변 길이가 유한하지 않거나 0 이하인 경우.
     */
    static std::unique_ptr<Mesh> CreateCube(float sideLengthMeters = 1.0F);

private:
    // VAO는 location별 형식과 EBO 연결을 저장하며, 실제 정점과 index 데이터는 아래 buffer가 가진다.
    std::unique_ptr<VertexArray> vertexArray_;

    // VBO에는 Vertex 배열의 연속된 byte가 들어간다. attribute stride는 이 구조체 크기다.
    std::unique_ptr<VertexBuffer> vertexBuffer_;

    // EBO에는 uint32 정점 번호가 들어가며 draw 시 GL_TRIANGLES가 세 번호씩 삼각형으로 읽는다.
    std::unique_ptr<IndexBuffer> indexBuffer_;
};
