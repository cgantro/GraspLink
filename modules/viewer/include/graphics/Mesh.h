#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>
namespace PoseLink
{
class VertexArray;
class VertexBuffer;
class IndexBuffer;
class Mesh;

/**
 * @brief 단일 GLB asset 안에서 독립적으로 그릴 수 있는 named mesh다.
 *
 * HCR-12A GLB는 base와 여섯 link를 한 파일에 담지만, FK는 link마다 서로 다른
 * rigid transform을 적용한다. `name`은 CAD visual rig가 어느 FK reference frame에
 * 연결할지를 검증하는 stable asset contract이고, `mesh`의 GPU resource는
 * `Renderable`과 같은 shared ownership으로 보관된다.
 */
struct StaticGlbMesh
{
    std::string name;
    std::shared_ptr<Mesh> mesh;
};

/*
    정점 하나의 데이터
    pos -> 3D 고안
    tex -> Texture에서 어느 위치를 읽을지 나타내는 u,v 좌표
    [x][y][z][u][v]
*/
struct Vertex{
    glm::vec3 position;
    glm::vec2 texCoord;
};
class Mesh{
public:
    /*
        vertices CPU상의 Vertex 시작주소
        vertexSize Vertex 데이터 크기(Byte)
        indices index 배열 시작주소
        indexCount index 개수

        Mesh는 전달받은 데이터를 GPU Buffer로 복사
        생성자가 끝난뒤 원본 배열이 사라져도 문제 X
    */
    Mesh(const Vertex* vertices, uint32_t vertexCount ,
        const uint32_t* indices, uint32_t indexCount);
    ~Mesh();

    // 복사 X
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    

    /*
        이 Mesh의 VAO를 OpenGL Context에 bind
        VAO 내부 :
            - Vertex 특징 설정
            - VBO 정보
            - EBO 정보
    */
    void Bind() const;
    void UnBind() const;
    static std::unique_ptr<Mesh> CreateCube();
    /**
     * @brief Offline-converted triangular OBJ를 GPU mesh로 읽는다.
     *
     * STEP/CAD parser는 runtime dependency가 아니며 FreeCAD가 link별 OBJ를 만든다.
     * 이 importer는 `v`, optional `vt`, 그리고 triangle 또는 fan-triangulatable `f`만
     * 허용한다. HCR link는 local joint frame을 유지한 채 export해야 하므로 이 함수는
     * mesh vertex에 robot transform을 추가로 적용하지 않는다.
     */
    static std::unique_ptr<Mesh> LoadObj(const std::string& path);

    /**
     * @brief GraspLink exporter profile의 one-file static GLB를 named mesh 목록으로 읽는다.
     *
     * 이 함수는 범용 glTF importer가 아니다. GLB v2, embedded BIN chunk, `POSITION`
     * float/VEC3, unsigned-32 triangle index라는 `export_hcr12a_glb.py`의 정확한
     * profile만 허용한다. profile 밖의 skin, animation, external URI, sparse accessor는
     * 명시적으로 지원하지 않아 CAD asset parsing이 renderer의 공격 표면이나 hidden
     * dependency가 되지 않는다. GLB normal은 현재 unlit texture shader가 소비하지
     * 않으므로 검증만 하고, position/index만 OpenGL mesh로 옮긴다.
     */
    static std::vector<StaticGlbMesh> LoadStaticGlb(const std::string& path);
    uint32_t GetIndexCount() const;
private:
    std::unique_ptr<VertexArray> m_VertexArray;
    std::unique_ptr<VertexBuffer> m_VertexBuffer;
    std::unique_ptr<IndexBuffer> m_IndexBuffer;
};
} // namespace poseLink
