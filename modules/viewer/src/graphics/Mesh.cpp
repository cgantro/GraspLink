#include "Mesh.h"

#include "VertexArray.h"
#include "VertexBuffer.h"
#include "IndexBuffer.h"

#include <glad/glad.h>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace PoseLink
{
namespace
{
constexpr std::uint32_t kGlbMagic = 0x46546C67U; // ASCII "glTF" in little-endian.
constexpr std::uint32_t kGlbVersion2 = 2U;
constexpr std::uint32_t kJsonChunk = 0x4E4F534AU;
constexpr std::uint32_t kBinaryChunk = 0x004E4942U;
constexpr std::uint32_t kFloatComponent = 5126U;
constexpr std::uint32_t kUnsignedIntComponent = 5125U;

struct GlbBufferView
{
    std::size_t byteOffset = 0U;
    std::size_t byteLength = 0U;
    std::uint32_t target = 0U;
};

struct GlbAccessor
{
    std::size_t bufferView = 0U;
    std::uint32_t componentType = 0U;
    std::size_t count = 0U;
    std::string type;
};

struct GlbMeshDescription
{
    std::string name;
    std::size_t positionAccessor = 0U;
    std::size_t normalAccessor = 0U;
    std::size_t indexAccessor = 0U;
};

std::uint32_t ReadLittleEndian32(const std::vector<std::uint8_t>& bytes, std::size_t offset)
{
    if (offset > bytes.size() || bytes.size() - offset < 4U)
    {
        throw std::runtime_error("GLB ends inside a 32-bit field");
    }
    return static_cast<std::uint32_t>(bytes[offset])
        | (static_cast<std::uint32_t>(bytes[offset + 1U]) << 8U)
        | (static_cast<std::uint32_t>(bytes[offset + 2U]) << 16U)
        | (static_cast<std::uint32_t>(bytes[offset + 3U]) << 24U);
}

std::size_t ParseSize(const std::string& value, const char* field)
{
    try
    {
        const unsigned long long parsed = std::stoull(value);
        if (parsed > std::numeric_limits<std::size_t>::max())
        {
            throw std::runtime_error("GLB numeric field is too large: " + std::string(field));
        }
        return static_cast<std::size_t>(parsed);
    }
    catch (const std::exception&)
    {
        throw std::runtime_error("Invalid GLB numeric field: " + std::string(field));
    }
}

void RequireRange(std::size_t offset, std::size_t length, std::size_t containerLength, const char* what)
{
    if (offset > containerLength || length > containerLength - offset)
    {
        throw std::runtime_error(std::string("GLB ") + what + " lies outside the BIN chunk");
    }
}

float ReadLittleEndianFloat(const std::vector<std::uint8_t>& bytes, std::size_t offset)
{
    const std::uint32_t bits = ReadLittleEndian32(bytes, offset);
    float value = 0.0F;
    static_assert(sizeof(value) == sizeof(bits), "GLB float decoding requires IEEE-754 float32");
    std::memcpy(&value, &bits, sizeof(value));
    if (!std::isfinite(value))
    {
        throw std::runtime_error("GLB POSITION contains NaN or infinity");
    }
    return value;
}

std::vector<GlbBufferView> ParseBufferViews(const std::string& json)
{
    // This exact key order is the output contract of export_hcr12a_glb.py.
    // A permissive JSON search could mistake an extension for geometry, so the
    // loader rejects assets outside that deterministic profile instead.
    static const std::regex pattern(
        R"glb(\{"buffer":0,"byteOffset":([0-9]+),"byteLength":([0-9]+),"target":([0-9]+)\})glb");
    std::vector<GlbBufferView> result;
    for (std::sregex_iterator it(json.begin(), json.end(), pattern), end; it != end; ++it)
    {
        result.push_back({ParseSize((*it)[1].str(), "bufferView.byteOffset"),
                          ParseSize((*it)[2].str(), "bufferView.byteLength"),
                          static_cast<std::uint32_t>(ParseSize((*it)[3].str(), "bufferView.target"))});
    }
    return result;
}

std::vector<GlbAccessor> ParseAccessors(const std::string& json)
{
    static const std::regex pattern(
        R"glb(\{"bufferView":([0-9]+),"componentType":([0-9]+),"count":([0-9]+),"type":"([A-Z0-9]+)")glb");
    std::vector<GlbAccessor> result;
    for (std::sregex_iterator it(json.begin(), json.end(), pattern), end; it != end; ++it)
    {
        result.push_back({ParseSize((*it)[1].str(), "accessor.bufferView"),
                          static_cast<std::uint32_t>(ParseSize((*it)[2].str(), "accessor.componentType")),
                          ParseSize((*it)[3].str(), "accessor.count"), (*it)[4].str()});
    }
    return result;
}

std::vector<GlbMeshDescription> ParseMeshes(const std::string& json)
{
    static const std::regex pattern(
        R"glb(\{"name":"([^"]+)","primitives":\[\{"attributes":\{"POSITION":([0-9]+),"NORMAL":([0-9]+)\},"indices":([0-9]+),"material":[0-9]+\}\]\})glb");
    std::vector<GlbMeshDescription> result;
    for (std::sregex_iterator it(json.begin(), json.end(), pattern), end; it != end; ++it)
    {
        result.push_back({(*it)[1].str(), ParseSize((*it)[2].str(), "mesh.POSITION"),
                          ParseSize((*it)[3].str(), "mesh.NORMAL"),
                          ParseSize((*it)[4].str(), "mesh.indices")});
    }
    return result;
}
} // namespace
Mesh::Mesh(const Vertex* vertices, uint32_t vertexCount,
        const uint32_t* indices, uint32_t indexCount){
    if (vertices == nullptr || vertexCount  == 0)
        throw std::runtime_error("Mesh has no vertices");

    if (indices == nullptr || indexCount == 0)
        throw std::runtime_error("Mesh has no indices");

    
    /*
        VAO 먼저 Bind
        이후 설정하는 Vertex Att와 EBO Binding을 VAO가 기억한다.
    */
   
    m_VertexArray = std::make_unique<VertexArray>();
    m_VertexArray->Bind();

   /*
        실제 Vertex 데이터를 GPU로 복사
        현재는 [x][y][z][u][v]
   */
    const std::size_t byteCount = sizeof(Vertex) * static_cast<std::size_t>(vertexCount);
    if (byteCount > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::runtime_error("Mesh vertex data exceeds the current GPU buffer API size");
    }
    m_VertexBuffer = std::make_unique<VertexBuffer>(
        vertices, static_cast<std::uint32_t>(byteCount)
    );
    m_VertexBuffer->Bind();

    /*
        VAO의 Indexbuffer 생성
    */
    m_IndexBuffer = std::make_unique<IndexBuffer>(
        indices, indexCount
    );

    /*
        Vertex Att 0 = Position
        Px Py Pz U V
        l0

        Pos는 float 3개 사용
    */

    glEnableVertexAttribArray(0); // loc 0
    glVertexAttribPointer(
        0,                  // Shader의 layout(location = 0)
        3,                  // x, y, z
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),  // 다음 Vertex까지의 거리
        // Vertex 시작 위치부터 읽음
         reinterpret_cast<void*>(
            offsetof(Vertex, position)
        )            
    );

     /*
        Vertex Attribute 1 = Texture Coordinate

        [Px][Py][U][V]
                 ↑
                 location 1

        float 2개(Px, Py)를 건너뛴 위치에서 시작한다.
    */

    // offsetof
    // Vertex 구조체 시작 주소에서 texCoord까지 몇 Byte 떨어져 있어
    
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1,
        2,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(
            offsetof(Vertex, texCoord)
        )
    );

    m_VertexArray->UnBind();

    m_VertexBuffer->UnBind();
}
Mesh::~Mesh() = default;
void Mesh::Bind() const{
    /*
        VAO를 다시 Bind하면 생성 시 저장한
        - Vertex Att 설정
        - VBO, EBO 연결 정보 복원 가능
    */
    m_VertexArray->Bind();
}
void Mesh::UnBind() const{
    m_VertexArray->UnBind();
}

uint32_t Mesh::GetIndexCount() const{ return m_IndexBuffer->GetCount();}

std::unique_ptr<Mesh> Mesh::CreateCube(){
    /*
        Cube는 꼭짓점만 보면 8개임
        그러나 각 Face마다 Vertex따로 만듦

        공간상 물리적 위치(Position)가 같더라도, 
        그 정점이 속한 면(Face)마다 맵핑되는 UV 좌표(텍스처 좌표)가 다를 수 있기 때문
    */
   const Vertex vertices[] = {

        // Front (+Z)
        {{-0.5f, -0.5f,  0.5f}, {0.0f, 0.0f}},
        {{ 0.5f, -0.5f,  0.5f}, {1.0f, 0.0f}},
        {{ 0.5f,  0.5f,  0.5f}, {1.0f, 1.0f}},
        {{-0.5f,  0.5f,  0.5f}, {0.0f, 1.0f}},

        // Back (-Z)
        {{ 0.5f, -0.5f, -0.5f}, {0.0f, 0.0f}},
        {{-0.5f, -0.5f, -0.5f}, {1.0f, 0.0f}},
        {{-0.5f,  0.5f, -0.5f}, {1.0f, 1.0f}},
        {{ 0.5f,  0.5f, -0.5f}, {0.0f, 1.0f}},

        // Left (-X)
        {{-0.5f, -0.5f, -0.5f}, {0.0f, 0.0f}},
        {{-0.5f, -0.5f,  0.5f}, {1.0f, 0.0f}},
        {{-0.5f,  0.5f,  0.5f}, {1.0f, 1.0f}},
        {{-0.5f,  0.5f, -0.5f}, {0.0f, 1.0f}},

        // Right (+X)
        {{ 0.5f, -0.5f,  0.5f}, {0.0f, 0.0f}},
        {{ 0.5f, -0.5f, -0.5f}, {1.0f, 0.0f}},
        {{ 0.5f,  0.5f, -0.5f}, {1.0f, 1.0f}},
        {{ 0.5f,  0.5f,  0.5f}, {0.0f, 1.0f}},

        // Top (+Y)
        {{-0.5f,  0.5f,  0.5f}, {0.0f, 0.0f}},
        {{ 0.5f,  0.5f,  0.5f}, {1.0f, 0.0f}},
        {{ 0.5f,  0.5f, -0.5f}, {1.0f, 1.0f}},
        {{-0.5f,  0.5f, -0.5f}, {0.0f, 1.0f}},

        // Bottom (-Y)
        {{-0.5f, -0.5f, -0.5f}, {0.0f, 0.0f}},
        {{ 0.5f, -0.5f, -0.5f}, {1.0f, 0.0f}},
        {{ 0.5f, -0.5f,  0.5f}, {1.0f, 1.0f}},
        {{-0.5f, -0.5f,  0.5f}, {0.0f, 1.0f}}
    };

    /*
        OpenGL은 Triangle을 기본 단위로 그린다.

        한 Face는 사각형이므로 Triangle 두 개가 필요하다.

            3 ------ 2
            |      / |
            |    /   |
            |  /     |
            |/       |
            0 ------ 1

        Triangle 1 = 0, 1, 2
        Triangle 2 = 2, 3, 0

        Cube 전체:
            6 Face
            × 2 Triangle
            × 3 Index
            = 36 Index
    */

    const uint32_t indices[] = {
         0,  1,  2,   2,  3,  0,   // Front
         4,  5,  6,   6,  7,  4,   // Back
         8,  9, 10,  10, 11,  8,   // Left
        12, 13, 14,  14, 15, 12,   // Right
        16, 17, 18,  18, 19, 16,   // Top
        20, 21, 22,  22, 23, 20    // Bottom
    };

    return std::make_unique<Mesh>(
        vertices,
        24,
        indices,
        36
    );
}

std::unique_ptr<Mesh> Mesh::LoadObj(const std::string& path)
{
    std::ifstream input(path);
    if (!input)
    {
        throw std::runtime_error("Unable to open OBJ mesh: " + path);
    }

    std::vector<glm::vec3> positions;
    std::vector<glm::vec2> texCoords;
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;
    std::string line;
    std::size_t lineNumber = 0U;

    const auto resolveIndex = [](int objIndex, std::size_t size) -> std::size_t
    {
        // OBJ index 1 is the first item; negative index -1 means the latest
        // previously declared item. Index 0 is forbidden by the format.
        const long resolved = objIndex > 0 ? static_cast<long>(objIndex - 1)
                                           : static_cast<long>(size) + objIndex;
        if (objIndex == 0 || resolved < 0 || resolved >= static_cast<long>(size))
        {
            throw std::runtime_error("OBJ index is outside its declared vertex list");
        }
        return static_cast<std::size_t>(resolved);
    };

    while (std::getline(input, line))
    {
        ++lineNumber;
        std::istringstream row(line);
        std::string keyword;
        row >> keyword;
        if (keyword.empty() || keyword.front() == '#') { continue; }
        if (keyword == "v")
        {
            glm::vec3 position{};
            if (!(row >> position.x >> position.y >> position.z))
            {
                throw std::runtime_error("Invalid OBJ position at line " + std::to_string(lineNumber));
            }
            positions.push_back(position);
        }
        else if (keyword == "vt")
        {
            glm::vec2 uv{};
            if (!(row >> uv.x >> uv.y))
            {
                throw std::runtime_error("Invalid OBJ texture coordinate at line " + std::to_string(lineNumber));
            }
            texCoords.push_back(uv);
        }
        else if (keyword == "f")
        {
            std::vector<Vertex> face;
            std::string token;
            while (row >> token)
            {
                const std::size_t firstSlash = token.find('/');
                const std::string positionPart = token.substr(0U, firstSlash);
                int positionIndex = 0;
                try { positionIndex = std::stoi(positionPart); }
                catch (...) { throw std::runtime_error("Invalid OBJ face position at line " + std::to_string(lineNumber)); }
                Vertex vertex{};
                vertex.position = positions.at(resolveIndex(positionIndex, positions.size()));
                if (firstSlash != std::string::npos)
                {
                    const std::size_t secondSlash = token.find('/', firstSlash + 1U);
                    const std::string uvPart = token.substr(firstSlash + 1U, secondSlash - firstSlash - 1U);
                    if (!uvPart.empty())
                    {
                        int uvIndex = 0;
                        try { uvIndex = std::stoi(uvPart); }
                        catch (...) { throw std::runtime_error("Invalid OBJ face UV at line " + std::to_string(lineNumber)); }
                        vertex.texCoord = texCoords.at(resolveIndex(uvIndex, texCoords.size()));
                    }
                }
                face.push_back(vertex);
            }
            if (face.size() < 3U)
            {
                throw std::runtime_error("OBJ face needs at least three vertices at line " + std::to_string(lineNumber));
            }
            // FreeCAD export should already triangulate. Fan triangulation keeps
            // simple convex n-gons usable while avoiding a runtime CAD dependency.
            for (std::size_t i = 1U; i + 1U < face.size(); ++i)
            {
                const std::uint32_t first = static_cast<std::uint32_t>(vertices.size());
                vertices.push_back(face[0U]);
                vertices.push_back(face[i]);
                vertices.push_back(face[i + 1U]);
                indices.push_back(first);
                indices.push_back(first + 1U);
                indices.push_back(first + 2U);
            }
        }
    }
    if (vertices.empty())
    {
        throw std::runtime_error("OBJ contains no renderable faces: " + path);
    }
    return std::make_unique<Mesh>(vertices.data(), static_cast<std::uint32_t>(vertices.size()),
                                  indices.data(), static_cast<std::uint32_t>(indices.size()));
}

std::vector<StaticGlbMesh> Mesh::LoadStaticGlb(const std::string& path)
{
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input)
    {
        throw std::runtime_error("Unable to open GLB mesh: " + path);
    }
    const std::streamoff byteCount = input.tellg();
    if (byteCount < 20 || static_cast<unsigned long long>(byteCount)
                            > std::numeric_limits<std::size_t>::max())
    {
        throw std::runtime_error("GLB has an invalid file size: " + path);
    }
    std::vector<std::uint8_t> file(static_cast<std::size_t>(byteCount));
    input.seekg(0);
    if (!input.read(reinterpret_cast<char*>(file.data()), static_cast<std::streamsize>(file.size())))
    {
        throw std::runtime_error("Unable to read complete GLB mesh: " + path);
    }

    if (ReadLittleEndian32(file, 0U) != kGlbMagic || ReadLittleEndian32(file, 4U) != kGlbVersion2
        || ReadLittleEndian32(file, 8U) != file.size())
    {
        throw std::runtime_error("GLB must be a complete version-2 binary container: " + path);
    }
    const std::size_t jsonLength = ReadLittleEndian32(file, 12U);
    if (ReadLittleEndian32(file, 16U) != kJsonChunk)
    {
        throw std::runtime_error("GLB first chunk is not JSON: " + path);
    }
    RequireRange(20U, jsonLength, file.size(), "JSON chunk");
    const std::size_t binaryHeader = 20U + jsonLength;
    if (binaryHeader > file.size() || file.size() - binaryHeader < 8U)
    {
        throw std::runtime_error("GLB has no BIN chunk: " + path);
    }
    const std::size_t binaryLength = ReadLittleEndian32(file, binaryHeader);
    if (ReadLittleEndian32(file, binaryHeader + 4U) != kBinaryChunk)
    {
        throw std::runtime_error("GLB second chunk is not BIN: " + path);
    }
    const std::size_t binaryOffset = binaryHeader + 8U;
    RequireRange(binaryOffset, binaryLength, file.size(), "BIN chunk");
    if (binaryOffset + binaryLength != file.size())
    {
        throw std::runtime_error("GLB exporter profile permits exactly one JSON and one BIN chunk");
    }

    const std::string json(reinterpret_cast<const char*>(file.data() + 20U), jsonLength);
    const std::vector<GlbBufferView> views = ParseBufferViews(json);
    const std::vector<GlbAccessor> accessors = ParseAccessors(json);
    const std::vector<GlbMeshDescription> descriptions = ParseMeshes(json);
    if (views.empty() || accessors.empty() || descriptions.empty())
    {
        throw std::runtime_error("GLB does not match the required static HCR mesh profile: " + path);
    }

    std::vector<StaticGlbMesh> result;
    result.reserve(descriptions.size());
    for (const GlbMeshDescription& description : descriptions)
    {
        if (description.positionAccessor >= accessors.size() || description.normalAccessor >= accessors.size()
            || description.indexAccessor >= accessors.size())
        {
            throw std::runtime_error("GLB mesh references a missing accessor: " + description.name);
        }
        const GlbAccessor& position = accessors[description.positionAccessor];
        const GlbAccessor& normal = accessors[description.normalAccessor];
        const GlbAccessor& index = accessors[description.indexAccessor];
        if (position.componentType != kFloatComponent || position.type != "VEC3"
            || normal.componentType != kFloatComponent || normal.type != "VEC3"
            || normal.count != position.count || index.componentType != kUnsignedIntComponent
            || index.type != "SCALAR" || index.count % 3U != 0U
            || position.bufferView >= views.size() || normal.bufferView >= views.size()
            || index.bufferView >= views.size() || position.count > std::numeric_limits<std::uint32_t>::max()
            || index.count > std::numeric_limits<std::uint32_t>::max())
        {
            throw std::runtime_error("GLB mesh has unsupported vertex/index layout: " + description.name);
        }
        const GlbBufferView& positionView = views[position.bufferView];
        const GlbBufferView& normalView = views[normal.bufferView];
        const GlbBufferView& indexView = views[index.bufferView];
        if (positionView.target != 34962U || normalView.target != 34962U || indexView.target != 34963U
            || position.count > std::numeric_limits<std::size_t>::max() / 12U
            || index.count > std::numeric_limits<std::size_t>::max() / 4U
            || positionView.byteLength != position.count * 12U || normalView.byteLength != normal.count * 12U
            || indexView.byteLength != index.count * 4U)
        {
            throw std::runtime_error("GLB mesh has invalid tightly packed buffer views: " + description.name);
        }
        RequireRange(positionView.byteOffset, positionView.byteLength, binaryLength, "POSITION buffer view");
        RequireRange(normalView.byteOffset, normalView.byteLength, binaryLength, "NORMAL buffer view");
        RequireRange(indexView.byteOffset, indexView.byteLength, binaryLength, "index buffer view");

        std::vector<Vertex> vertices(position.count);
        for (std::size_t vertex = 0U; vertex < position.count; ++vertex)
        {
            const std::size_t offset = binaryOffset + positionView.byteOffset + vertex * 12U;
            vertices[vertex].position = {ReadLittleEndianFloat(file, offset),
                                         ReadLittleEndianFloat(file, offset + 4U),
                                         ReadLittleEndianFloat(file, offset + 8U)};
            // GLB CAD geometry has no texture coordinates. The existing shader
            // samples its diagnostic texture uniformly, so a zero UV is the
            // explicit, stable substitute rather than uninitialized memory.
            vertices[vertex].texCoord = {0.0F, 0.0F};
        }
        std::vector<std::uint32_t> indices(index.count);
        for (std::size_t element = 0U; element < index.count; ++element)
        {
            const std::uint32_t decoded = ReadLittleEndian32(
                file, binaryOffset + indexView.byteOffset + element * 4U);
            if (decoded >= position.count)
            {
                throw std::runtime_error("GLB index exceeds POSITION count: " + description.name);
            }
            indices[element] = decoded;
        }
        result.push_back({description.name,
            std::make_shared<Mesh>(vertices.data(), static_cast<std::uint32_t>(vertices.size()),
                                   indices.data(), static_cast<std::uint32_t>(indices.size()))});
    }
    return result;
}
} // namespace PoseLink
