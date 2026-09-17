#pragma once

#include <string>
#include <cstdint>
#include <memory>
#include <unordered_map>

#include <glm/glm.hpp>

namespace PoseLink {

class Shader{
public:
    // 파일 기반 Shader 생성
    // 하나의 Shader 파일 안에서
    // #type vertex
    // #type fragment 영역을 찾아 각각 컴파일
    // 예: Shader shader("assets/shaders/Model.glsl");
    Shader(const std::string& filepath);

    // Source Code 기반 Shader 생성
    // 파일을 사용하지 않고,Vertex / Fragment Shader 코드를 std::string으로 직접 전달할 때 사용
    Shader(
        const std::string& name, 
        const std::string& vertexSrc, 
        const std::string& fragmentSrc
    );

    // Shader 프로그램은 GPU 자원 -> 객체 파괴시 정리
    ~Shader();

    // 복사 금지
    // Shader 객체 여러 개가 같은 m_RendererID를 복사해서 가진다면, 소멸 시 같은 프로그램을 두 번 삭제 가능
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    // Factory 함수

    // 파일 기반 Shader 객체 생성
    static std::shared_ptr<Shader> Create(
        const std::string& filepath
    );

    static std::shared_ptr<Shader> CreateFromSource(
        const std::string& name,
        const std::string& vertexSrc, 
        const std::string& fragmentSrc
    );

    // Shader 프로그램 사용
    // 내부적으로 glUseProgram(m_RendererID);
    void Bind() const;
    // Shader 프로그램 해제
    // 내부적으로 glUseProgram(0);
    void UnBind() const;

    // Uniform Setter
    // GLSL:
    // uniform int u_Texture;
    // uniform float u_Opacity;
    // uniform vec3 u_Color;
    // 같은 값을 CPU -> GPU로 전달하기 위한 함수들

    void SetInt(const std::string& name, int value);
    void SetIntArray(const std::string& name, int* value, uint32_t count);
    void SetFloat(const std::string& name, float value);
    void SetFloat2(const std::string& name, const glm::vec2& value);
    void SetFloat3(const std::string& name, const glm::vec3& value);
    void SetFloat4(const std::string& name, const glm::vec4& value);
    
    void SetMat3(const std::string& name, const glm::mat3& matrix);
    void SetMat4(const std::string& name, const glm::mat4& matrix);

    // Shader의 이름 반환.
    //
    // 예:
    // assets/shaders/Model.glsl
    //
    // → "Quad"
    const std::string& GetName() const {return m_Name;}

private:

    // Shader 파일을 읽어서 전체 내용을 std::string으로 반환한다.
    std::string ReadFile(const std::string& filepath);

    // Shader Source 분리
    // #type vertex
    // ...
    // #type fragment
    // ...
    // 를 읽어서:
    // GL_VERTEX_SHADER   → vertex source
    // GL_FRAGMENT_SHADER → fragment source
    // 형태로 분리한다.

    std::unordered_map<unsigned int, std::string> PreProcess(const std::string& source);

    // Shader 컴파일 + 프로그램 링크
    // 셰이더소스안에 들어있는 각 Shader Stage를 컴파일하고, 하나의 OpenGL 프로그램으로 링크한다.
    void Compile(const std::unordered_map<unsigned int, std::string>& shaderSources);
    
    // Uniform Location 조회
    // glGetUniformLocation() 결과를 캐시에 저장한다.
    // 같은 uniform을 매 Frame마다 다시 검색하지 않기 위함
    // uniform이란 셰이더 외부에서 셰이더 프로그램에 전달해주는 변수
    // CPU->GPU로 정보를 넘겨주는 변수(한 번 정해지면 바뀌지 않는 변수)

    int GetUniformLocation(const std::string& name) const;
private:
    // 셰이더 프로그램 ID
    // glCreateProgram()의 반환값
    uint32_t m_RendererID = 0;

    // Debug 관리용 Shader 이름
    std::string m_Name;

    // Uniform 이름 -> GL UniformLocation
    // "u_texture" -> 0
    // mutable인 이유:
    // GetUniformLocation()이 const함수이지만, Cache 자체는 갱신할 수 있게 하기 위해서
    mutable std::unordered_map<std::string, int> m_UniformLocationCache;
};

} // namespace PoseLink
