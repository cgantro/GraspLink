#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include <glm/glm.hpp>

/**
 * @brief GLSL source를 컴파일/링크하고 OpenGL Program과 Uniform을 관리한다.
 *
 * @details
 * 하나의 .glsl 파일 안에서 `#type vertex`, `#type fragment` 구역을 나누는 프로젝트 전용 형식을 지원한다.
 * OpenGL에서 vertex/fragment shader는 각각 컴파일한 뒤 하나의 Program으로 link해야 실제 draw에 사용할 수 있다.
 * Uniform location은 문자열 검색 비용을 줄이기 위해 캐시한다.
 *
 * @todo [FUTURE] geometry/compute shader stage가 필요해지면 PreProcess/Compile stage map을 확장한다.
 */
class Shader
{
public:
    /** @brief 프로젝트 형식의 Shader 파일을 읽어 OpenGL Program을 생성한다. */
    explicit Shader(const std::string& filepath);

    /** @brief 메모리에 있는 vertex/fragment GLSL source로 Program을 생성한다. */
    Shader(
        const std::string& name,
        const std::string& vertexSrc,
        const std::string& fragmentSrc);

    /** @brief 소유한 OpenGL Program을 삭제한다. */
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    /** @brief 파일 기반 Shader를 shared_ptr로 생성한다. */
    static std::shared_ptr<Shader> Create(const std::string& filepath);

    /** @brief source string 기반 Shader를 shared_ptr로 생성한다. */
    static std::shared_ptr<Shader> CreateFromSource(
        const std::string& name,
        const std::string& vertexSrc,
        const std::string& fragmentSrc);

    /** @brief glUseProgram으로 이 Program을 활성화한다. */
    void Bind() const;

    /** @brief 현재 OpenGL Program binding을 해제한다. */
    void UnBind() const;

    /** @brief int uniform을 설정한다. */
    void SetInt(const std::string& name, int value);
    /** @brief int array uniform을 설정한다. */
    void SetIntArray(const std::string& name, int* value, uint32_t count);
    /** @brief float uniform을 설정한다. */
    void SetFloat(const std::string& name, float value);
    /** @brief vec2 uniform을 설정한다. */
    void SetFloat2(const std::string& name, const glm::vec2& value);
    /** @brief vec3 uniform을 설정한다. */
    void SetFloat3(const std::string& name, const glm::vec3& value);
    /** @brief vec4 uniform을 설정한다. */
    void SetFloat4(const std::string& name, const glm::vec4& value);
    /** @brief mat3 uniform을 설정한다. */
    void SetMat3(const std::string& name, const glm::mat3& matrix);
    /** @brief mat4 uniform을 설정한다. */
    void SetMat4(const std::string& name, const glm::mat4& matrix);

    /** @brief 디버깅용 Shader 이름을 반환한다. */
    const std::string& GetName() const { return m_Name; }

private:
    /** @brief Shader 파일 전체를 문자열로 읽는다. */
    std::string ReadFile(const std::string& filepath);

    /** @brief `#type` 구역을 OpenGL shader stage별 source로 분리한다. */
    std::unordered_map<unsigned int, std::string> PreProcess(const std::string& source);

    /** @brief 각 stage를 컴파일하고 하나의 Program으로 링크한다. */
    void Compile(const std::unordered_map<unsigned int, std::string>& shaderSources);

    /** @brief Uniform location을 조회하고 이름별 cache에 저장한다. */
    int GetUniformLocation(const std::string& name) const;

private:
    uint32_t m_RendererID = 0;
    std::string m_Name;
    mutable std::unordered_map<std::string, int> m_UniformLocationCache;
};
