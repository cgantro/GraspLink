#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include <glm/glm.hpp>

/**
 * @brief GLSL source를 컴파일/링크하고 OpenGL Program과 Uniform 설정을 관리한다.
 *
 * @details
 * 프로젝트 전용 단일 `.glsl` 파일 형식에서 `#type vertex`, `#type fragment` 구역을 분리해 각 stage를 컴파일한다.
 * Uniform location은 문자열 이름 -> GLint location map으로 cache하여 반복 glGetUniformLocation 호출을 줄인다.
 *
 * Matrix/Vector 값은 GLM 타입을 그대로 받으며 단위 변환을 수행하지 않는다. 예를 들어 model matrix의 translation이
 * meter인지 여부는 상위 Scene/Transform 규칙에 의해 결정된다.
 *
 * @todo [FUTURE] geometry/compute shader stage가 필요해지면 PreProcess/Compile stage map을 확장한다.
 */
class Shader
{
public:
    /**
     * @brief 프로젝트 형식의 Shader 파일을 읽어 OpenGL Program을 생성한다.
     * @param filepath `#type` section을 포함한 GLSL 파일 경로.
     */
    explicit Shader(const std::string& filepath);

    /**
     * @brief 메모리에 있는 vertex/fragment GLSL source로 Program을 생성한다.
     * @param name 디버깅/진단용 shader 이름.
     * @param vertexSrc vertex shader GLSL source.
     * @param fragmentSrc fragment shader GLSL source.
     */
    Shader(
        const std::string& name,
        const std::string& vertexSrc,
        const std::string& fragmentSrc);

    /** @brief 소유한 OpenGL Program object를 삭제한다. */
    ~Shader();

    /** @brief OpenGL Program object ownership 중복을 막기 위해 copy construction을 금지한다. */
    Shader(const Shader&) = delete;

    /** @brief OpenGL Program object ownership 중복을 막기 위해 copy assignment를 금지한다. */
    Shader& operator=(const Shader&) = delete;

    /** @brief 파일 기반 Shader를 shared_ptr로 생성한다. */
    static std::shared_ptr<Shader> Create(const std::string& filepath);

    /** @brief source string 기반 Shader를 shared_ptr로 생성한다. */
    static std::shared_ptr<Shader> CreateFromSource(
        const std::string& name,
        const std::string& vertexSrc,
        const std::string& fragmentSrc);

    /** @brief glUseProgram으로 이 Program을 현재 render state에 활성화한다. */
    void Bind() const;

    /** @brief 현재 OpenGL Program binding을 0으로 해제한다. */
    void UnBind() const;

    /** @brief scalar int uniform 하나를 설정한다. */
    void SetInt(const std::string& name, int value);

    /**
     * @brief int array uniform을 설정한다.
     * @param value 연속 int 배열 시작 주소.
     * @param count 배열 원소 개수.
     */
    void SetIntArray(const std::string& name, int* value, uint32_t count);

    /** @brief scalar float uniform을 설정한다. */
    void SetFloat(const std::string& name, float value);

    /** @brief vec2 uniform을 설정한다. */
    void SetFloat2(const std::string& name, const glm::vec2& value);

    /** @brief vec3 uniform을 설정한다. */
    void SetFloat3(const std::string& name, const glm::vec3& value);

    /** @brief vec4 uniform을 설정한다. */
    void SetFloat4(const std::string& name, const glm::vec4& value);

    /** @brief 3x3 matrix uniform을 설정한다. */
    void SetMat3(const std::string& name, const glm::mat3& matrix);

    /** @brief 4x4 matrix uniform을 설정한다. */
    void SetMat4(const std::string& name, const glm::mat4& matrix);

    /** @return 디버깅용 Shader 이름. */
    const std::string& GetName() const { return m_Name; }

private:
    /** @brief Shader 파일 전체를 UTF-8 문자열로 읽는다. */
    std::string ReadFile(const std::string& filepath);

    /** @brief `#type` section을 OpenGL shader stage별 source map으로 분리한다. */
    std::unordered_map<unsigned int, std::string> PreProcess(const std::string& source);

    /** @brief 각 stage를 컴파일한 뒤 하나의 OpenGL Program으로 링크한다. */
    void Compile(const std::unordered_map<unsigned int, std::string>& shaderSources);

    /**
     * @brief Uniform location을 조회하고 이름별 cache에 저장한다.
     * @return OpenGL uniform location. 존재하지 않거나 최적화로 제거된 uniform은 음수일 수 있다.
     */
    int GetUniformLocation(const std::string& name) const;

private:
    /** @brief OpenGL Program object ID. 0은 생성 전/해제 후 상태. */
    uint32_t m_RendererID = 0;

    /** @brief 파일명 또는 CreateFromSource에서 받은 디버깅용 이름. */
    std::string m_Name;

    /** @brief uniform name -> OpenGL location cache. const setter helper에서도 갱신하므로 mutable. */
    mutable std::unordered_map<std::string, int> m_UniformLocationCache;
};
