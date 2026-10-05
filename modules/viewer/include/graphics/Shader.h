#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include <glm/glm.hpp>

/**
 * @brief GLSL stage를 컴파일하고 하나의 OpenGL Program으로 관리한다.
 * @details
 * 파일 생성자는 `#type vertex`와 `#type fragment` 구역을 분리하고, 각 구역을 독립적으로
 * 컴파일한 다음 같은 Program에 연결해 링크한다. 소스 생성자는 이미 분리된 두 문자열을 받는다.
 * 링크가 성공하면 개별 stage 객체는 해제되고 실행 가능한 Program만 이 객체가 소유한다.
 * 렌더 호출부는 먼저 Bind한 뒤 uniform 값을 설정하고 draw를 수행한다. Uniform 위치는 이름별로
 * 캐시되며, sampler uniform에는 texture 객체 자체가 아니라 활성 texture unit 번호를 전달한다.
 * OpenGL 객체를 만들고 사용하는 동안 유효한 GL context가 현재 스레드에 있어야 한다.
 */
class Shader
{
public:
    /**
     * @brief `#type` 구역이 들어 있는 shader 파일을 읽어 Program을 만든다.
     * @param filepath 파일 경로. 확장자나 파일명은 stage를 결정하지 않는다.
     * @throws std::runtime_error 파일 열기, stage 이름 해석, 컴파일 또는 링크 실패 시 발생한다.
     */
    explicit Shader(const std::string& filepath);

    /**
     * @brief 분리된 vertex와 fragment source로 Program을 만든다.
     * @param name 로그와 식별에 사용할 이름
     * @param vertexSrc vertex stage의 GLSL source
     * @param fragmentSrc fragment stage의 GLSL source
     * @throws std::runtime_error stage 컴파일 또는 Program 링크 실패 시 발생한다.
     */
    Shader(
        const std::string& name,
        const std::string& vertexSrc,
        const std::string& fragmentSrc);

    /** @brief 소유한 OpenGL Program을 삭제한다. */
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    /**
     * @brief 파일을 읽어 공유 가능한 Shader 객체를 만든다.
     * @param filepath `#type` 구역을 포함한 shader 파일 경로
     * @return Program을 소유하는 shared pointer
     * @throws std::runtime_error 파일 읽기, 컴파일 또는 링크 실패 시 전달된다.
     */
    static std::shared_ptr<Shader> Create(const std::string& filepath);

    /**
     * @brief 분리된 vertex와 fragment source로 공유 가능한 Shader 객체를 만든다.
     * @param name Shader 식별 이름
     * @param vertexSrc vertex stage의 GLSL source
     * @param fragmentSrc fragment stage의 GLSL source
     * @return Program을 소유하는 shared pointer
     * @throws std::runtime_error 컴파일 또는 링크 실패 시 전달된다.
     */
    static std::shared_ptr<Shader> CreateFromSource(
        const std::string& name,
        const std::string& vertexSrc,
        const std::string& fragmentSrc);

    /**
     * @brief 이 Program을 현재 렌더링 대상으로 선택한다.
     * @details 이후 uniform 설정과 draw가 끝날 때까지 이 Program이 활성 상태여야 한다.
     */
    void Bind() const;

    /** @brief 현재 Program 선택을 해제하고 OpenGL의 기본 Program 상태(0)로 돌린다. */
    void UnBind() const;

    /** @brief 정수 uniform을 설정한다. sampler에는 texture unit 번호를 전달한다. */
    void SetInt(const std::string& name, int value);
    /** @brief 연속된 정수 uniform 배열을 설정한다. count는 정수 원소 수다. */
    void SetIntArray(const std::string& name, int* value, uint32_t count);
    /** @brief 실수 uniform을 설정한다. */
    void SetFloat(const std::string& name, float value);
    /** @brief 2성분 실수 uniform을 설정한다. */
    void SetFloat2(const std::string& name, const glm::vec2& value);
    /** @brief 3성분 실수 uniform을 설정한다. */
    void SetFloat3(const std::string& name, const glm::vec3& value);
    /** @brief 4성분 실수 uniform을 설정한다. */
    void SetFloat4(const std::string& name, const glm::vec4& value);
    /** @brief 전치하지 않는 3×3 행렬 uniform을 설정한다. */
    void SetMat3(const std::string& name, const glm::mat3& matrix);
    /** @brief 전치하지 않는 4×4 행렬 uniform을 설정한다. */
    void SetMat4(const std::string& name, const glm::mat4& matrix);

    /** @brief 생성 시 지정한 식별 이름을 반환한다. */
    const std::string& GetName() const { return m_Name; }

private:
    std::string ReadFile(const std::string& filepath);

    // 파일의 #type 구역을 OpenGL stage별 source로 분리한다.
    std::unordered_map<unsigned int, std::string> PreProcess(const std::string& source);

    // 실패하면 생성한 stage와 Program을 정리한 뒤 예외를 던진다.
    void Compile(const std::unordered_map<unsigned int, std::string>& shaderSources);

    int GetUniformLocation(const std::string& name) const;

private:
    // Compile 성공 뒤 링크된 Program ID. Shader가 소유한다.
    uint32_t m_RendererID = 0;

    std::string m_Name;

    mutable std::unordered_map<std::string, int> m_UniformLocationCache;
};
