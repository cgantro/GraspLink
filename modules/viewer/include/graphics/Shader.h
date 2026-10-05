#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include <glm/glm.hpp>

// #type 구역을 stage별로 나누고 컴파일한 뒤 하나의 Program으로 링크한다.
class Shader
{
public:
    explicit Shader(const std::string& filepath);

    Shader(
        const std::string& name,
        const std::string& vertexSrc,
        const std::string& fragmentSrc);

    // 성공한 Compile이 만든 Program은 Shader가 소유하고 소멸 때 삭제한다.
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    static std::shared_ptr<Shader> Create(const std::string& filepath);

    static std::shared_ptr<Shader> CreateFromSource(
        const std::string& name,
        const std::string& vertexSrc,
        const std::string& fragmentSrc);

    void Bind() const;

    void UnBind() const;

    void SetInt(const std::string& name, int value);
    void SetIntArray(const std::string& name, int* value, uint32_t count);
    void SetFloat(const std::string& name, float value);
    void SetFloat2(const std::string& name, const glm::vec2& value);
    void SetFloat3(const std::string& name, const glm::vec3& value);
    void SetFloat4(const std::string& name, const glm::vec4& value);
    void SetMat3(const std::string& name, const glm::mat3& matrix);
    void SetMat4(const std::string& name, const glm::mat4& matrix);

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
