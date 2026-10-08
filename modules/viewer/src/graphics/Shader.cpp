#include "graphics/Shader.h"

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <filesystem>

namespace grasplink::graphics
{



// 파일에서 지원하는 #type 이름만 OpenGL Shader 단계로 바꾼다. pixel은 fragment와 같은 단계의 별칭으로 받는다.
static GLenum ShaderTypeFromString(const std::string& type){
    if(type == "vertex") return GL_VERTEX_SHADER;
    if(type == "fragment" || type == "pixel") return GL_FRAGMENT_SHADER;
    throw std::runtime_error(
        "Unknown Shader Type: " + type
    );
}

Shader::Shader(const std::string& filepath){
    const std::string src = ReadFile(filepath);

    // 파일 내용을 #type 표식이 나눈 구간별로 읽어 각 Shader 단계의 소스 문자열을 만든다.
    const auto shaderSources = PreProcess(src);

    Compile(shaderSources);

    m_Name = std::filesystem::path(filepath)
        .stem()
        .string();
}

Shader::Shader(
    const std::string& name,
    const std::string& vertexSrc,
    const std::string& fragmentSrc): m_Name(name){
    std::unordered_map<unsigned int, std::string> srcs;

    srcs[GL_VERTEX_SHADER] = vertexSrc;
    srcs[GL_FRAGMENT_SHADER] = fragmentSrc;

    Compile(srcs);
}

Shader::~Shader(){
    if(m_RendererID != 0){
        glDeleteProgram(m_RendererID);

        m_RendererID = 0;
    }
}

std::shared_ptr<Shader> Shader::Create(const std::string& filepath){
    return std::make_shared<Shader>(filepath);
}

std::shared_ptr<Shader> Shader::CreateFromSource(const std::string& name, const std::string& vertexSrc, const std::string& fragmentSrc)
{
    return std::make_shared<Shader>(name, vertexSrc, fragmentSrc);
}

std::string Shader::ReadFile(const std::string& filepath){
    // Shader 전처리는 호출자가 수행하므로 이 단계에서는 #type 표식까지 원문 그대로 유지한다.
    std::ifstream file(filepath, std::ios::in | std::ios::binary);

    if(!file.is_open()) throw std::runtime_error(
        "Could not open Shader File: " + filepath
    );

    std::stringstream stream;
    stream << file.rdbuf();

    return stream.str();
}

std::unordered_map<unsigned int, std::string> Shader::PreProcess(const std::string& source){
    std::unordered_map<unsigned int, std::string> shaderSources;

    std::istringstream stream(source);
    std::string line;

    GLenum currentType = 0;
    std::stringstream currentSource;

    // 각 표식은 다음 줄부터 특정 Shader 단계의 소스를 모으는 경계다. 첫 표식 앞에 있는 설명문은 단계 소스에 넣지 않는다.
    while(std::getline(stream,line)){
        // Windows 줄바꿈은 '\r\n'으로 끝날 수 있다. 줄 끝에 남은 '\r'을 제거해 표식 문자열을 정확히 비교한다.
        if(!line.empty() && line.back() == '\r') line.pop_back();

        // 새 #type 경계를 만나면 앞 단계의 소스를 저장하고, 이후 줄은 새로 지정된 단계에 모은다.
        if(line.rfind("#type ",0) == 0){
            if(currentType != 0){
                shaderSources[currentType] = currentSource.str();

                currentSource.str("");
                currentSource.clear();
            }

            const std::string type = line.substr(6);
            currentType = ShaderTypeFromString(type);
            continue;
        }
        if(currentType == 0) continue;
        currentSource << line << '\n';
    }
    // 파일 끝(EOF)에 도달하면 마지막으로 읽은 #type 구간의 소스도 저장한다.
    if(currentType != 0) shaderSources[currentType] = currentSource.str();
    
    if(shaderSources.empty()) throw std::runtime_error("No Shader Source found.");

    return shaderSources;
}

void Shader::Compile(const std::unordered_map<unsigned int, std::string>& shaderSources){
    GLuint program = glCreateProgram();
    if(program == 0) throw std::runtime_error("Failed to create Shader Program");

    // Shader 단계 ID를 기록해 실패한 생성 과정에서 해제한다. 예외가 발생해도 링크되지 않은 Program이 남지 않게 한다.
    std::vector<GLuint> shaderIDs;

    for(const auto& [type, source] : shaderSources){
        GLuint shader = glCreateShader(type);
        if(shader == 0){
            for(GLuint id: shaderIDs) glDeleteShader(id);
            glDeleteProgram(program);
            throw std::runtime_error("Failed to Create Shader object");
        }

        const GLchar* sourceCstr = source.c_str();

        glShaderSource(shader,1,&sourceCstr,nullptr);

        glCompileShader(shader);

        GLint isCompiled = GL_FALSE;

        glGetShaderiv(shader,GL_COMPILE_STATUS,&isCompiled);

        if(isCompiled == GL_FALSE){
            GLint logLen = 0;
            glGetShaderiv(shader,GL_INFO_LOG_LENGTH,&logLen);

            std::vector<GLchar> infoLog(logLen);
            glGetShaderInfoLog(shader,logLen,nullptr,infoLog.data());

            glDeleteShader(shader);

            for(GLuint id: shaderIDs) glDeleteShader(id);

            glDeleteProgram(program);

            throw std::runtime_error(std::string("Shader Compile Failed\n") + infoLog.data());

        }

        glAttachShader(program,shader);

        shaderIDs.push_back(shader);
    }

    // 각 단계가 따로 컴파일되어도 서로 연결되지 않을 수 있다. 따라서 모든 단계를 Program으로 묶는 링크 작업을 별도로 확인한다.
    glLinkProgram(program);

    GLint isLinked = GL_FALSE;
    
    glGetProgramiv(program,GL_LINK_STATUS,&isLinked);

    if (isLinked == GL_FALSE){
        GLint logLength = 0;

        glGetProgramiv(
            program,
            GL_INFO_LOG_LENGTH,
            &logLength
        );


        std::vector<GLchar> infoLog(logLength);

        glGetProgramInfoLog(program,logLength,nullptr,infoLog.data());


        for (GLuint id : shaderIDs) glDeleteShader(id);

        glDeleteProgram(program);


        throw std::runtime_error(std::string("Shader Link Failed:\n")+ infoLog.data());
    }

    // 링크가 성공하면 실행에 필요한 코드는 Program에 보존된다. 따라서 컴파일에 쓴 임시 Shader 단계 객체는 Program에서 분리해 해제한다.
    for(GLuint id : shaderIDs){
        glDetachShader(program,id);
        glDeleteShader(id);
    }

    m_RendererID = static_cast<uint32_t>(program);
}

void Shader::Bind() const {glUseProgram(m_RendererID);} 
void Shader::UnBind() const {glUseProgram(0);}

int Shader::GetUniformLocation(const std::string& name)const{
    auto it = m_UniformLocationCache.find(name);
    if(it != m_UniformLocationCache.end()) return it->second;

    // uniform의 위치는 링크된 Program 안에서 고정된다. 찾은 위치와 존재하지 않음을 뜻하는 -1을 모두 이름별로 저장해 반복 조회를 피한다.
    int location = glGetUniformLocation(m_RendererID,name.c_str());
    if(location == -1) std::cout << "Warning: Uniform " << name << "'doesn't exist\n";

    m_UniformLocationCache[name] = location;

    return location;
}


// 각 Set 함수는 값을 대응하는 OpenGL uniform 자료형으로 전달한다. 값을 쓰기 전에 Program을 Bind해야 하며 행렬은 전치 없이 보낸다.
void Shader::SetInt(const std::string& name, int value){glUniform1i(GetUniformLocation(name),value);}
void Shader::SetIntArray(const std::string& name, int* values, uint32_t cnt){glUniform1iv(GetUniformLocation(name),static_cast<GLsizei>(cnt),values);}
void Shader::SetFloat(const std::string& name, float value){glUniform1f(GetUniformLocation(name),value);}
void Shader::SetFloat2(const std::string& name, const glm::vec2& value){glUniform2fv(GetUniformLocation(name),1,glm::value_ptr(value));}
void Shader::SetFloat3(const std::string& name, const glm::vec3& value){glUniform3fv(GetUniformLocation(name),1,glm::value_ptr(value));}
void Shader::SetFloat4(const std::string& name, const glm::vec4& value){glUniform4fv(GetUniformLocation(name),1,glm::value_ptr(value));}
void Shader::SetMat3(const std::string& name, const glm::mat3& value){glUniformMatrix3fv(GetUniformLocation(name),1,GL_FALSE,glm::value_ptr(value));}
void Shader::SetMat4(const std::string& name, const glm::mat4& value){glUniformMatrix4fv(GetUniformLocation(name),1,GL_FALSE,glm::value_ptr(value));}

} // namespace grasplink::graphics
