#include "Shader.h"

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <filesystem>



// #type 구역 이름을 OpenGL stage로 바꾼다.
static GLenum ShaderTypeFromString(const std::string& type){
    if(type == "vertex") return GL_VERTEX_SHADER;
    if(type == "fragment" || type == "pixel") return GL_FRAGMENT_SHADER;
    throw std::runtime_error(
        "Unknown Shader Type: " + type
    );
}

Shader::Shader(const std::string& filepath){
    const std::string src = ReadFile(filepath);

    // 파일의 #type 구역을 stage별 source로 나눈다.
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
    // #type 구역을 포함한 GLSL 원문을 그대로 읽는다.
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

    while(std::getline(stream,line)){
        // Windows 줄바꿈에서 남을 수 있는 '\r'을 제거한다.
        if(!line.empty() && line.back() == '\r') line.pop_back();

        // 새 #type 구역에서 앞 구역을 저장하고 source 수집을 시작한다.
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
    // 마지막 #type 구역은 EOF에서 저장한다.
    if(currentType != 0) shaderSources[currentType] = currentSource.str();
    
    if(shaderSources.empty()) throw std::runtime_error("No Shader Source found.");

    return shaderSources;
}

void Shader::Compile(const std::unordered_map<unsigned int, std::string>& shaderSources){
    GLuint program = glCreateProgram();
    if(program == 0) throw std::runtime_error("Failed to create Shader Program");

    // 실패 때 이미 만든 stage와 Program을 정리한다.
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

    // 모든 stage를 Program에 연결한 뒤 링크한다.
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

    // 링크된 Program만 남기고 stage 객체를 해제한다.
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

    int location = glGetUniformLocation(m_RendererID,name.c_str());
    if(location == -1) std::cout << "Warning: Uniform " << name << "'doesn't exist\n";

    m_UniformLocationCache[name] = location;

    return location;
}


void Shader::SetInt(const std::string& name, int value){glUniform1i(GetUniformLocation(name),value);}
void Shader::SetIntArray(const std::string& name, int* values, uint32_t cnt){glUniform1iv(GetUniformLocation(name),static_cast<GLsizei>(cnt),values);}
void Shader::SetFloat(const std::string& name, float value){glUniform1f(GetUniformLocation(name),value);}
void Shader::SetFloat2(const std::string& name, const glm::vec2& value){glUniform2fv(GetUniformLocation(name),1,glm::value_ptr(value));}
void Shader::SetFloat3(const std::string& name, const glm::vec3& value){glUniform3fv(GetUniformLocation(name),1,glm::value_ptr(value));}
void Shader::SetFloat4(const std::string& name, const glm::vec4& value){glUniform4fv(GetUniformLocation(name),1,glm::value_ptr(value));}
void Shader::SetMat3(const std::string& name, const glm::mat3& value){glUniformMatrix3fv(GetUniformLocation(name),1,GL_FALSE,glm::value_ptr(value));}
void Shader::SetMat4(const std::string& name, const glm::mat4& value){glUniformMatrix4fv(GetUniformLocation(name),1,GL_FALSE,glm::value_ptr(value));}
