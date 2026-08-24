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

// #type vertex와 같은 문자열을 OpenGL Shader Type으로 변환
static GLenum ShaderTypeFromString(const std::string& type){
    if(type == "vertex") return GL_VERTEX_SHADER;
    if(type == "fragment" || type == "pixel") return GL_FRAGMENT_SHADER;
    // 기존엔 잘못된 타입이면 0을 반환 했으나, glCreateShader(0)이 호출될 수 있다
    // 디버깅 어려움
    // Shader 자체가 잘못된 것이기 때문에, 예외 발생
    throw std::runtime_error(
        "Unknown Shader Type: " + type
    );
}

Shader::Shader(const std::string& filepath){
    // 파일 읽기
    const std::string src = ReadFile(filepath);

    // 하나의 문자열을
    // vertex
    // fragment
    // source로 분리한다.
    const auto shaderSources = PreProcess(src);

    // 실제 OpenGL Shader 생성 / Compile / Link
    Compile(shaderSources);

    // 기존엔 find_last_of, rfind()를 직접 사용했으나
    // C++의 filesystem을 사용한다.
    m_Name = std::filesystem::path(filepath)
        .stem() // 확장자를 제외한 파일 이름
        .string();
}

Shader::Shader(
    const std::string& name,
    const std::string& vertexSrc,
    const std::string& fragmentSrc): m_Name(name){
    // 이미 Vertex/Fragment가 전처리 됨
    std::unordered_map<unsigned int, std::string> srcs;

    srcs[GL_VERTEX_SHADER] = vertexSrc;
    srcs[GL_FRAGMENT_SHADER] = fragmentSrc;

    Compile(srcs);
}

// 소멸자
Shader::~Shader(){
    if(m_RendererID != 0){
        // Shader Program이 생성된 경우만 삭제한다.
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
    // binary로 연다. 그대로 읽고 가공한다.
    // 텍스트 파일이 아닌 다른 이진 데이터를 읽을 때 필수적
    // 1. 줄바꿈 문자 변환 방지
    // Windows에서는 텍스트 모드로 파일을 읽을 때 \r\n(CRLF)을 프로그램 내부로 가져오면서 \n(LF)으로 자동 변환한다.
    // 이진을 켜면, 이런 변환을 방지하고, 기록된 바이트 그대로(0x0D, 0x0A) 읽는다.
    // 2. 파일 끝(EOF) 오작동 방지
    // 일부 시스템의 텍스트 모드에서는 특정 바이트(0x1A/Ctrl+Z)를 만나면, 크기와 상관없이 끝으로 오인할 수 있다.

    std::ifstream file( // 파일 입력 스트림 객체 file

        filepath, 
        std::ios::in | std::ios::binary // 읽기 모드(생략 가능), 바이너리
    );

    if(!file.is_open()) throw std::runtime_error(
        "Could not open Shader File: " + filepath
    );

    // 파일 전체 내용을 문자로 읽는다.
    std::stringstream stream;

    // 파일 스트림의 내부 스트림 버퍼(스트레치 버퍼 포인터)를 직접 반환
    stream << file.rdbuf(); 

    return stream.str();
}

std::unordered_map<unsigned int, std::string> Shader::PreProcess(const std::string& source){
    std::unordered_map<unsigned int, std::string> shaderSources;

    std::istringstream stream(source); // 한 줄씩 읽는다.
    std::string line;

    GLenum currentType = 0; // 아직은 0
    
    std::stringstream currentSource; // Shader Stage의 GLSL 코드를 모은다

    while(std::getline(stream,line)){
        // Window CRLF 파일에서는 getline이 '\n'만 제거한다.
        // -> '\r'이 남을 수 있다
        if(!line.empty() && line.back() == '\r') line.pop_back();

        // #type으로 시작하는 줄인가
        if(line.rfind("#type ",0) == 0){
            if(currentType != 0){ // 이전 Shader의 Source가 있다면 저장한다.
                // currentType = GL_VERTEX_SHADER
                // currentSource = Vertex GLSL 코드
                shaderSources[currentType] = currentSource.str();

                // 스트림 문자열 버퍼 지우기
                currentSource.str("");
                // 스트림 상태 플래그 초기화
                currentSource.clear();
            }

            // "#type "은 6글자라 6띄우고 시작
            const std::string type = line.substr(6);
            currentType = ShaderTypeFromString(type);
            continue;
        }
        if(currentType == 0) continue; // #type 만나기 전 내용 무시
        currentSource << line << '\n';
    }
    // 마지막 Shader는 다음 #type을 만나지 않고 EOF에 도달한다.
    if(currentType != 0) shaderSources[currentType] = currentSource.str();
    
    if(shaderSources.empty()) throw std::runtime_error("No Shader Source found.");

    return shaderSources;
}

void Shader::Compile(const std::unordered_map<unsigned int, std::string>& shaderSources){
    // 컴파일된 셰이더들을 담고, Link하고, GPU에게 지시하는 역할
    GLuint program = glCreateProgram(); // 여러 Shader Stage를 묶을 Program 객체 생성 


    // Shader Stage
    /*
        기본적이고 필수적인 스테이지는 Vertex Shader와 Fragment Shader
        3D 정점 데이터 
            |
        1. Vertex Shader(정점의 위치 결정 3D -> 2D)
            |  3D공간의 정점을 모니터 화면 좌표계(NDC)에 맞게 계산하고 변환한다.
            |  모든 정점마다 한 번씩 실행된다.
        [Tessellation / Geometry Shader] (선택적 기하 구조 변경 | optional)
            |   테셀레이션은 정점을 더 잘게 쪼개어 디테일하고 부드러운 곡면이나 지형을 만든다.
            |   지오메트리는 입력된 정점들을 조합해, 새로운 도형을 추가로 생성하거나 삭제한다.
        [래스터화(Rasterization)] (하드웨어가 정점을 픽셀 영역으로 쪼갬 | optional) 
            |
        2. Fragment Shader (픽셀의 최종 색상 결정)
    */
    if(program == 0) throw std::runtime_error("Failed to create Shader Program");

    // Link가 끝나면, 각 Shader 객체를 삭제해야한다. 따라서 ID를 저장해둔다.    1 
    // 1. 셰이더 객체 만들고 링크하고 해제해야 한다.
    // 해제하려면 Id 알아야한다. GPU 메모리에서
    std::vector<GLuint> shaderIDs;

    for(const auto& [type, source] : shaderSources){
        GLuint shader = glCreateShader(type); // Vertex | Fragment 셰이더 생성
        if(shader == 0){
            glDeleteProgram(program);
            throw std::runtime_error("Failed to Create Shader object");
        }

        const GLchar* sourceCstr = source.c_str();

        // 셰이더 객체에 소스코드 주입
        glShaderSource(shader,1,&sourceCstr,nullptr);

        // 주입된 코드를 바탕으로 GPU용 컴파일 실행
        glCompileShader(shader);

        GLint isCompiled = GL_FALSE;

        glGetShaderiv(shader,GL_COMPILE_STATUS,&isCompiled);

        if(isCompiled == GL_FALSE){ // 실패 로그
            GLint logLen = 0;
            glGetShaderiv(shader,GL_INFO_LOG_LENGTH,&logLen);

            std::vector<GLchar> infoLog(logLen);
            glGetShaderInfoLog(shader,logLen,nullptr,infoLog.data());

            glDeleteShader(shader);

            // 앞에서 성공했던 셰이더들도 정리
            for(GLuint id: shaderIDs) glDeleteShader(id);

            glDeleteProgram(program);

            throw std::runtime_error(std::string("Shader Compile Failed\n") + infoLog.data());

        }

        // 성공

        glAttachShader(program,shader);

        shaderIDs.push_back(shader);
    }

    // Vertex + Fragment Shader Link 단계
    glLinkProgram(program);

    GLint isLinked = GL_FALSE;
    
    glGetProgramiv(program,GL_LINK_STATUS,&isLinked);

    if (isLinked == GL_FALSE){  // 링크 실패
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

    // Link가 끝났으니 이제 개별 Shader 해제
    for(GLuint id : shaderIDs){
        glDetachShader(program,id);
        glDeleteShader(id);
    }

    m_RendererID = static_cast<uint32_t>(program);
}

// 이후 그리기 요청 때 Shader Program 사용
void Shader::Bind() const {glUseProgram(m_RendererID);} 
void Shader::UnBind() const {glUseProgram(0);} // 해제

//  Uniform Location
/*  Uniform이란 CPU(C++)에서 GPU(Shader)로 데이터를 넘겨줄 때 사용하는 전역 변수다.
    한 번 화면을 그릴 때(Draw Call), 모든 정점과 모든 픽셀에 똑같이 적용되는 상수 값
    셰이더 코드는 GPU 내부에서 수백만 번 실행된다.
    모든 정점과 픽셀이 똑같이 참고해야 하는 데이터가 필요하다.
*/
int Shader::GetUniformLocation(const std::string& name)const{
    // 이미 조회했던 Uniform이라면 Cache에서 바로 반환한다.
    auto it = m_UniformLocationCache.find(name);
    if(it != m_UniformLocationCache.end()) return it->second;

    // 처음 사용하는 Uniform이면 OpenGL에 조회
    int location = glGetUniformLocation(m_RendererID,name.c_str());
    if(location == -1) std::cout << "Warning: Uniform " << name << "'doesn't exist\n";

    m_UniformLocationCache[name] = location;

    return location;
}

// Uniform Setter

void Shader::SetInt(const std::string& name, int value){glUniform1i(GetUniformLocation(name),value);}
void Shader::SetIntArray(const std::string& name, int* values, uint32_t cnt){glUniform1iv(GetUniformLocation(name),static_cast<GLsizei>(cnt),values);}
void Shader::SetFloat(const std::string& name, float value){glUniform1f(GetUniformLocation(name),value);}
void Shader::SetFloat2(const std::string& name, const glm::vec2& value){glUniform2fv(GetUniformLocation(name),1,glm::value_ptr(value));}
void Shader::SetFloat3(const std::string& name, const glm::vec3& value){glUniform3fv(GetUniformLocation(name),1,glm::value_ptr(value));}
void Shader::SetFloat4(const std::string& name, const glm::vec4& value){glUniform4fv(GetUniformLocation(name),1,glm::value_ptr(value));}
void Shader::SetMat3(const std::string& name, const glm::mat3& value){glUniformMatrix3fv(GetUniformLocation(name),1,GL_FALSE,glm::value_ptr(value));}
void Shader::SetMat4(const std::string& name, const glm::mat4& value){glUniformMatrix4fv(GetUniformLocation(name),1,GL_FALSE,glm::value_ptr(value));}

