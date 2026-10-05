#include "MultisampleFramebuffer.h"
#include "Shader.h"
#include "ShadowMap.h"
#include "Window.h"
#include "TestSupport.h"

#include <glad/glad.h>

#include <array>
#include <iostream>
#include <string>
#include <vector>

namespace
{
struct GlHooks;
GlHooks* activeHooks = nullptr;

/**
 * @brief OpenGL 객체 생성/실패 지점을 감시하는 테스트용 GLAD 함수 집합.
 * @details 실제 OpenGL 동작은 저장된 GLAD 함수를 호출하고 생성 ID를 기록한다. 일부 검사는 반환 상태나
 * shader 입력만 제어해 생성자 실패 경로와 부분 생성 자원의 정리를 검사한다.
 */
struct GlHooks
{
    PFNGLGENFRAMEBUFFERSPROC genFramebuffers = glad_glGenFramebuffers;
    PFNGLGENTEXTURESPROC genTextures = glad_glGenTextures;
    PFNGLGENRENDERBUFFERSPROC genRenderbuffers = glad_glGenRenderbuffers;
    PFNGLCHECKFRAMEBUFFERSTATUSPROC checkFramebufferStatus = glad_glCheckFramebufferStatus;
    PFNGLGETINTEGERVPROC getIntegerv = glad_glGetIntegerv;
    PFNGLCREATEPROGRAMPROC createProgram = glad_glCreateProgram;
    PFNGLCREATESHADERPROC createShader = glad_glCreateShader;
    PFNGLCOMPILESHADERPROC compileShader = glad_glCompileShader;
    std::vector<GLuint> framebuffers;
    std::vector<GLuint> textures;
    std::vector<GLuint> renderbuffers;
    std::vector<GLuint> programs;
    std::vector<GLuint> shaders;
    bool incompleteFramebuffer = false;
    bool unsupportedSamples = false;
    bool failCreateProgram = false;
    int failCreateShaderCall = 0;
    int failCompileShaderCall = 0;
    int createShaderCalls = 0;
    int compileShaderCalls = 0;

    GlHooks();
    ~GlHooks();
    GlHooks(const GlHooks&) = delete;
    GlHooks& operator=(const GlHooks&) = delete;

    void RequireReleased(const std::string& label) const
    {
        for (GLuint id : framebuffers) Require(glIsFramebuffer(id) == GL_FALSE, label + ": framebuffer released");
        for (GLuint id : textures) Require(glIsTexture(id) == GL_FALSE, label + ": texture released");
        for (GLuint id : renderbuffers) Require(glIsRenderbuffer(id) == GL_FALSE, label + ": renderbuffer released");
        for (GLuint id : shaders) Require(glIsShader(id) == GL_FALSE, label + ": shader stage released");
        for (GLuint id : programs) Require(glIsProgram(id) == GL_FALSE, label + ": program released");
    }

    void Reset()
    {
        RequireReleased("before next case");
        framebuffers.clear();
        textures.clear();
        renderbuffers.clear();
        programs.clear();
        shaders.clear();
        incompleteFramebuffer = false;
        unsupportedSamples = false;
        failCreateProgram = false;
        failCreateShaderCall = 0;
        failCompileShaderCall = 0;
        createShaderCalls = 0;
        compileShaderCalls = 0;
    }
};

void APIENTRY RecordFramebuffers(GLsizei count, GLuint* ids)
{
    activeHooks->genFramebuffers(count, ids);
    activeHooks->framebuffers.insert(activeHooks->framebuffers.end(), ids, ids + count);
}

void APIENTRY RecordTextures(GLsizei count, GLuint* ids)
{
    activeHooks->genTextures(count, ids);
    activeHooks->textures.insert(activeHooks->textures.end(), ids, ids + count);
}

void APIENTRY RecordRenderbuffers(GLsizei count, GLuint* ids)
{
    activeHooks->genRenderbuffers(count, ids);
    activeHooks->renderbuffers.insert(activeHooks->renderbuffers.end(), ids, ids + count);
}

GLenum APIENTRY CheckFramebuffer(GLenum target)
{
    return activeHooks->incompleteFramebuffer ? GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT
        : activeHooks->checkFramebufferStatus(target);
}

void APIENTRY GetInteger(GLenum name, GLint* value)
{
    if (name == GL_MAX_SAMPLES && activeHooks->unsupportedSamples) *value = 1;
    else activeHooks->getIntegerv(name, value);
}

GLuint APIENTRY RecordProgram()
{
    if (activeHooks->failCreateProgram) return 0;
    const GLuint id = activeHooks->createProgram();
    if (id != 0) activeHooks->programs.push_back(id);
    return id;
}

GLuint APIENTRY RecordShader(GLenum type)
{
    if (++activeHooks->createShaderCalls == activeHooks->failCreateShaderCall) return 0;
    const GLuint id = activeHooks->createShader(type);
    if (id != 0) activeHooks->shaders.push_back(id);
    return id;
}

void APIENTRY CompileShader(GLuint shader)
{
    if (++activeHooks->compileShaderCalls == activeHooks->failCompileShaderCall)
    {
        // 실패: stage 순서에 의존하지 않고 두 번째 stage를 실제 GLSL 컴파일 오류로 만듦.
        const GLchar* invalidSource = "#version 330 core\nvoid main(){ invalid GLSL; }\n";
        glShaderSource(shader, 1, &invalidSource, nullptr);
    }
    activeHooks->compileShader(shader);
}

GlHooks::GlHooks()
{
    // 수명: 테스트 범위가 끝나거나 예외가 나면 원래 GLAD 함수를 복원.
    activeHooks = this;
    glad_glGenFramebuffers = RecordFramebuffers;
    glad_glGenTextures = RecordTextures;
    glad_glGenRenderbuffers = RecordRenderbuffers;
    glad_glCheckFramebufferStatus = CheckFramebuffer;
    glad_glGetIntegerv = GetInteger;
    glad_glCreateProgram = RecordProgram;
    glad_glCreateShader = RecordShader;
    glad_glCompileShader = CompileShader;
}

GlHooks::~GlHooks()
{
    glad_glGenFramebuffers = genFramebuffers;
    glad_glGenTextures = genTextures;
    glad_glGenRenderbuffers = genRenderbuffers;
    glad_glCheckFramebufferStatus = checkFramebufferStatus;
    glad_glGetIntegerv = getIntegerv;
    glad_glCreateProgram = createProgram;
    glad_glCreateShader = createShader;
    glad_glCompileShader = compileShader;
    activeHooks = nullptr;
}

template<typename Function>
void RequireFailure(Function&& function, const std::string& expectedMessage)
{
    try { function(); }
    catch (const std::runtime_error& error)
    {
        Require(std::string(error.what()).find(expectedMessage) != std::string::npos,
            "unexpected failure: " + std::string(error.what()));
        return;
    }
    throw std::runtime_error("missing failure: " + expectedMessage);
}

void TestShadowMap(GlHooks& hooks)
{
    // 잘못된 크기와 불완전 framebuffer의 정리, Begin/End의 framebuffer·viewport 복원 계약을 확인한다.
    for (int size : {0, -1})
        ExpectThrows<std::invalid_argument>([&] { ShadowMap invalid(size); }, "invalid shadow size rejected");
    Require(hooks.framebuffers.empty() && hooks.textures.empty(), "invalid shadow size allocates nothing");

    const std::array<GLint, 4> previousViewport{2, 3, 70, 80};
    glViewport(previousViewport[0], previousViewport[1], previousViewport[2], previousViewport[3]);
    {
        ShadowMap shadow(32);
        Require(hooks.framebuffers.size() == 1 && hooks.textures.size() == 1, "shadow resource count");
        shadow.Begin();
        GLint bound = 0;
        glGetIntegerv(GL_FRAMEBUFFER_BINDING, &bound);
        Require(bound == static_cast<GLint>(hooks.framebuffers.front()), "shadow framebuffer bound");
        Require(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE, "shadow framebuffer complete");
        std::array<GLint, 4> viewport{};
        glGetIntegerv(GL_VIEWPORT, viewport.data());
        Require(viewport == std::array<GLint, 4>{0, 0, 32, 32}, "shadow viewport matches storage");
        shadow.End();
        glGetIntegerv(GL_FRAMEBUFFER_BINDING, &bound);
        Require(bound == 0, "shadow End restores default framebuffer");
        glGetIntegerv(GL_VIEWPORT, viewport.data());
        Require(viewport == previousViewport, "shadow End restores viewport");
        shadow.Bind(0);
        GLint width = 0;
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
        Require(width == 32, "shadow depth texture storage");
    }
    hooks.RequireReleased("shadow destructor");
    hooks.Reset();
    hooks.incompleteFramebuffer = true;
    RequireFailure([&] { ShadowMap failed(32); }, "Shadow framebuffer is incomplete");
    Require(hooks.framebuffers.size() == 1 && hooks.textures.size() == 1, "failed shadow allocated both objects");
    hooks.RequireReleased("failed shadow constructor");
    hooks.Reset();
}

void TestMultisampleFramebuffer(GlHooks& hooks)
{
    // GPU 표본 한도 clamp, Resize의 보존/재생성, 실패한 생성·resize의 모든 attachment 정리를 검사한다.
    GLint maxSamples = 0;
    glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);
    Require(maxSamples >= 2, "test context supports MSAA");
    ExpectThrows<std::invalid_argument>([] { MultisampleFramebuffer invalid(0, 32); }, "zero MSAA width rejected");
    ExpectThrows<std::invalid_argument>([] { MultisampleFramebuffer invalid(32, -1); }, "negative MSAA height rejected");
    for (int samples : {-1, 0, 1})
        RequireFailure([&] { MultisampleFramebuffer invalid(32, 32, samples); }, "multisampling is not supported");
    hooks.unsupportedSamples = true;
    RequireFailure([] { MultisampleFramebuffer unsupported(32, 32, 4); }, "multisampling is not supported");
    Require(hooks.framebuffers.empty() && hooks.textures.empty() && hooks.renderbuffers.empty(),
        "invalid or unsupported MSAA allocates nothing");
    hooks.Reset();
    {
        MultisampleFramebuffer framebuffer(32, 48, maxSamples + 1);
        Require(framebuffer.GetSamples() == maxSamples, "MSAA samples clamp to GPU limit");
        framebuffer.Bind();
        Require(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE, "MSAA framebuffer complete");
        framebuffer.Resize(0, 72);
        Require(framebuffer.GetWidth() == 32 && framebuffer.GetHeight() == 48, "invalid resize keeps dimensions");
        framebuffer.Resize(32, 48);
        Require(hooks.framebuffers.size() == 1, "unchanged resize keeps storage");
        framebuffer.Resize(64, 72);
        Require(framebuffer.GetWidth() == 64 && framebuffer.GetHeight() == 72, "MSAA resized dimensions");
        framebuffer.Bind();
        Require(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE, "resized MSAA framebuffer complete");
        std::array<GLint, 4> viewport{};
        glGetIntegerv(GL_VIEWPORT, viewport.data());
        Require(viewport == std::array<GLint, 4>{0, 0, 64, 72}, "resized MSAA viewport");
        GLint width = 0;
        glGetTexLevelParameteriv(GL_TEXTURE_2D_MULTISAMPLE, 0, GL_TEXTURE_WIDTH, &width);
        Require(width == 64, "resized MSAA color storage");
        framebuffer.ResolveToDefault();
    }
    hooks.RequireReleased("MSAA destructor after resize");
    hooks.Reset();
    hooks.incompleteFramebuffer = true;
    RequireFailure([] { MultisampleFramebuffer failed(32, 32); }, "MSAA framebuffer is incomplete");
    Require(hooks.framebuffers.size() == 1 && hooks.textures.size() == 1 && hooks.renderbuffers.size() == 1,
        "failed MSAA constructor allocated all objects");
    hooks.RequireReleased("failed MSAA constructor");
    hooks.Reset();
    {
        MultisampleFramebuffer framebuffer(32, 32);
        hooks.incompleteFramebuffer = true;
        RequireFailure([&] { framebuffer.Resize(64, 80); }, "MSAA framebuffer is incomplete");
        Require(hooks.framebuffers.size() == 2 && hooks.textures.size() == 2 && hooks.renderbuffers.size() == 2,
            "failed resize attempted new storage");
        hooks.RequireReleased("failed resize releases original and new storage");
        hooks.Reset();
        framebuffer.Resize(64, 80);
        Require(hooks.framebuffers.size() == 1 && hooks.textures.size() == 1 && hooks.renderbuffers.size() == 1,
            "same-size retry recreates all MSAA storage");
        framebuffer.Bind();
        Require(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE,
            "MSAA recovers at same size after failed resize");
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    hooks.RequireReleased("recovered MSAA destructor");
    hooks.Reset();
}

void TestShader(GlHooks& hooks)
{
    // Program/stage 생성 실패와 실제 compile/link 실패에서 임시 GL object가 남지 않는지 확인한다.
    const std::string vertex = "#version 330 core\nout vec3 value; void main(){value=vec3(1);gl_Position=vec4(0,0,0,1);}";
    const std::string fragment = "#version 330 core\nin vec3 value; out vec4 color; void main(){color=vec4(value,1);}";
    {
        auto shader = Shader::CreateFromSource("resource-test", vertex, fragment);
        Require(shader->GetName() == "resource-test", "shader source name");
        Require(hooks.programs.size() == 1 && hooks.shaders.size() == 2, "shader success resource count");
        Require(glIsProgram(hooks.programs.front()) == GL_TRUE, "linked program retained");
        for (GLuint id : hooks.shaders) Require(glIsShader(id) == GL_FALSE, "linked stage detached and deleted");
        shader->Bind();
        GLint current = 0;
        glGetIntegerv(GL_CURRENT_PROGRAM, &current);
        Require(current == static_cast<GLint>(hooks.programs.front()), "shader binds linked program");
        shader->UnBind();
    }
    hooks.RequireReleased("shader destructor");
    hooks.Reset();
    hooks.failCompileShaderCall = 2;
    RequireFailure([&] { Shader failed("compile-failure", vertex, fragment); }, "Shader Compile Failed");
    Require(hooks.shaders.size() == 2 && hooks.compileShaderCalls == 2, "second-stage real compile failure exercised");
    // 확인: 앞 stage는 Program에 연결된 상태라 Program 삭제 후 실제 해제까지 검사.
    hooks.RequireReleased("second-stage compile failure");
    hooks.Reset();
    const std::string mismatch = "#version 330 core\nin vec2 value; out vec4 color; void main(){color=vec4(value,0,1);}";
    RequireFailure([&] { Shader failed("link-failure", vertex, mismatch); }, "Shader Link Failed");
    Require(hooks.shaders.size() == 2, "real shader link failure exercised");
    hooks.RequireReleased("link failure");
    hooks.Reset();
    for (int failedCall : {1, 2})
    {
        hooks.failCreateShaderCall = failedCall;
        RequireFailure([&] { Shader failed("stage-create-failure", vertex, fragment); }, "Failed to Create Shader object");
        Require(hooks.createShaderCalls == failedCall && hooks.shaders.size() == static_cast<std::size_t>(failedCall - 1),
            "selected shader creation failure exercised");
        hooks.RequireReleased("stage creation failure");
        hooks.Reset();
    }
    hooks.failCreateProgram = true;
    RequireFailure([&] { Shader failed("program-create-failure", vertex, fragment); }, "Failed to create Shader Program");
    Require(hooks.programs.empty() && hooks.shaders.empty(), "program creation failure allocates no stages");
    hooks.Reset();
}
}

/**
 * @brief ShadowMap, MSAA framebuffer, Shader의 OpenGL 자원 수명과 실패 복구를 확인한다.
 * @details 숨긴 Window가 GL context를 소유한다. 테스트 객체가 만드는 GL 자원과 GlHooks가 바꾼 GLAD 함수 포인터를
 * 먼저 정리하고 Window가 마지막에 파괴되어야 한다. 불완전 상태는 hook으로 재현하고 객체 삭제 여부를 GL에서 조회한다.
 */
int main()
{
    try
    {
        // 수명: 함수 지역 GPU 객체와 GlHooks가 먼저 정리되고, Window가 GL context를 마지막에 닫는다.
        Window window(Window::Properties{128, 128, "GraphicsResourceTests", false, false});
        GlHooks hooks;
        TestShadowMap(hooks);
        TestMultisampleFramebuffer(hooks);
        TestShader(hooks);
        Require(glGetError() == GL_NO_ERROR, "resource checks leave no OpenGL error");
        std::cout << "Graphics resource lifetime and failure cleanup checks passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
