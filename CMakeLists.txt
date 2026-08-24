cmake_minimum_required(VERSION 3.20)

#glad.c가 C파일이기 때문에 C, CXX 둘 다 활성화
project(MiniBCG
    VERSION 0.1.0
    LANGUAGES C CXX
)

#C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# -----------------
# GLAD
# -----------------

# GLAD는 OpenGL 함수의 주소를 런타임에 가져오기 위한 Loader

# Window에서 OpenGL 함수들을 바로 호출할 수 없다.
# glCreateShader와 같은 OpenGL 3.x 함수 주소를 실행 시점에 가져와야 한다.

# external/glad/src/glad.c를 별도의 정적 라이브러리로 만든다.
add_library(glad STATIC
    external/glad/src/glad.c
)

# glad/glad.h를 다른 타깃에서도 include 가능하도록 경로 공개
target_include_directories(glad
    PUBLIC
        external/glad/include
)

# GLFW / OpenGL

# GLFW
# - Window 생성
# - OpenGL Context 생성
# - 키보드/마우스 이벤트 등을 담당한다

include(FetchContent)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(glfw glm)
find_package(OpenGL REQUIRED)

# MiniBCG

add_executable(MiniBCG
    src/main.cpp

    src/core/EngineApp.cpp
    src/core/Window.cpp
    src/graphics/Shader.cpp
    src/graphics/VertexBuffer.cpp
    src/graphics/VertexArray.cpp
)

# Project Root

target_include_directories(MiniBCG
    PRIVATE
        include
)

target_compile_definitions(MiniBCG PRIVATE GLFW_INCLUDE_NONE)

target_link_libraries(MiniBCG
    PRIVATE
        glad
        glfw
        glm::glm
        OpenGL::GL
        ${CMAKE_DL_LIBS}
)


