include(FetchContent)

function(poselink_add_graphics_library)
  add_library(poselink_glad STATIC third_party/glad/src/glad.c)
  target_include_directories(poselink_glad PUBLIC third_party/glad/include)

  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.4 GIT_SHALLOW TRUE)
  FetchContent_Declare(glm GIT_REPOSITORY https://github.com/g-truc/glm.git GIT_TAG 1.0.1 GIT_SHALLOW TRUE)
  # Scene/Object 관리에만 사용할 ECS 라이브러리.
  # OpenGL, Mesh, Shader, Texture 구현 자체는 기존 모듈이 계속 담당한다.
  FetchContent_Declare(flecs GIT_REPOSITORY https://github.com/SanderMertens/flecs.git GIT_TAG v4.1.5 GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(glfw glm flecs)
  find_package(OpenGL REQUIRED)

  add_library(poselink_graphics STATIC
    modules/common/include/Pose.h
    modules/common/include/Transform.h
    modules/viewer/include/Renderable.h
    modules/viewer/src/Window.cpp
    modules/viewer/src/Renderer.cpp
    modules/viewer/src/Shader.cpp
    modules/viewer/src/Mesh.cpp
    modules/viewer/src/Camera.cpp
    modules/viewer/src/Texture.cpp
    modules/viewer/src/IndexBuffer.cpp
    modules/viewer/src/VertexArray.cpp
    modules/viewer/src/VertexBuffer.cpp)
  target_include_directories(poselink_graphics PUBLIC
    modules/common/include
    modules/viewer/include)
  target_link_libraries(poselink_graphics PUBLIC poselink_glad glfw glm::glm flecs::flecs_static OpenGL::GL)
  target_compile_definitions(poselink_graphics PUBLIC GLFW_INCLUDE_NONE)

endfunction()
