include(FetchContent)

function(grasplink_add_graphics_dependencies)
  FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(glm)
endfunction()

function(grasplink_add_graphics_library)
  add_library(grasplink_glad STATIC third_party/glad/src/glad.c)
  target_include_directories(grasplink_glad PUBLIC third_party/glad/include)

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
    flecs
    GIT_REPOSITORY https://github.com/SanderMertens/flecs.git
    GIT_TAG v4.1.5
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(glfw flecs)
  find_package(OpenGL REQUIRED)

  add_library(grasplink_graphics STATIC
    modules/viewer/include/components/TransformComponents.h
    modules/viewer/include/components/RenderComponents.h
    modules/viewer/include/graphics/Camera.h
    modules/viewer/include/graphics/IndexBuffer.h
    modules/viewer/include/graphics/Material.h
    modules/viewer/include/graphics/Mesh.h
    modules/viewer/include/graphics/Renderer.h
    modules/viewer/include/graphics/Shader.h
    modules/viewer/include/graphics/VertexArray.h
    modules/viewer/include/graphics/VertexBuffer.h
    modules/viewer/include/graphics/Window.h
    modules/viewer/src/graphics/Camera.cpp
    modules/viewer/src/graphics/IndexBuffer.cpp
    modules/viewer/src/graphics/Material.cpp
    modules/viewer/src/graphics/Mesh.cpp
    modules/viewer/src/graphics/Renderer.cpp
    modules/viewer/src/graphics/Shader.cpp
    modules/viewer/src/graphics/VertexArray.cpp
    modules/viewer/src/graphics/VertexBuffer.cpp
    modules/viewer/src/graphics/Window.cpp
  )
  target_include_directories(grasplink_graphics PUBLIC
    modules/common/include
    modules/viewer/include
    modules/viewer/include/components
    modules/viewer/include/graphics
  )
  target_link_libraries(grasplink_graphics PUBLIC
    grasplink_glad
    glfw
    glm::glm
    OpenGL::GL
  )
  target_compile_definitions(grasplink_graphics PUBLIC GLFW_INCLUDE_NONE)
endfunction()
