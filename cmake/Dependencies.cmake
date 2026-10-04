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

  set(TINYGLTF_BUILD_LOADER_EXAMPLE OFF CACHE BOOL "" FORCE)
  set(TINYGLTF_BUILD_GL_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(TINYGLTF_BUILD_VALIDATOR_EXAMPLE OFF CACHE BOOL "" FORCE)
  set(TINYGLTF_BUILD_BUILDER_EXAMPLE OFF CACHE BOOL "" FORCE)
  set(TINYGLTF_INSTALL OFF CACHE BOOL "" FORCE)

  FetchContent_Declare(
    tinygltf
    GIT_REPOSITORY https://github.com/syoyo/tinygltf.git
    GIT_TAG v2.9.7
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(glfw flecs tinygltf)
  find_package(OpenGL REQUIRED)

  set(GRASPLINK_GRAPHICS_SOURCES
    modules/viewer/src/graphics/Camera.cpp
    modules/viewer/src/graphics/OrbitCameraController.cpp
    modules/viewer/src/graphics/IndexBuffer.cpp
    modules/viewer/src/graphics/Material.cpp
    modules/viewer/src/graphics/Mesh.cpp
    modules/viewer/src/graphics/Renderer.cpp
    modules/viewer/src/graphics/Shader.cpp
    modules/viewer/src/graphics/Texture.cpp
    modules/viewer/src/graphics/VertexArray.cpp
    modules/viewer/src/graphics/VertexBuffer.cpp
    modules/viewer/src/graphics/Window.cpp
    modules/viewer/src/graphics/ShadowMap.cpp
    modules/viewer/src/graphics/MultisampleFramebuffer.cpp
  )
  set(GRASPLINK_GRAPHICS_HEADERS
    modules/viewer/include/components/TransformComponents.h
    modules/viewer/include/components/RenderComponents.h
    modules/viewer/include/graphics/Camera.h
    modules/viewer/include/graphics/OrbitCameraController.h
    modules/viewer/include/graphics/IndexBuffer.h
    modules/viewer/include/graphics/Material.h
    modules/viewer/include/graphics/Mesh.h
    modules/viewer/include/graphics/Renderer.h
    modules/viewer/include/graphics/Shader.h
    modules/viewer/include/graphics/Texture.h
    modules/viewer/include/graphics/VertexArray.h
    modules/viewer/include/graphics/VertexBuffer.h
    modules/viewer/include/graphics/Window.h
    modules/viewer/include/graphics/ShadowMap.h
    modules/viewer/include/graphics/MultisampleFramebuffer.h
  )

  add_library(grasplink_graphics STATIC
    ${GRASPLINK_GRAPHICS_SOURCES}
    ${GRASPLINK_GRAPHICS_HEADERS}
  )
  target_include_directories(grasplink_graphics PUBLIC
    modules/common/include
    modules/viewer/include
    modules/viewer/include/components
    modules/viewer/include/graphics
    modules/viewer/include/scene
    modules/viewer/include/systems
  )
  target_link_libraries(grasplink_graphics PUBLIC
    grasplink_glad
    glfw
    glm::glm
    OpenGL::GL
  )
  target_compile_definitions(grasplink_graphics PUBLIC GLFW_INCLUDE_NONE)
endfunction()


function(grasplink_add_physics_dependencies)

  # Jolt의 기본값은 MSVC에서 Static Runtime(/MT, /MTd)을 사용한다.
  #
  # GraspLink는 기본 MSVC Dynamic Runtime(/MD, /MDd)을 사용하므로
  # Jolt도 같은 Runtime Library를 사용하도록 Static Runtime을 끈다.
  #
  # Debug:
  #   GraspLink -> /MDd
  #   Jolt      -> /MDd
  #
  # Release:
  #   GraspLink -> /MD
  #   Jolt      -> /MD
  #
  # FORCE를 사용하는 이유:
  # 이전 CMake configure에서 ON 값이 Cache에 남아 있을 수 있기 때문이다.
  set(
    USE_STATIC_MSVC_RUNTIME_LIBRARY
    OFF
    CACHE BOOL
    "Use the static MSVC runtime library"
    FORCE
  )

  FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG v5.6.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR Build
  )

  FetchContent_MakeAvailable(JoltPhysics)

endfunction()
