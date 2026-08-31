include(FetchContent)

function(poselink_enable_viewer app_target)
  add_library(poselink_glad STATIC third_party/glad/src/glad.c)
  target_include_directories(poselink_glad PUBLIC third_party/glad/include)

  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.4 GIT_SHALLOW TRUE)
  FetchContent_Declare(glm GIT_REPOSITORY https://github.com/g-truc/glm.git GIT_TAG 1.0.1 GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(glfw glm)
  find_package(OpenGL REQUIRED)

  add_library(poselink_graphics STATIC
    modules/graphics/src/Renderer.cpp
    modules/graphics/src/Shader.cpp
    modules/graphics/src/Texture.cpp
    modules/graphics/src/VertexArray.cpp
    modules/graphics/src/VertexBuffer.cpp)
  target_include_directories(poselink_graphics PUBLIC modules/graphics/include)
  target_link_libraries(poselink_graphics PUBLIC poselink_glad glm::glm OpenGL::GL)

  target_sources(${app_target} PRIVATE apps/poselink/src/ViewerMode.cpp)
  target_link_libraries(${app_target} PRIVATE poselink_graphics glfw)
  target_compile_definitions(${app_target} PRIVATE POSELINK_WITH_VIEWER=1 GLFW_INCLUDE_NONE)
  add_custom_command(TARGET ${app_target} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_SOURCE_DIR}/modules/graphics/shaders $<TARGET_FILE_DIR:${app_target}>/shaders)
endfunction()
