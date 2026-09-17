cmake_minimum_required(VERSION 3.20)

project(GraspLink VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8)
endif()

option(GRASPLINK_BUILD_GRAPHICS "Build the OpenGL viewer" ON)

if(GRASPLINK_BUILD_GRAPHICS)
  include(cmake/Dependencies.cmake)
  grasplink_add_graphics_dependencies()
  grasplink_add_graphics_library()

  add_executable(grasplink_simulator
    apps/viewer/main.cpp
    apps/viewer/ViewerApp.cpp
    apps/viewer/ViewerApp.h
    modules/viewer/include/Entity.h
    modules/viewer/include/components/Components.h
    modules/viewer/include/components/GripperComponents.h
    modules/viewer/include/components/RenderComponents.h
    modules/viewer/include/components/RobotComponents.h
    modules/viewer/include/components/TransformComponents.h
    modules/viewer/include/systems/RenderSystemModule.h
    modules/viewer/include/systems/TransformSystemModule.h
    modules/viewer/src/systems/RenderSystemModule.cpp
    modules/viewer/src/systems/TransformSystemModule.cpp
  )
  target_include_directories(grasplink_simulator PRIVATE
    apps/viewer
    modules/common/include
    modules/viewer/include
    modules/viewer/include/components
    modules/viewer/include/graphics
    modules/viewer/include/systems
  )
  target_link_libraries(grasplink_simulator PRIVATE
    grasplink_graphics
    flecs::flecs_static
  )

  add_custom_command(TARGET grasplink_simulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_SOURCE_DIR}/assets/shaders
      $<TARGET_FILE_DIR:grasplink_simulator>/shaders
  )
endif()
