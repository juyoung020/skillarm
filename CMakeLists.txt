cmake_minimum_required(VERSION 3.24)
project(skillarm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(MSVC)
  add_compile_options(/W4 /permissive- /utf-8 /O2)
endif()

# MuJoCo 4.0.1 prebuilt (third_party/mujoco, unpacked release zip)
set(MUJOCO_DIR ${CMAKE_SOURCE_DIR}/third_party/mujoco)
add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES
  IMPORTED_IMPLIB ${MUJOCO_DIR}/lib/mujoco.lib
  IMPORTED_LOCATION ${MUJOCO_DIR}/bin/mujoco.dll
  INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_DIR}/include)

add_library(skillarm_core
  src/core/calibration.cpp
  src/core/stl.cpp)
target_include_directories(skillarm_core PUBLIC include src)

add_library(skillarm_sim
  src/sim/so101_model.cpp)
target_link_libraries(skillarm_sim PUBLIC skillarm_core mujoco)

add_executable(skillarm tools/skillarm.cpp)
target_link_libraries(skillarm PRIVATE skillarm_sim)
target_compile_definitions(skillarm PRIVATE SKILLARM_ROOT="${CMAKE_SOURCE_DIR}")
add_custom_command(TARGET skillarm POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${MUJOCO_DIR}/bin/mujoco.dll $<TARGET_FILE_DIR:skillarm>)
