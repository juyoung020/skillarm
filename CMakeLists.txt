cmake_minimum_required(VERSION 3.24)
project(skillarm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(MSVC)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/W4> $<$<COMPILE_LANGUAGE:CXX>:/permissive-> $<$<COMPILE_LANGUAGE:CXX>:/utf-8> $<$<COMPILE_LANGUAGE:CXX>:/O2>)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
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
  src/sim/so101_model.cpp
  src/sim/path_check.cpp
  src/sim/sim_camera.cpp)
target_link_libraries(skillarm_sim PUBLIC skillarm_core mujoco opengl32)
target_compile_definitions(skillarm_sim PRIVATE SKILLARM_ROOT="${CMAKE_SOURCE_DIR}")

add_library(skillarm_kin src/kin/kinematics.cpp)
target_link_libraries(skillarm_kin PUBLIC skillarm_core mujoco)

add_library(skillarm_harness
  src/env/sim_pick_cube.cpp
  src/record/recorder.cpp
  src/api/api_impl.cpp
  src/runner/runner.cpp)
target_include_directories(skillarm_harness PUBLIC third_party)
target_link_libraries(skillarm_harness PUBLIC skillarm_sim skillarm_kin skillarm_real)
target_compile_definitions(skillarm_harness PRIVATE SKILLARM_ROOT="${CMAKE_SOURCE_DIR}")

add_library(skillarm_real src/real/arm_client.cpp src/real/real_so101.cpp)
target_include_directories(skillarm_real PUBLIC include src third_party)
target_link_libraries(skillarm_real PUBLIC skillarm_core mujoco ws2_32)

add_library(skillarm_perceive src/perceive/clip_tokenizer.cpp src/perceive/depth_shift.cpp src/perceive/onnx_bake.cpp)
target_include_directories(skillarm_perceive PUBLIC include src third_party)

# GPU perception (SAM3 + MoGe-2 on TensorRT, CUDA pre/post-processing). Built when CUDA and TensorRT are found.
set(TENSORRT_DIR "C:/tools/TensorRT-11.5.0.63" CACHE PATH "TensorRT zip root (include/, lib/, bin/)")
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER AND EXISTS "${TENSORRT_DIR}/include/NvInfer.h")
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  set(CMAKE_CUDA_ARCHITECTURES 120)  # RTX 50xx (Blackwell)
  add_library(skillarm_gpu
    src/perceive/kernels.cu
    src/perceive/trt_engine.cpp
    src/perceive/sam3_segmenter.cpp
    src/perceive/moge_depth.cpp)
  target_compile_options(skillarm_gpu PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-O3 --use_fast_math -Xcompiler=/utf-8>)
  target_include_directories(skillarm_gpu PUBLIC include src third_party "${TENSORRT_DIR}/include")
  target_link_libraries(skillarm_gpu PUBLIC skillarm_perceive CUDA::cudart_static
    "${TENSORRT_DIR}/lib/nvinfer_11.lib" "${TENSORRT_DIR}/lib/nvonnxparser_11.lib")
  set(SKILLARM_GPU ON)
  message(STATUS "skillarm: GPU perception enabled (TensorRT ${TENSORRT_DIR})")
else()
  message(STATUS "skillarm: GPU perception disabled (CUDA or TensorRT not found)")
endif()

add_executable(skillarm tools/skillarm.cpp tools/perceive_cmds.cpp)
target_link_libraries(skillarm PRIVATE skillarm_sim skillarm_kin skillarm_harness skillarm_real skillarm_perceive)
target_compile_definitions(skillarm PRIVATE SKILLARM_ROOT="${CMAKE_SOURCE_DIR}")
add_custom_command(TARGET skillarm POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${MUJOCO_DIR}/bin/mujoco.dll $<TARGET_FILE_DIR:skillarm>)
if(SKILLARM_GPU)
  # TensorRT DLLs load on first GPU use (from TENSORRT_DIR/bin), so CPU-only commands run without them
  target_link_libraries(skillarm PRIVATE skillarm_gpu delayimp)
  target_link_options(skillarm PRIVATE /DELAYLOAD:nvinfer_11.dll /DELAYLOAD:nvonnxparser_11.dll)
  target_compile_definitions(skillarm PRIVATE SKILLARM_GPU=1 SKILLARM_TENSORRT_BIN="${TENSORRT_DIR}/bin")
endif()
