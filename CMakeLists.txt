cmake_minimum_required(VERSION 3.5)

set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
# ONNX Runtime Configuration - Centralized version management
set(ONNX_VERSION "1.22.0" CACHE STRING "ONNX Runtime version to use")
set(ONNX_DIR "${THIRD_PARTY_DIR}/onnxruntime-linux-x64-${ONNX_VERSION}" CACHE PATH "ONNX Runtime directory")
set(ONNX_INCLUDE_DIR "${ONNX_DIR}/include")
set(ONNX_LIB_DIR "${ONNX_DIR}/lib")

# Validate ONNX directory exists
if(NOT EXISTS ${ONNX_DIR})
    message(FATAL_ERROR "ONNX Runtime directory not found: ${ONNX_DIR}")
endif()

# Find ONNX library
find_library(ONNX_LIBRARY onnxruntime ${ONNX_LIB_DIR})

# Set up include and link directories globally
include_directories(${ONNX_INCLUDE_DIR})
link_directories(${ONNX_LIB_DIR})

message(STATUS "ONNX Runtime configured:")
message(STATUS "  Version: ${ONNX_VERSION}")
message(STATUS "  Directory: ${ONNX_DIR}")
message(STATUS "  Include: ${ONNX_INCLUDE_DIR}")
message(STATUS "  Library: ${ONNX_LIBRARY}")

# Eigen Configuration - Optional local installation with system fallback
option(USE_LOCAL_EIGEN "Use locally installed Eigen 3.4.0 instead of system Eigen" OFF)

if(USE_LOCAL_EIGEN)
    message(STATUS "Using local Eigen 3.4.0")
    # Eigen 3.4.0 Configuration - Local installation
    set(EIGEN3_INSTALL_DIR "${THIRD_PARTY_DIR}/eigen3.4.0_install" CACHE PATH "Eigen 3.4.0 installation directory")
    set(EIGEN3_INCLUDE_DIR "${EIGEN3_INSTALL_DIR}/include/eigen3")
    
    # Add Eigen include directory
    include_directories(${EIGEN3_INCLUDE_DIR})
    
    message(STATUS "Eigen 3.4.0 configured (local installation):")
    message(STATUS "  Version: 3.4.0")
    message(STATUS "  Include Directory: ${EIGEN3_INCLUDE_DIR}")
else()
    # Use system Eigen
    message(STATUS "Using system Eigen")
    find_package(Eigen3 REQUIRED)
    message(STATUS "Eigen3 configured (system installation):")
    message(STATUS "  Version: ${EIGEN3_VERSION_STRING}")
    message(STATUS "  Include Directory: ${EIGEN3_INCLUDE_DIR}")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_C_COMPILER "/usr/bin/clang-12")
#set(CMAKE_CXX_COMPILER "/usr/bin/clang++-12")
set(CMAKE_C_COMPILER "/usr/bin/gcc-11")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-11")

# choose make definition
# *********************************************************************************************************************

if (NOT DEFINED ROS_BUILD)
    # add_compile_definitions(DG_ENGINEER)
    add_compile_definitions(SIRIUS_WHEEL)   # choose the robot type: {CHAOJI_GO  BELT  GO1 DG_ENGINEER}
    add_compile_definitions(ICEORYX_PREFIX_V2954) # {V , V2.95.4}
    message(WARNING "\nsub robot define included! Use local path")
else ()
    message(WARNING "\nSub robot define not included! Use ros path")
endif ()

if (REAL_ROBOT)
    message(WARNING "\nREAL_ROBOT define included!")
else ()
    set(SIMULATOR_FLG true)
    add_compile_definitions(SIMULATOR)
    message(WARNING "\nSimulator define included!")
endif ()

if(GAME_STAR)
    message(WARNING "Remote Controller: Game Star\n")
    add_compile_definitions(GAME_STAR)
else ()
    message(WARNING "Remote Controller: LOGIC Gamepad\n")
endif ()
# *********************************************************************************************************************


SET(CMAKE_BUILD_TYPE "Release")
project(My_Mujoco_Simulator)
set(CMAKE_CXX_FLAGS "-O3 -fPIC -no-pie -march=native -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Wno-overloaded-virtual \
  -Wno-missing-field-initializers -Wno-unused-parameter")

set(CMAKE_C_FLAGS "-O3  -ggdb -std=gnu99 -march=native -I.")
set(BUILD_TYPE_RELEASE TRUE)
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)
add_definitions(-DEIGEN_FAST_MATH=1)
if(SIMULATOR_FLG)
    add_subdirectory(simulator)
    message(WARNING "\n Build the simulator file")
endif ()
add_subdirectory(robot)
add_subdirectory(robot_ctrl)
add_subdirectory(test_repo)
add_subdirectory(lcm_convert)

