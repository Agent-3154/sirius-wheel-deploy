cmake_minimum_required(VERSION 3.12)

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_C_COMPILER "/usr/bin/clang-12")
#set(CMAKE_CXX_COMPILER "/usr/bin/clang++-12")

# choose make definition
# *********************************************************************************************************************

if (ROS_BUILD_OPT)
    message(WARNING "\nUse ros path\n")
else ()
    add_compile_definitions(DG_ENGINEER)   # choose the robot type: {CHAOJI_GO  BELT  GO1 DG_ENGINEER}
    message(WARNING "\n Use local path\n")
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
#add_definitions(-DMATRIX_DIM=24)
add_subdirectory(third-party)
add_subdirectory(simulator)
#add_subdirectory(absl_repo)
add_subdirectory(robot)
add_subdirectory(robot_ctrl)
add_subdirectory(test_repo)
add_subdirectory(lcm_convert)

