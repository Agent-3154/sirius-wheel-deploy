cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_C_COMPILER "/usr/bin/clang-12")
#set(CMAKE_CXX_COMPILER "/usr/bin/clang++-12")

# choose make definition
# *********************************************************************************************************************

if (NOT DEFINED ROS_BUILD)
    add_compile_definitions(DG_ENGINEER)   # choose the robot type: {CHAOJI_GO  BELT  GO1 DG_ENGINEER}
    add_compile_definitions(WORK_COMPUTOR)
    message(WARNING "\n\nsub robot define included! Use local path\n\n")
else ()
    message(WARNING "\n\nSub robot define not included! Use ros path\n\n")
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
add_subdirectory(simulator)
add_subdirectory(robot)
add_subdirectory(robot_ctrl)
add_subdirectory(test_repo)
add_subdirectory(lcm_convert)

