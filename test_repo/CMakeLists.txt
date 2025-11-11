cmake_minimum_required(VERSION 3.10)

project(test_repo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

include_directories("../mujoco-3.1.3/include")
link_directories("../mujoco-3.1.3/lib")
link_directories("../utilities/src/")
include_directories("../utilities/inc/")
include_directories("../robot_ctrl")

# Fetch cxxopts for command-line argument parsing
include(FetchContent)
FetchContent_Declare(
    cxxopts
    GIT_REPOSITORY https://github.com/jarro2783/cxxopts.git
    GIT_TAG v3.1.1
)
FetchContent_MakeAvailable(cxxopts)

if (REAL_ROBOT)
    message(WARNING "\nBuild in Real Robot, Exclude Simulation Library")
    add_executable(run_real run_real.cpp)
    target_link_libraries(run_real hardware_library robot_ctrl_library)
    target_link_libraries(run_real yaml-cpp ${ONNX_LIBRARY})
else()
    add_executable(sim_test start_sim.cpp)
    target_link_libraries(sim_test simulation_library)

    add_executable(sim_ctrl run_sim_ctrl.cpp)
    target_link_libraries(sim_ctrl hardware_library robot_ctrl_library)
    target_link_libraries(sim_ctrl yaml-cpp ${ONNX_LIBRARY})
    target_link_libraries(sim_ctrl cxxopts::cxxopts)
endif ()

#add_executable(test_ros run_robot_ros.cpp)
#target_link_libraries(test_ros simulation_library hardware_library robot_ctrl_library)


