cmake_minimum_required(VERSION 3.10)

project(test_repo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

include_directories("../mujoco-3.1.3/include")
link_directories("../mujoco-3.1.3/lib")
link_directories("../utilities/src/")
include_directories("../utilities/inc/")
include_directories("../onnxruntime-linux-x64-1-16-1/include")
find_library(onnx onnxruntime ../onnxruntime-linux-x64-1-16-1/lib)
if (REAL_ROBOT)
    message(WARNING "\nBuild in Real Robot, Exclude Simulation Library")
    add_executable(test_hardware run_robot.cpp)
    target_link_libraries(test_hardware hardware_library robot_ctrl_library ${onnx})
else()
    add_executable(sim_test start_sim.cpp)
    target_link_libraries(sim_test simulation_library ${onnx})

    add_executable(sim_ctrl run_sim_ctrl.cpp)
    target_link_libraries(sim_ctrl hardware_library robot_ctrl_library ${onnx})

    add_executable(show_robot real_robot_show.cpp)
    target_link_libraries(show_robot simulation_library hardware_library robot_ctrl_library ${onnx})
endif ()

#add_executable(test_ros run_robot_ros.cpp)
#target_link_libraries(test_ros simulation_library hardware_library robot_ctrl_library)


