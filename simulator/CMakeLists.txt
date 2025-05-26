cmake_minimum_required(VERSION 3.10)

project(my_simulator)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -Wno-unused-parameter -Wno-unused-variable -Wno-sign-compare")

include_directories("../mujoco-3.1.3/include")
link_directories("../mujoco-3.1.3/lib")
include_directories(include)
include_directories("../robot/robot_model")
link_directories("../utilities/src/")
include_directories("../utilities/inc/")

file(GLOB simulator_sources "source/*.cc" "source/*.cpp" "../utilities/src/*.cpp"
        "../robot/robot_model/*.cpp" "motor_control/*.cpp")

include(GNUInstallDirs)
include_directories("/usr/local/include")
find_package(iceoryx_platform REQUIRED)
find_package(iceoryx_posh CONFIG REQUIRED)
find_package(iceoryx_hoofs CONFIG REQUIRED)

add_library(simulation_library SHARED ${simulator_sources}
        my_simulator.cpp
        SimulationBridge.h
        SimulationBridge.cpp
        motor_control/my_motor_model.h
        motor_control/my_motor_model.cpp
        ../utilities/src/easylogging++.cc
)

target_link_libraries(simulation_library
        libmujoco.so glfw iceoryx_posh::iceoryx_posh
        lcm)


