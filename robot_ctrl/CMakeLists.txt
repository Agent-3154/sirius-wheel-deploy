set(INC_DIR /usr/local/include)
set(LINK_DIR /usr/local/lib)

set(CMAKE_CXX_STANDARD 17)
include_directories(${INC_DIR})

include_directories("../utilities/types")
include_directories("../utilities/inc")
include_directories("FSM/")
link_directories(${LINK_DIR})
file(GLOB_RECURSE robot_ctrl_sources "*.cpp" "FSM/*.cpp")
include_directories("../onnxruntime-linux-x64-1-16-1/include")
find_package(iceoryx_platform REQUIRED)
find_package(iceoryx_posh CONFIG REQUIRED)
find_package(iceoryx_hoofs CONFIG REQUIRED)

add_library(robot_ctrl_library SHARED ${robot_ctrl_sources}
        ../utilities/src/easylogging++.cc
)
target_link_libraries(robot_ctrl_library
        lcm iceoryx_posh::iceoryx_posh ${onnx})