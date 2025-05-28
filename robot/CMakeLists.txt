set(INC_DIR /usr/local/include)
set(LINK_DIR /usr/local/lib)

set(CMAKE_CXX_STANDARD 17)
include_directories("estimators/")
include_directories("leg_controller/")
include_directories("robot_model/")
include_directories("robot_runner/")
include_directories("thread_tasks/")
include_directories("thread_tasks/inc")
include_directories("hardwares/usb/include")
include_directories("../robot_ctrl")
include_directories(${INC_DIR})
include_directories("../utilities/types")
include_directories("../robot_ctrl/")

link_directories(${LINK_DIR})

file(GLOB_RECURSE hardware_sources "hardwares/usb/src/*.cpp"  "../utilities/src/*.cpp" "*.cpp"
        "estimators/*.cpp" "leg_controller/*.cpp" "thread_tasks/src/*.cpp" "thread_tasks/*.cpp" "robot_model/*.cpp"
        "hardwares/usb/src/*.cpp" "robot_runner/*.cpp")

include(GNUInstallDirs)
#include_directories("/home/hb/legged/iceoryx/")
find_package(iceoryx_platform REQUIRED)
find_package(iceoryx_posh CONFIG REQUIRED)
find_package(iceoryx_hoofs CONFIG REQUIRED)

add_library(hardware_library SHARED ${hardware_sources}
        thread_tasks/inc/thread_usb_hardware.h
        hardwares/usb/include/rt_usb_base.h
        hardwares/usb/src/rt_usb_base.cpp
        thread_tasks/src/thread_usb_hardware.cpp
        ../config/Config.h
        estimators/sim_estimator.h
        estimators/sim_estimator.cpp
        #        ../utilities/src/easylogging++.cc
)

target_link_libraries(hardware_library
        lcm libusb-1.0.so iceoryx_posh::iceoryx_posh)
