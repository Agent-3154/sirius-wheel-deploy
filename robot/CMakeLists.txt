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
include_directories("hardwares/fdsc_utils/inc")
include_directories("../robot_ctrl")
include_directories(${INC_DIR})

include_directories("../utilities/types")
include_directories("../robot_ctrl/")
#include_directories("../absl_repo/absl")
include_directories(../mujoco-3.1.3/include)

link_directories("../mujoco-3.1.3/lib")
link_directories(${LINK_DIR})

file(GLOB_RECURSE hardware_sources "hardwares/usb/src/*.cpp" "hardwares/fdsc_utils/src/*.cpp" "../utilities/src/*.cpp" "*.cpp"
        "estimators/*.cpp" "leg_controller/*.cpp" "thread_tasks/src/*.cpp" "thread_tasks/*.cpp" "robot_model/*.cpp"
        "hardwares/usb/src/*.cpp" "robot_runner/*.cpp")

add_library(hardware_library SHARED ${hardware_sources}
        thread_tasks/inc/thread_usb_hardware.h
        hardwares/usb/include/rt_usb_base.h
        hardwares/usb/src/rt_usb_base.cpp
        thread_tasks/src/thread_usb_hardware.cpp
        ../config/Config.h
        estimators/sim_estimator.h
        estimators/sim_estimator.cpp
        thread_tasks/inc/thread_fdsc.h
        hardwares/fdsc_utils/my_fdsc.h
        hardwares/fdsc_utils/my_fdsc.cpp
        thread_tasks/src/thread_fdsc.cpp
#        ../utilities/src/easylogging++.cc
)

target_link_libraries(hardware_library
        lcm libusb-1.0.so libmujoco.so)