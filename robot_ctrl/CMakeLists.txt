set(INC_DIR /usr/local/include)
set(LINK_DIR /usr/local/lib)

set(CMAKE_CXX_STANDARD 17)
include_directories(${INC_DIR})

include_directories("../utilities/types")
include_directories("../utilities/inc")
#include_directories("../absl_repo/absl")
include_directories("../mujoco-3.1.3/include")
include_directories("../third-party/qpOASES/include")
include_directories("../third-party/")
include_directories("FSM/")
include_directories("gait_scheduler/")
include_directories("mpc/")
include_directories("planner/")
include_directories("wbc/wbic/")
link_directories("../mujoco-3.1.3/lib")
link_directories(${LINK_DIR})

file(GLOB_RECURSE robot_ctrl_sources "*.cpp" "FSM/*.cpp")
find_package(iceoryx_platform REQUIRED)
find_package(iceoryx_posh CONFIG REQUIRED)
find_package(iceoryx_hoofs CONFIG REQUIRED)

add_library(robot_ctrl_library SHARED ${robot_ctrl_sources}
        mpc/thread_mpc.cpp
        planner/thread_planner.cpp
        FSM/FSM_State_damping.h
        FSM/FSM_State_damping.cpp
        ../utilities/src/easylogging++.cc
)

target_link_libraries(robot_ctrl_library
#        absl::check
#        absl::flat_hash_map
#        absl::log
#        absl::random_random
#        absl::span
#        absl::flags
#        absl::flags_parse
        lcm libmujoco.so wbc_library solver_library iceoryx_posh::iceoryx_posh)

add_subdirectory(wbc)
add_subdirectory(solver)