set(CMAKE_CXX_STANDARD 17)
set(INC_DIR /usr/local/include)
set(LINK_DIR /usr/local/lib)

include_directories(tasks)
include_directories(wbc_ctrl)
include_directories(wbic)
include_directories(${INC_DIR})
link_directories(${LINK_DIR})

add_subdirectory(QuadProg++)

file(GLOB_RECURSE wbc_library_sources "*.cpp" "tasks/*.cpp" "wbc_ctrl/*.cpp" "wbic/*cpp")
#file(GLOB_RECURSE wbc_library_sources "*.cpp")

add_library(wbc_library SHARED ${wbc_library_sources})
target_link_libraries(wbc_library Goldfarb_Optimizer lcm)
#target_link_libraries(wbc_library
