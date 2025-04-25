set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE solver_library_sources "src/*.cpp" "*.cpp")

add_library(solver_library SHARED ${solver_library_sources}
        src/Linear_QP_Solver.cpp)
target_link_libraries(solver_library qpOASES)