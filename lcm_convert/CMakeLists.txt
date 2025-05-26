cmake_minimum_required(VERSION 3.10)

project(lcm_reader)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
include_directories("../utilities/inc/")

#add_executable(lcm_reader lcm_reader.cpp ../utilities/src/easylogging++.cc)
#target_link_libraries(lcm_reader lcm )
