cmake_minimum_required(VERSION 3.5)
project(vrp_solver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp matr.cpp utils.cpp gen-cpp/vrp_solver_constants.cpp gen-cpp/VRPSolver.cpp
        gen-cpp/vrp_solver_types.cpp gen-cpp/VRPSolver_server.cpp)



include_directories(/home/metis/Desktop/ortools/or-tools.Linux64/include)
include_directories(/home/metis/Downloads/or-tools-2016-04/src)


link_directories(/usr/local/lib)
link_directories(/home/metis/Desktop/ortools/or-tools.Linux64/lib)



add_executable(vrp_solver ${SOURCE_FILES})


target_link_libraries(vrp_solver ortools thrift thriftnb event )