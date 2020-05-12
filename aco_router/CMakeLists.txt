cmake_minimum_required(VERSION 3.10)
project(aco_tsp)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_library(acolib vrp_solver.h vrp_solver.cpp tsp_solver.h tsp_solver.cpp utils.h utils_impl.h types.h types.cpp utils.cpp)

include_directories(test)

add_executable(test_tsp test/test_tsp.cpp)
add_executable(test_vrp test/test_vrp.cpp)

target_link_libraries(test_tsp acolib)
target_link_libraries(test_vrp acolib)