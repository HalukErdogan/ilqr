cmake_minimum_required(VERSION 3.22)

project(ilqr VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
option(BUILD_TESTS "Build tests" ON)
option(BUILD_EXAMPLES "Build examples" ON)

find_package(Eigen3 CONFIG REQUIRED)

add_library(ilqr INTERFACE)
target_link_libraries(ilqr INTERFACE Eigen3::Eigen)
target_include_directories(ilqr INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()