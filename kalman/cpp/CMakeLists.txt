cmake_minimum_required(VERSION 3.15)
project(KalmanFilters LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_EXAMPLES "Build example executables" ON)

add_subdirectory(Lib)

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
