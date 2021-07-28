cmake_minimum_required(VERSION 3.19)
project(E01)

set(CMAKE_CXX_STANDARD 14)
find_package(Sophus REQUIRED)
include_directories("usr/include/eigen3")

add_executable(E01 main.cpp)
target_link_libraries(E01 Sophus::Sophus)