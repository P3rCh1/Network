cmake_minimum_required(VERSION 3.31)
project(Network)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(CMAKE_CXX_STANDARD 20)

add_executable(Network main.cpp)
