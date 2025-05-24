cmake_minimum_required(VERSION 3.30.5)
project(Network)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(CMAKE_CXX_STANDARD 14)

add_executable(Network main.cpp graph.h hash_map.h fwd_list.h vector_hash.h
        command_handler.cpp command_handler.h
        network_app_algorithm.cpp
        network_app_algorithm.h)
