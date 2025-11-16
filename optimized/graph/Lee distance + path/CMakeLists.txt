cmake_minimum_required (VERSION 3.29)
project (Lee_distance___path)

set (CMAKE_CXX_STANDARD 20)

add_executable (Lee_distance___path main.cpp)

add_compile_options(
        -x c++
        -std=gnu++20
        -Wall
        -O2
        -static
        -pipe
        -DONLINE_JUDGE
        -DDOMJUDGE
)