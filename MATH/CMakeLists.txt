cmake_minimum_required(VERSION 3.19)
project(MATH)

set(CMAKE_CXX_STANDARD 17)

add_executable(MATH main.cpp MATH_CLASS.h jni.h jni_md.h main_ex.cpp)