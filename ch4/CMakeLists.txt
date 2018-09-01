cmake_minimum_required(VERSION 2.8)
project(useSophus)

# 为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")
add_executable(useSophus useSophus.cpp)

add_subdirectory(example)
