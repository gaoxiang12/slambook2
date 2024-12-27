# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 2.8)

# 声明一个 cmake 工程
project(HelloSLAM)

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
# add_executable(helloSLAM helloSLAM.cpp)
add_executable(helloSLAM helloSLAM2.cpp)

add_library( hello libHelloSLAM.cpp )

add_library( hello_shared SHARED libHelloSLAM.cpp )

add_executable( useHello useHello.cpp )
target_link_libraries( useHello hello_shared )

set(CMAKE_BUILD_TYPE "Debug")      # 若设置为"Release"模式则改为"Debug"，否则断点设置无效