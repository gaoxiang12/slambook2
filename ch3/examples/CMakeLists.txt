include_directories("/usr/include/eigen3")
add_executable(coordinateTransform coordinateTransform.cpp)

set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
