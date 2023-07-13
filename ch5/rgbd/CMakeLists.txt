find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

find_package(FMT REQUIRED)
set(FMT_LIBRARIES fmt::fmt)
include_directories(${FMT_LIBRARIES})
target_link_libraries(joinMap ${FMT_LIBRARIES})
