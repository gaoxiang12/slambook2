option(USE_UBUNTU_20 "Set to ON if you are using Ubuntu 20.04" OFF)
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
if(USE_UBUNTU_20)
    message("You are using Ubuntu 20.04, fmt::fmt will be linked")
    find_package(fmt REQUIRED)
    set(FMT_LIBRARIES fmt::fmt)
endif()

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${FMT_LIBRARIES})
