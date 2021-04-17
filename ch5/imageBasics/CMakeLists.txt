project(imageBasics)

#set(Opencv_INCLUDE_DIRS "/usr/local/opt/opencv@3/include")
#set(Opencv_LIBRARIES_DIRS "/usr/local/opt/opencv@3/lib")
set(CMAKE_MODULE_PATH "/usr/local/opt/opencv@3/share/OpenCV")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(imageBasics imageBasics.cpp)
# 链接OpenCV库
target_link_libraries(imageBasics ${OpenCV_LIBS})

add_executable(undistortImage undistortImage.cpp)
# 链接OpenCV库
target_link_libraries(undistortImage ${OpenCV_LIBS})
