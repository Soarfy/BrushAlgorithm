cmake_minimum_required(VERSION 3.10)
project(cpp_example_lib)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV 路径
set(OpenCV_DIR "D:/UsmileProject/ArmControllerCamera/3rdparty/opencv420")

set(Lib_DIR "D:/UsmileProject/ArmControllerCamera/cpp_example")


include_directories(${OpenCV_DIR}/include)

# 本地库头文件
include_directories(${CMAKE_SOURCE_DIR})

# 源文件
add_library(cpp_example_lib STATIC VideoCapture.cpp)

# 链接 OpenCV 静态库
target_link_libraries(cpp_example_lib
    ${OpenCV_DIR}/lib/opencv_core420.lib
    ${OpenCV_DIR}/lib/opencv_highgui420.lib
    ${OpenCV_DIR}/lib/opencv_imgcodecs420.lib
    ${OpenCV_DIR}/lib/opencv_imgproc420.lib
    ${OpenCV_DIR}/lib/opencv_videoio420.lib
    ${Lib_DIR}/camera.lib
    ${Lib_DIR}/enumerate.lib
) 