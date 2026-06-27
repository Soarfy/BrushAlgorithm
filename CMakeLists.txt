cmake_minimum_required(VERSION 3.13)
project(ArmController)

# ============================================================
# 基础设置
# ============================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (MSVC)
    add_compile_options(/utf-8)
endif()

# ============================================================
# 所有 exe 输出到同一个 bin 目录
# ============================================================

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 对 VS 多配置生成器（Debug/Release）分别指定
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin/Release)
set(NLOHMANN_ROOT "D:/UsmileProject/nlohmann")

# ============================================================
# Force Six 设置
# ============================================================

set(ForceBase_ROOT "D:/UsmileProject/ArmControllerCameraNew")
set(ForceSix_ROOT "${ForceBase_ROOT}/console_demo/include")
set(ForceSix_LIB "${ForceBase_ROOT}/console_demo/lib/windows_amd64/lib/kw-cpp-lib.lib")

# ============================================================
# Eigen 设置
# ============================================================

set(Eigen3_DIR
    "D:/UsmileProject/ArmControllerCameraNew/force_control/thirdparty/eigen-3.4.0/eigen3"
)

# ============================================================
# 子目录
# ============================================================

add_subdirectory(cpp_example)

# ============================================================
# 公共静态库
# ============================================================

add_library(ArmCommon STATIC
    TcpConfigHelper.cpp
    DobotTcpDemo.cpp
    api/BitConverter.cpp
    api/Dashboard.cpp
    api/DobotClient.cpp
    api/ErrorInfoBean.cpp
    api/ErrorInfoHelper.cpp
    api/Feedback.cpp
)

target_include_directories(ArmCommon PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp_example
    ${ForceSix_ROOT}
    ${Eigen3_DIR}
    ${NLOHMANN_ROOT}
)

target_link_libraries(ArmCommon PUBLIC
    cpp_example_lib
    ${ForceSix_LIB}
)

target_compile_options(ArmCommon PRIVATE /bigobj)

target_compile_definitions(ArmCommon PRIVATE
    $<$<CONFIG:Debug>:ARM_CONTROLLER_DEBUG>
    $<$<CONFIG:Release>:ARM_CONTROLLER_RELEASE>
)

# ============================================================
# 需要生成的 exe
# ============================================================

set(ARM_EXECUTABLES
    ArmControllerGenerateModel
    ArmControllerBrushCenter
    ArmControllerBrushUpRight
    ArmControllerBrushUpLeft
    ArmControllerBrushSideRight
    ArmControllerBrushSideAhead
    ArmControllerBrushSideLeft
    ArmControllerBrushSideLeftboom
    ArmControllerBrushInSideRight
    ArmControllerBrushInSideLeft
    ArmControllerBrushAllPath
    BrushTooth
    BrushToothcalibrate
    CaptureImg
    ArmControllerGeneratebrush
    ArmControllerBrushSideRightdelayrepaired
)

# ============================================================
# 批量生成 exe（全部放一个文件夹）
# ============================================================

foreach(target ${ARM_EXECUTABLES})

    add_executable(${target} ${target}.cpp)

    target_link_libraries(${target} PRIVATE ArmCommon)

endforeach()

message(STATUS "All executables will output to: ${CMAKE_BINARY_DIR}/bin")
