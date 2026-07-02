# BrushAlgorithm

Dobot 机械臂智能拂刷控制系统 - 支持力控轨迹执行、视觉感知与多区域自动拂刷。

## 功能特性

- **TCP 通讯控制**：通过 Dobot TCP 协议与机械臂通信，支持 Dashboard 和 Feedback 双通道
- **力控轨迹执行**：集成 Force Six 力控模块，实时调整末端位姿保持恒定接触力
- **视觉感知**：OpenCV 相机支持，可捕获刷拭过程中的图像数据
- **MovS 轨迹拟合**：Smooth 轨迹模式，实现多点的平滑轨迹运动
- **拖拽示教微调**：支持手动拖拽机械臂进行姿态微调
- **多区域拂刷**：支持牙面中心、左右侧面、左右上颌、内侧等多区域的自动拂刷

## 目录结构

```
BrushAlgorithm/
├── api/                    # Dobot TCP API 封装
│   ├── Dashboard.cpp/h     # 控制指令接口
│   ├── Feedback.cpp/h      # 状态反馈接口
│   └── ...
├── cpp_example/            # 相机图像采集示例
├── 3rdparty/               # 第三方依赖库
│   └── opencv420/          # OpenCV 4.2.0
├── ArmController*.cpp      # 各区域拂刷控制器
├── DobotTcpDemo.cpp/h      # TCP 通信核心类
├── TcpConfigHelper.cpp/h   # TCP 配置辅助
├── ForceTrajectoryIO.h     # 力控轨迹文件读写
├── BrushRetreatHelper.h    # 拂刷退避辅助
├── TrajectoryRetreatHelper.cpp  # 轨迹退避辅助
├── BrushDemoConfig.h       # 拂刷演示配置
└── CMakeLists.txt          # CMake 构建配置
```

## 拂刷模块

| 模块 | 功能 |
|------|------|
| `ArmControllerBrushCenter` | 牙面中心区域拂刷 |
| `ArmControllerBrushSideLeft` | 左侧牙面拂刷 |
| `ArmControllerBrushSideRight` | 右侧牙面拂刷 |
| `ArmControllerBrushUpLeft` | 左上颌拂刷 |
| `ArmControllerBrushUpRight` | 右上颌拂刷 |
| `ArmControllerBrushInSideLeft` | 左侧内侧拂刷 |
| `ArmControllerBrushInSideRight` | 右侧内侧拂刷 |
| `ArmControllerBrushSideAhead` | 前侧拂刷 |

## 构建

### 环境要求

- CMake >= 3.13
- MSVC C++ 编译器 (C++17)
- OpenCV 4.2.0
- Eigen 3.4.0
- nlohmann/json
- Force Six SDK

### 构建步骤

```bash
# 创建构建目录
mkdir build && cd build

# 生成 Visual Studio 项目
cmake .. -G "Visual Studio 17 2022"

# 编译
cmake --build . --config Release

# 可执行文件输出到 build/bin/Release/
```

## 使用说明

### 基本运行

1. 连接 Dobot 机械臂
2. 配置 TCP 连接参数（IP 和端口）
3. 选择对应的拂刷模式并运行

### 力控轨迹调整

程序支持力控模式下的轨迹实时调整：
- 开启力控后，系统会根据接触力自动微调 z 高度
- 支持键盘微调（上下左右前后）
- 支持拖拽示教直接调整姿态

### 轨迹文件格式

力控轨迹文件为文本格式，每行 6 个浮点数（空格分隔）：

```
x y z rx ry rz
x y z rx ry rz
...
```

支持 `#` 注释行和空行跳过。

## 依赖

| 库 | 版本 | 用途 |
|----|------|------|
| Dobot API | - | 机械臂 TCP 通信 |
| OpenCV | 4.2.0 | 图像处理 |
| Eigen | 3.4.0 | 矩阵运算 |
| nlohmann/json | 3.x | JSON 配置解析 |
| Force Six SDK | - | 力控模块 |

## 分支说明

- `main`: 稳定版本
- `dev`: 开发版本，包含最新功能和改进
