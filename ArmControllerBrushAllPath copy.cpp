#include "ArmController.h"
#include "Define.h"
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include "VideoCapture.hpp"
#include <iostream>
#include <vector>
#include <crtdbg.h>
#include <conio.h>
#include <fstream>

// 直接使用六维力原始代码
#include "kw-lib-all.h"
#define MODE 0
bool capturing = true;
NS_KW_USING

// 弧度定义
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// 将弧度转为角度
constexpr double RAD2DEG = 180.0 / M_PI;

/**
 * @brief 构造函数，初始化机器人控制器
 * @param ip 机器人IP地址，默认为"192.168.101.100"
 * @param port 机器人端口号，默认为"9000"
 */
ArmController::ArmController(const std::string &ip, const std::string &port)
{
    api_ = std::make_unique<c2::CodroidApi>(ip, port);
    connected_ = false;
}

/**
 * @brief 析构函数，确保断开连接
 */
ArmController::~ArmController()
{
    if (connected_)
    {
        disconnect();
    }
}

/**
 * @brief 连接到机器人（检测是否正常）
 * @return 连接是否成功
 */
bool ArmController::isconnected()
{
    if (!api_)
    {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }

    // 获取机器人状态
    auto res = api_->getRobotState();
    if (res.data != 0)
    {
        std::cerr << "Failed to get robot state: " << res.msg << std::endl;
        return false;
    }

    // 检查机器人状态是否为Ready或Auto模式
    int state = res.data;
    if (state != static_cast<int>(c2::RobotState::Ready) &&
        state != static_cast<int>(c2::RobotState::Auto))
    {
        // 尝试切换到Ready模式
        res = api_->sendUserCommand(c2::UserCommand::ToReady);
        if (res.code != c2::ResponseCode::OK)
        {
            std::cerr << "Failed to switch to Ready mode: " << res.msg << std::endl;
            return false;
        }
    }

    connected_ = true;
    return true;
}

/**
 * @brief 断开与机器人的连接（断电）
 * @return 断开连接是否成功
 */
bool ArmController::disconnect()
{
    if (!api_)
    {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }
    auto res = api_->sendUserCommand(c2::UserCommand::SwitchOff);
    if (res.code != c2::ResponseCode::OK)
    {
        std::cerr << "Failed to switch off robot: " << res.msg << std::endl;
        return false;
    }

    // 检查机器人状态是否为StandBy
    res = api_->getRobotState();
    if (res.code != c2::ResponseCode::OK)
    {
        std::cerr << "Failed to get robot state: " << res.msg << std::endl;
        return false;
    }

    int state = res.data;
    if (state != static_cast<int>(c2::RobotState::StandBy))
    {
        std::cerr << "Robot did not enter StandBy state after switch off" << std::endl;
        return false;
    }

    connected_ = false;
    return true;
}

/**
 * @brief 打开夹爪
 * @param port 控制夹爪的数字输出端口号
 * @return 操作是否成功
 */
bool ArmController::openGripper(int port)
{
    if (!api_)
    {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }

    auto res = api_->setDO(port, 1); // 设置数字输出为高电平，打开夹爪
    return res.code == c2::ResponseCode::OK;
}

/**
 * @brief 关闭夹爪
 * @param port 控制夹爪的数字输出端口号
 * @return 操作是否成功
 */
bool ArmController::closeGripper(int port)
{
    if (!api_)
    {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }

    auto res = api_->setDO(port, 0); // 设置数字输出为低电平，关闭夹爪
    return res.code == c2::ResponseCode::OK;
}

/**
 * @brief 检查夹爪状态
 * @param port 控制夹爪的数字输出端口号
 * @return 夹爪是否处于打开状态
 */
bool ArmController::isGripperOpen(int port) const
{
    if (!api_)
    {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }

    auto res = api_->getDI(port); // 读取数字输入状态
    if (res.code == c2::ResponseCode::OK)
    {
        return res.data.get<int>() == 1; // 如果输入为高电平，则夹爪处于打开状态
    }
    return false;
}

/**
 * @brief 执行笛卡尔轨迹段
 * @param segments 需要执行的笛卡尔轨迹段集合
 * @return 操作是否成功
 */
bool ArmController::StartBrushTeethTrajectoryJoint(const c2::MovJointSegments &segments)
{
    if (!api_)
    {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }
    auto res = api_->movJointSegments(segments);
    if (res.code != c2::ResponseCode::OK)
    {
        std::cout << "[执行中断] movJointSegments 执行失败: " << res.msg << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 设置Home位置
 * @param position 包含6个关节角度的vector
 * @return 设置是否成功
 */
bool ArmController::setHomePosition(const std::vector<double> &position)
{
    // 检查api_是否已初始化
    if (!api_)
    {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }

    // 检查输入的关节角度数量是否为6
    if (position.size() != 6)
    {
        std::cerr << "Home position must have exactly 6 joint angles." << std::endl;
        return false;
    }

    // 拷贝vector到数组
    double pos[6];
    std::copy(position.begin(), position.end(), pos);

    // 调用API设置Home位置
    api_->setHomePosition(pos);
    return true;
}

/**
 * @brief 解析yaml文件，生成关节空间轨迹（MovJointSegments）
 *
 * 该函数从指定的yaml文件中读取关节角度点（Points），
 * 并将其转换为MovJointSegments结构体，用于机械臂的关节空间运动轨迹规划。
 *
 * @param yaml_path yaml文件路径，文件中应包含"Points"字段，每个点为6维关节角度
 * @param speed 关节运动速度（单位：度/秒）
 * @param acc 关节运动加速度（单位：度/秒^2）
 * @return 生成的MovJointSegments轨迹段集合
 */
c2::MovJointSegments ArmController::parseYamlToMovJointSegments(const std::string &yaml_path, const c2::Speed &speed, const c2::Acc &acc)
{
    c2::MovJointSegments segments;
    try
    {
        YAML::Node root = YAML::LoadFile(yaml_path);
        YAML::Node points = root["Points"];
        if (!points || !points.IsSequence())
        {
            std::cerr << "Invalid or missing 'Points' section in YAML." << std::endl;
            return segments;
        }
        for (const auto &point : points)
        {
            if (point["apos"] && point["apos"].IsSequence() && point["apos"].size() == 6)
            {
                c2::Point p;
                p.type = c2::PointType::Joint;
                for (size_t i = 0; i < 6; ++i)
                {
                    p.apos.jntPos[i] = point["apos"][i].as<double>() * RAD2DEG;
                }
                c2::Zone z;
                z.per = 50;
                z.dis = 60;
                segments.AddMovJ(p, speed, acc, c2::ZoneType::Relative, z);
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading YAML: " << e.what() << std::endl;
    }
    return segments;
}

/**
 * @brief 根据点云和法向量生成笛卡尔空间轨迹（MovCartSegments）(先试用python仿真)
 *
 * 该函数根据给定的点云和法向量，生成笛卡尔空间的运动轨迹。
 * speed和acc参数现在应为c2::Speed和c2::Acc类型，允许分别设置各分量。
 * 它首先遍历指定的点索引（ids），对于每个点，计算其扩展后的新位置，
 * 然后构造一个MovL（直线插补）类型的轨迹段，并添加到轨迹集合中。
 */
std::vector<c2::MovCartSegments> ArmController::createCartTrajectoryFromPointCloud(
    const open3d::geometry::PointCloud &cloud,
    const std::vector<int> &ids,
    const c2::Speed &speed,
    const c2::Acc &acc,
    double extend_dist)
{
    std::vector<c2::MovCartSegments> traj_vec;
    c2::Zone z;
    z.per = 0;
    z.dis = 0;
    size_t count = 0;
    c2::MovCartSegments traj;
    for (int idx : ids)
    {
        if (idx < 0 || idx >= static_cast<int>(cloud.points_.size()) || idx >= static_cast<int>(cloud.normals_.size()))
            continue;
        const auto &pt = cloud.points_[idx];
        const auto &n = cloud.normals_[idx];
        Eigen::Vector3d new_pt = pt + n.normalized() * extend_dist;
        c2::Point cpos;
        cpos.type = c2::PointType::Cart;
        cpos.cpos.x = new_pt.x();
        cpos.cpos.y = new_pt.y();
        cpos.cpos.z = new_pt.z();

        // 计算姿态：法向量与xyz坐标系的旋转角(需要根据python仿真)
        // 假设工具z轴对准法向量，x轴在xy平面投影指向x正方向
        Eigen::Vector3d z_axis = n.normalized();
        Eigen::Vector3d x_ref(1, 0, 0);
        Eigen::Vector3d y_axis = z_axis.cross(x_ref);
        if (y_axis.norm() < 1e-6)
        {
            // 法向量接近x轴，选择y轴为参考
            x_ref = Eigen::Vector3d(0, 1, 0);
            y_axis = z_axis.cross(x_ref);
        }
        y_axis.normalize();
        Eigen::Vector3d x_axis = y_axis.cross(z_axis);
        x_axis.normalize();

        // 构造旋转矩阵
        Eigen::Matrix3d rot;
        rot.col(0) = x_axis;
        rot.col(1) = y_axis;
        rot.col(2) = z_axis;

        // 提取欧拉角（ZYX顺序，单位为弧度）
        Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0); // ZYX
        cpos.cpos.a = euler[2];                           // X
        cpos.cpos.b = euler[1];                           // Y
        cpos.cpos.c = euler[0];                           // Z

        traj.AddMovL(cpos, speed, acc, c2::ZoneType::Fine, z);
        ++count;
        if (count == 28)
        {
            traj_vec.push_back(traj);
            traj = c2::MovCartSegments();
            count = 0;
        }
    }
    if (count > 0)
    {
        traj_vec.push_back(traj);
    }
    return traj_vec;
}

/**
 * @brief 根据点云和法向量生成关节空间轨迹，考虑工具标定参数
 * @param cloud 点云（已在机械臂坐标系下）
 * @param ids 轨迹点在点云中的索引（每28个点分为一组）
 * @param speed 关节运动速度（c2::Speed类型）
 * @param acc 关节运动加速度（c2::Acc类型）
 * @param tool_calibration 工具标定参数（x, y, z, roll, pitch, yaw），默认全为0
 * @param extend_dist 沿法向量延伸距离（米），默认0.05m
 * @return 按顺序分组的关节空间轨迹段集合vector
 */
std::vector<c2::MovJointSegments> ArmController::createJointTrajectoryFromPointCloud(
    const open3d::geometry::PointCloud &cloud,
    const std::vector<int> &ids,
    const c2::Speed &speed,
    const c2::Acc &acc,
    const std::vector<double> &tool_calibration,
    double extend_dist)
{
    std::vector<c2::MovJointSegments> traj_vec;
    c2::Zone z;
    z.per = 0;
    z.dis = 0;
    size_t count = 0;
    c2::MovJointSegments traj;

    // Check tool calibration vector size
    if (tool_calibration.size() != 6)
    {
        std::cerr << "[createJointTrajectoryFromPointCloud] Tool calibration vector must have 6 elements. Using default values." << std::endl;
    }

    for (int idx : ids)
    {
        if (idx < 0 || idx >= static_cast<int>(cloud.points_.size()) || idx >= static_cast<int>(cloud.normals_.size()))
            continue;

        const auto &pt = cloud.points_[idx];
        const auto &n = cloud.normals_[idx];
        Eigen::Vector3d new_pt = pt + n.normalized() * extend_dist;

        // Apply tool calibration offset to Cartesian coordinates
        Eigen::Vector3d calibrated_pt = new_pt;
        if (tool_calibration.size() >= 3)
        {
            calibrated_pt.x() -= tool_calibration[0];
            calibrated_pt.y() -= tool_calibration[1];
            calibrated_pt.z() -= tool_calibration[2];
        }

        // Calculate orientation: normal vector aligned with z-axis（这里需要存疑，可能直接剪掉就可以了今天去看看）
        Eigen::Vector3d z_axis = n.normalized();
        Eigen::Vector3d x_ref(1, 0, 0);
        Eigen::Vector3d y_axis = z_axis.cross(x_ref);
        if (y_axis.norm() < 1e-6)
        {
            x_ref = Eigen::Vector3d(0, 1, 0);
            y_axis = z_axis.cross(x_ref);
        }
        y_axis.normalize();
        Eigen::Vector3d x_axis = y_axis.cross(z_axis);
        x_axis.normalize();

        // Construct rotation matrix
        Eigen::Matrix3d rot;
        rot.col(0) = x_axis;
        rot.col(1) = y_axis;
        rot.col(2) = z_axis;

        // Apply tool calibration rotation if available
        if (tool_calibration.size() >= 6)
        {
            double rx = tool_calibration[3], ry = tool_calibration[4], rz = tool_calibration[5];
            Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());
            Eigen::Matrix3d tool_rot = (Rz * Ry * Rx).toRotationMatrix();
            rot = rot * tool_rot;
        }

        // Extract Euler angles (ZYX order, in radians)
        Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0); // ZYX

        // Create Cartesian position for inverse kinematics
        c2::Point cpos;
        cpos.type = c2::PointType::Cart;
        cpos.cpos.x = calibrated_pt.x();
        cpos.cpos.y = calibrated_pt.y();
        cpos.cpos.z = calibrated_pt.z();
        cpos.cpos.a = euler[2]; // X
        cpos.cpos.b = euler[1]; // Y
        cpos.cpos.c = euler[0]; // Z

        // 直接传入笛卡尔点，无需手动转换为关节空间
        traj.AddMovJ(cpos, speed, acc, c2::ZoneType::Fine, z);
        ++count;
        if (count == 28)
        {
            traj_vec.push_back(traj);
            traj = c2::MovJointSegments();
            count = 0;
        }
    }

    if (count > 0)
    {
        traj_vec.push_back(traj);
    }

    return traj_vec;
}

bool ArmController::StartBrushTeethTrajectoryCart(const c2::MovCartSegments &traj)
{
    if (!api_)
    {
        std::cerr << "api_ is not initialized." << std::endl;
        return false;
    }
    auto res = api_->movCartSegments(traj);
    if (res.code != c2::ResponseCode::OK)
    {
        std::cerr << "[执行中断] movCartSegments 执行失败: " << res.msg << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 执行指定的project
 * @param projectName 工程名称
 * @param taskName 任务名称（可选）
 * @return true表示执行成功，false表示失败
 */
bool ArmController::runProject(const std::string &projectName, const std::string &taskName)
{
    if (!api_)
        return false;
    auto res = api_->runProject(projectName, taskName);
    if (res.code != c2::ResponseCode::OK)
    {
        std::cerr << "runProject failed: " << res.msg << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 获取当前机械臂状态
 * @return 状态码（参考c2::RobotState），失败返回-1
 */
int ArmController::getRobotState()
{
    if (!api_)
        return -1;
    auto res = api_->getRobotState();
    if (res.code != c2::ResponseCode::OK)
    {
        std::cerr << "getRobotState failed: " << res.msg << std::endl;
        return -1;
    }
    return res.data.get<int>();
}

/**
 * @brief 获取当前机械臂末端的笛卡尔坐标系位姿
 * @param out_pose 输出参数，返回位姿（c2::CPos）
 * @return true表示获取成功，false表示失败
 */
bool ArmController::getCartPosition(c2::CPos &out_pose)
{
    if (!api_)
        return false;
    auto res = api_->getCartPosition();
    if (res.code != c2::ResponseCode::OK)
    {
        std::cerr << "getCartPosition failed: " << res.msg << std::endl;
        return false;
    }
    // res.data 是一个 double 数组，顺序为 x, y, z, a, b, c
    try
    {
        auto arr = res.data;
        if (!arr.is_array() || arr.size() < 6)
        {
            std::cerr << "getCartPosition: data is not a valid array." << std::endl;
            return false;
        }
        out_pose.x = arr[0].get<double>();
        out_pose.y = arr[1].get<double>();
        out_pose.z = arr[2].get<double>();
        out_pose.a = arr[3].get<double>();
        out_pose.b = arr[4].get<double>();
        out_pose.c = arr[5].get<double>();
    }
    catch (const std::exception &e)
    {
        std::cerr << "getCartPosition: parse error: " << e.what() << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 改变机械臂模式（如切换到手动/自动等）
 * @param mode 目标模式（参考c2::UserCommand）
 * @return true表示切换成功，false表示失败
 */
bool ArmController::changeRobotMode(c2::UserCommand mode)
{
    if (!api_)
        return false;
    auto res = api_->sendUserCommand(mode);
    if (res.code != c2::ResponseCode::OK)
    {
        std::cerr << "changeRobotMode failed: " << res.msg << std::endl;
        return false;
    }
    return true;
}

std::vector<double> ArmController::ToolsCalibration()
{
    std::vector<c2::CPos> poses;
    std::vector<double> result(6, 0.0);
    // 1. Switch to manual mode
    if (!changeRobotMode(c2::UserCommand::ToReady))
    {
        std::cerr << "[ToolsCalibration] Failed to switch to Manual mode." << std::endl;
        return result;
    }
    std::cout << "[ToolsCalibration] Entering manual mode. Please move the robot arm to 4 different calibration points. After each move, press Enter to record the pose." << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        // 需要避免某个位置重复采样
        std::cout << "Move the robot arm to calibration point " << (i + 1) << " and press Enter..." << std::endl;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        c2::CPos pose;
        if (!getCartPosition(pose))
        {
            std::cerr << "[ToolsCalibration] Failed to get pose for point " << (i + 1) << "." << std::endl;
            continue;
        }
        poses.push_back(pose);
        std::cout << "Recorded pose: x=" << pose.x << ", y=" << pose.y << ", z=" << pose.z
                  << ", a=" << pose.a << ", b=" << pose.b << ", c=" << pose.c << std::endl;
    }
    if (poses.size() < 4)
    {
        std::cerr << "[ToolsCalibration] Less than 4 calibration points recorded. Returning zero vector." << std::endl;
        return result;
    }
    // Simple processing: use the mean of the 4 points as the calibration result
    // 使用四点法估算 TCP 在法兰坐标系下的位置
    // 1. 将 poses 转换为 4x4 齐次变换矩阵
    std::vector<Eigen::Matrix4d> T_list;
    for (const auto &p : poses)
    {
        double rx = p.a, ry = p.b, rz = p.c;
        Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d R = (Rz * Ry * Rx).toRotationMatrix();
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = Eigen::Vector3d(p.x, p.y, p.z);
        T_list.push_back(T);
    }
    // 2. 构造 A, b
    const Eigen::Matrix3d &R0 = T_list[0].block<3, 3>(0, 0);
    const Eigen::Vector3d &p0 = T_list[0].block<3, 1>(0, 3);
    Eigen::MatrixXd A(3 * (T_list.size() - 1), 3);
    Eigen::VectorXd b(3 * (T_list.size() - 1));
    for (size_t i = 1; i < T_list.size(); ++i)
    {
        const Eigen::Matrix3d &Ri = T_list[i].block<3, 3>(0, 0);
        const Eigen::Vector3d &pi = T_list[i].block<3, 1>(0, 3);
        A.block<3, 3>(3 * (i - 1), 0) = Ri - R0;
        b.segment<3>(3 * (i - 1)) = p0 - pi;
    }
    // 3. 最小二乘求解
    Eigen::Vector3d tcp_pos = A.colPivHouseholderQr().solve(b);
    result[0] = tcp_pos(0);
    result[1] = tcp_pos(1);
    result[2] = tcp_pos(2);
    std::cout << "[ToolsCalibration] Calibration result (TCP in flange frame): x=" << result[0]
              << ", y=" << result[1] << ", z=" << result[2] << std::endl;

    // 旋转标定
    std::cout << "[ToolsCalibration] Please manually rotate the robot tool so that the polar coordinates are aligned as desired, then press Enter to continue..." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Wait for user to press Enter

    c2::CPos rot_pose;
    if (!getCartPosition(rot_pose))
    {
        std::cerr << "[ToolsCalibration] Failed to get pose for rotation calibration, setting last 3 values to 0." << std::endl;
        result[3] = 0;
        result[4] = 0;
        result[5] = 0;
    }
    else
    {
        result[3] = rot_pose.a;
        result[4] = rot_pose.b;
        result[5] = rot_pose.c;
        std::cout << "[ToolsCalibration] Recorded rotation calibration: a=" << result[3]
                  << ", b=" << result[4] << ", c=" << result[5] << std::endl;
    }
    return result;
}

int main()
{
    std::cout << "Start BrushAllPath" << std::endl;
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@机械臂初始化@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    ArmController arm;

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@相机初始化@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    CameraCapture camera("169.254.7.168");

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@6维力初始化@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#if MODE == 0
    SerialControlCreator uc;
#ifdef WIN32
    uc.serialPortName = "\\\\.\\COM3";
#else
    uc.serialPortName = "/dev/ttyUSB0";
#endif
    uc.baudRate = 460800;

#elif MODE == 1
    UdpControlCreator uc;
    uc.sensorIp = "192.168.1.101";
    uc.localIp = "192.168.1.100";
    uc.localPort = 8886;

#elif MODE == 2
#endif
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@初始化6维力对象@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    auto control = uc.createIOControler();
    HeadTailProtocolCreator htc;
    auto proto = htc.createProtocol();
    SensorControlCreator scc;
    scc.ioCtrl = control;
    scc.proto = proto;
    auto obj = scc.createSensorControl();
    int hr = obj->StartCapture();
    if (hr != 0)
    {
        printf("start capture faield\n");
        return hr;
    }

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@检测机械臂链接状态@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    std::cout << "isconnected: " << arm.isconnected() << std::endl;
    std::cout << "changeRobotMode: " << arm.changeRobotMode(c2::UserCommand::ToAuto) << std::endl;
    struct PointData
    {
        double x, y, z, a, b, c;
    };

    struct BrushVector
    {
        double x, y, z;
    };

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@回到初始态@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    c2::Zone zone;
    zone.dis = 0;
    zone.per = 0;
    c2::Speed speed{880, 880, 880};
    c2::Acc acc{380, 380, 380};

    std::vector<PointData> points;
    points = {
        // 牙齿和牙刷正对拍摄位置
        {25.411, 452.19, 422.686, -179.534, -0.215, 90.558},
    };

    for (const auto &pt : points)
    {
        c2::MovJointSegments definetraj;
        c2::Point cpos1;
        cpos1.type = c2::PointType::Cart;
        cpos1.cpos.x = pt.x;
        cpos1.cpos.y = pt.y;
        cpos1.cpos.z = pt.z;
        cpos1.cpos.a = pt.a;
        cpos1.cpos.b = pt.b;
        cpos1.cpos.c = pt.c;
        definetraj.AddMovJ(cpos1, speed, acc, c2::ZoneType::Fine, zone);
        arm.StartBrushTeethTrajectoryJoint(definetraj);
    }

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@基于已经修复的轨迹来实现力控调整部分@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    c2::Speed brushspeedrepair{880, 880, 880};
    c2::Acc brushaccrepair{380, 380, 380};
    std::vector<PointData> brushpointsoffsetrepairforce;
    std::vector<PointData> brushpointsoffsetrepairforce1;
    std::vector<PointData> brushpointsoffsetrepairforce2;
    std::vector<PointData> brushpointsoffsetrepairforce3;
    std::vector<PointData> brushpointsoffsetrepairforce4;
    std::vector<PointData> brushpointsoffsetrepairforce5;
    std::vector<PointData> brushpointsoffsetrepairforce6;

    obj->StopCapture();

    int n = 1;
    c2::Speed forcespeeda{880, 880, 880};
    c2::Acc forceacca{380, 380, 380};

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@右侧边@@@@@@@@@@@@@@@@@@@@@@@
    // 旋转到准确位置
    int rotate0 = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientRight.py\"");
    if (rotate0 != 0)
    {
        std::cerr << "Python script TcpclientRight.py execution failed!" << std::endl;
        return -1;
    }

    std::ifstream forceinfilerepair0("D:\\UsmileProject\\hand_eye_calibration\\transformsrepairedforcesideright.txt");

    if (!forceinfilerepair0.is_open())
    {
        std::cerr << "Failed to open transformsrepairedforcesideleft.txt" << std::endl;
        return -1;
    }
    double dxrepairf0, dyrepairf0, dzrepairf0, rxrepairf0, ryrepairf0, rzrepairf0;
    while (forceinfilerepair0 >> dxrepairf0 >> dyrepairf0 >> dzrepairf0 >> rxrepairf0 >> ryrepairf0 >> rzrepairf0)
    {
        brushpointsoffsetrepairforce.push_back({dxrepairf0, dyrepairf0, dzrepairf0, rxrepairf0, ryrepairf0, rzrepairf0});
    }
    forceinfilerepair0.close();

    // #########################跑新的轨迹###############################

    c2::Speed lift_speed{880, 880, 880};
    c2::Acc lift_acc{380, 380, 380};

    for (size_t group = 0; group < 1; ++group)
    {
        size_t start_index = group * 8;
        size_t end_index = start_index + 8;

        if (start_index >= brushpointsoffsetrepairforce.size())
            break;

        // =============================
        // group 内的重复刷牙 n 次
        // =============================
        for (int repeat = 0; repeat < n; ++repeat)
        {
            for (size_t i = start_index; i < end_index && i < brushpointsoffsetrepairforce.size(); ++i)
            {
                auto &offset = brushpointsoffsetrepairforce[i];
                c2::Point cpos;
                cpos.type = c2::PointType::Cart;
                cpos.cpos.x = offset.x;
                cpos.cpos.y = offset.y;
                cpos.cpos.z = offset.z;
                cpos.cpos.a = offset.a;
                cpos.cpos.b = offset.b;
                cpos.cpos.c = offset.c;
                // std::cout << "positive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
                c2::MovJointSegments brushing;
                brushing.AddMovJ(cpos, forcespeeda, forceacca, c2::ZoneType::Fine, zone);
                arm.StartBrushTeethTrajectoryJoint(brushing);
            }

            // for (size_t i = end_index - 1; i >= start_index && i < brushpointsoffsetrepairforce.size(); --i)
            // {
            //     auto &offset = brushpointsoffsetrepairforce[i];
            //     c2::Point cpos;
            //     cpos.type = c2::PointType::Cart;
            //     cpos.cpos.x = offset.x;
            //     cpos.cpos.y = offset.y;
            //     cpos.cpos.z = offset.z;
            //     cpos.cpos.a = offset.a;
            //     cpos.cpos.b = offset.b;
            //     cpos.cpos.c = offset.c;
            //     // std::cout << "negtive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
            //     c2::MovJointSegments brushing;
            //     brushing.AddMovJ(cpos, forcespeeda, forceacca, c2::ZoneType::Fine, zone);
            //     arm.StartBrushTeethTrajectoryJoint(brushing);
            // }
        }

        // ====================================================
        // ⭐ 每个 group 完成后：机械臂从最终点 Z 方向上抬 20mm
        // ====================================================
        // 揚起拂刷
        // int python_resultsss = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientRightBrushUp.py\"");
        // std::this_thread::sleep_for(std::chrono::seconds(2));

        // int python_resultssss = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientRight.py\"");
        // std::this_thread::sleep_for(std::chrono::seconds(2));
        // ====================================================
    }

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@右上边@@@@@@@@@@@@@@@@@@@@@@@
    for (const auto &pt : points)
    {
        c2::MovJointSegments definetraj;
        c2::Point cpos1;
        cpos1.type = c2::PointType::Cart;
        cpos1.cpos.x = pt.x;
        cpos1.cpos.y = pt.y;
        cpos1.cpos.z = pt.z;
        cpos1.cpos.a = pt.a;
        cpos1.cpos.b = pt.b;
        cpos1.cpos.c = pt.c;
        definetraj.AddMovJ(cpos1, speed, acc, c2::ZoneType::Fine, zone);
        arm.StartBrushTeethTrajectoryJoint(definetraj);
    }

    int rotate2 = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\Tcpclient.py\"");
    if (rotate2 != 0)
    {
        std::cerr << "Python script TcpclientRight.py execution failed!" << std::endl;
        return -1;
    }

    std::ifstream forceinfilerepair2("D:\\UsmileProject\\hand_eye_calibration\\transformsrepairedforceupright.txt");

    if (!forceinfilerepair2.is_open())
    {
        std::cerr << "Failed to open transformsrepairedforceupright.txt" << std::endl;
        return -1;
    }
    double dxrepairf2, dyrepairf2, dzrepairf2, rxrepairf2, ryrepairf2, rzrepairf2;
    while (forceinfilerepair2 >> dxrepairf2 >> dyrepairf2 >> dzrepairf2 >> rxrepairf2 >> ryrepairf2 >> rzrepairf2)
    {
        brushpointsoffsetrepairforce2.push_back({dxrepairf2, dyrepairf2, dzrepairf2, rxrepairf2, ryrepairf2, rzrepairf2});
    }
    forceinfilerepair2.close();

    // #########################跑新的轨迹#################################
    for (size_t group = 0; group < 1; ++group)
    {
        size_t start_index = group * 8;
        size_t end_index = start_index + 8;

        if (start_index >= brushpointsoffsetrepairforce2.size())
            break;

        for (int repeat = 0; repeat < n; ++repeat)
        {
            for (size_t i = start_index; i < end_index && i < brushpointsoffsetrepairforce2.size(); ++i)
            {
                auto &offset = brushpointsoffsetrepairforce2[i];
                c2::Point cpos;
                cpos.type = c2::PointType::Cart;
                cpos.cpos.x = offset.x;
                cpos.cpos.y = offset.y;
                cpos.cpos.z = offset.z;
                cpos.cpos.a = offset.a;
                cpos.cpos.b = offset.b;
                cpos.cpos.c = offset.c;
                // std::cout << "positive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
                c2::MovJointSegments brushing;
                brushing.AddMovJ(cpos, forcespeeda, forceacca, c2::ZoneType::Fine, zone);
                arm.StartBrushTeethTrajectoryJoint(brushing);
            }

            // for (size_t i = end_index - 1; i >= start_index && i < brushpointsoffsetrepairforce2.size(); --i)
            // {
            //     auto &offset = brushpointsoffsetrepairforce2[i];
            //     c2::Point cpos;
            //     cpos.type = c2::PointType::Cart;
            //     cpos.cpos.x = offset.x;
            //     cpos.cpos.y = offset.y;
            //     cpos.cpos.z = offset.z;
            //     cpos.cpos.a = offset.a;
            //     cpos.cpos.b = offset.b;
            //     cpos.cpos.c = offset.c;
            //     // std::cout << "negtive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
            //     c2::MovJointSegments brushing;
            //     brushing.AddMovJ(cpos, forcespeeda, forceacca, c2::ZoneType::Fine, zone);
            //     arm.StartBrushTeethTrajectoryJoint(brushing);
            // }
        }
    }

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@右内侧@@@@@@@@@@@@@@@@@@@@@@@
    for (const auto &pt : points)
    {
        c2::MovJointSegments definetraj;
        c2::Point cpos1;
        cpos1.type = c2::PointType::Cart;
        cpos1.cpos.x = pt.x;
        cpos1.cpos.y = pt.y;
        cpos1.cpos.z = pt.z;
        cpos1.cpos.a = pt.a;
        cpos1.cpos.b = pt.b;
        cpos1.cpos.c = pt.c;
        definetraj.AddMovJ(cpos1, speed, acc, c2::ZoneType::Fine, zone);
        arm.StartBrushTeethTrajectoryJoint(definetraj);
    }

    int rotate3 = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientLeft.py\"");
    if (rotate3 != 0)
    {
        std::cerr << "Python script TcpclientRight.py execution failed!" << std::endl;
        return -1;
    }

    std::ifstream forceinfilerepair3("D:\\UsmileProject\\hand_eye_calibration\\transformsrepairedforceinsideright.txt");

    if (!forceinfilerepair3.is_open())
    {
        std::cerr << "Failed to open transformsrepairedforceinsideright.txt" << std::endl;
        return -1;
    }
    double dxrepairf3, dyrepairf3, dzrepairf3, rxrepairf3, ryrepairf3, rzrepairf3;
    while (forceinfilerepair3 >> dxrepairf3 >> dyrepairf3 >> dzrepairf3 >> rxrepairf3 >> ryrepairf3 >> rzrepairf3)
    {
        brushpointsoffsetrepairforce3.push_back({dxrepairf3, dyrepairf3, dzrepairf3, rxrepairf3, ryrepairf3, rzrepairf3});
    }
    forceinfilerepair3.close();

    // ###################################跑新的轨迹############################
    for (size_t group = 0; group < 1; ++group)
    {
        size_t start_index = group * 8;
        size_t end_index = start_index + 8;

        if (start_index >= brushpointsoffsetrepairforce3.size())
            break;

        for (int repeat = 0; repeat < n; ++repeat)
        {
            for (size_t i = start_index; i < end_index && i < brushpointsoffsetrepairforce3.size(); ++i)
            {
                auto &offset = brushpointsoffsetrepairforce3[i];
                c2::Point cpos;
                cpos.type = c2::PointType::Cart;
                cpos.cpos.x = offset.x;
                cpos.cpos.y = offset.y;
                cpos.cpos.z = offset.z;
                cpos.cpos.a = offset.a;
                cpos.cpos.b = offset.b;
                cpos.cpos.c = offset.c;
                // std::cout << "positive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
                c2::MovJointSegments brushing;
                brushing.AddMovJ(cpos, forcespeeda, forceacca, c2::ZoneType::Fine, zone);
                arm.StartBrushTeethTrajectoryJoint(brushing);
            }

            // for (size_t i = end_index - 1; i >= start_index && i < brushpointsoffsetrepairforce3.size(); --i)
            // {
            //     auto &offset = brushpointsoffsetrepairforce3[i];
            //     c2::Point cpos;
            //     cpos.type = c2::PointType::Cart;
            //     cpos.cpos.x = offset.x;
            //     cpos.cpos.y = offset.y;
            //     cpos.cpos.z = offset.z;
            //     cpos.cpos.a = offset.a;
            //     cpos.cpos.b = offset.b;
            //     cpos.cpos.c = offset.c;
            //     // std::cout << "negtive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
            //     c2::MovJointSegments brushing;
            //     brushing.AddMovJ(cpos, forcespeeda, forceacca, c2::ZoneType::Fine, zone);
            //     arm.StartBrushTeethTrajectoryJoint(brushing);
            // }
        }

        // 揚起拂刷
        // int python_resultsss = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientLeftBrushUp.py\"");
        // std::this_thread::sleep_for(std::chrono::seconds(2));

        // int python_resultssss = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientLeft.py\"");
        // std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@左侧边@@@@@@@@@@@@@@@@@@@@@@@@@
    for (const auto &pt : points)
    {
        c2::MovJointSegments definetraj;
        c2::Point cpos1;
        cpos1.type = c2::PointType::Cart;
        cpos1.cpos.x = pt.x;
        cpos1.cpos.y = pt.y;
        cpos1.cpos.z = pt.z;
        cpos1.cpos.a = pt.a;
        cpos1.cpos.b = pt.b;
        cpos1.cpos.c = pt.c;
        definetraj.AddMovJ(cpos1, speed, acc, c2::ZoneType::Fine, zone);
        arm.StartBrushTeethTrajectoryJoint(definetraj);
    }
    // 旋转到准确位置
    int rotate4 = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientLeft.py\"");
    if (rotate4 != 0)
    {
        std::cerr << "Python script TcpclientRight.py execution failed!" << std::endl;
        return -1;
    }

    std::ifstream forceinfilerepair4("D:\\UsmileProject\\hand_eye_calibration\\transformsrepairedforcesideleft.txt");

    if (!forceinfilerepair4.is_open())
    {
        std::cerr << "Failed to open transformsrepairedforceinside.txt" << std::endl;
        return -1;
    }
    double dxrepairf4, dyrepairf4, dzrepairf4, rxrepairf4, ryrepairf4, rzrepairf4;
    while (forceinfilerepair4 >> dxrepairf4 >> dyrepairf4 >> dzrepairf4 >> rxrepairf4 >> ryrepairf4 >> rzrepairf4)
    {
        brushpointsoffsetrepairforce4.push_back({dxrepairf4, dyrepairf4, dzrepairf4, rxrepairf4, ryrepairf4, rzrepairf4});
    }
    forceinfilerepair4.close();

    // #########################跑新的轨迹##########################
    for (size_t group = 0; group < 1; ++group)
    {
        size_t start_index = group * 8;
        size_t end_index = start_index + 8;

        if (start_index >= brushpointsoffsetrepairforce4.size())
            break;

        for (int repeat = 0; repeat < n; ++repeat)
        {
            for (size_t i = start_index; i < end_index && i < brushpointsoffsetrepairforce4.size(); ++i)
            {
                auto &offset = brushpointsoffsetrepairforce4[i];
                c2::Point cpos;
                cpos.type = c2::PointType::Cart;
                cpos.cpos.x = offset.x;
                cpos.cpos.y = offset.y;
                cpos.cpos.z = offset.z;
                cpos.cpos.a = offset.a;
                cpos.cpos.b = offset.b;
                cpos.cpos.c = offset.c;
                // std::cout << "positive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
                c2::MovJointSegments brushing;
                brushing.AddMovJ(cpos, forcespeeda, forceacca, c2::ZoneType::Fine, zone);
                arm.StartBrushTeethTrajectoryJoint(brushing);
            }

            // for (size_t i = end_index - 1; i >= start_index && i < brushpointsoffsetrepairforce4.size(); --i)
            // {
            //     auto &offset = brushpointsoffsetrepairforce4[i];
            //     c2::Point cpos;
            //     cpos.type = c2::PointType::Cart;
            //     cpos.cpos.x = offset.x;
            //     cpos.cpos.y = offset.y;
            //     cpos.cpos.z = offset.z;
            //     cpos.cpos.a = offset.a;
            //     cpos.cpos.b = offset.b;
            //     cpos.cpos.c = offset.c;
            //     // std::cout << "negtive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
            //     c2::MovJointSegments brushing;
            //     brushing.AddMovJ(cpos, forcespeeda, forceacca, c2::ZoneType::Fine, zone);
            //     arm.StartBrushTeethTrajectoryJoint(brushing);
            // }
        }

        // 揚起拂刷
        // int python_resultsss = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientLeftBrushUp.py\"");
        // std::this_thread::sleep_for(std::chrono::seconds(2));

        // int python_resultssss = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientLeft.py\"");
        // std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@左上边@@@@@@@@@@@@@@@@@@@@@@@
    for (const auto &pt : points)
    {
        c2::MovJointSegments definetraj;
        c2::Point cpos1;
        cpos1.type = c2::PointType::Cart;
        cpos1.cpos.x = pt.x;
        cpos1.cpos.y = pt.y;
        cpos1.cpos.z = pt.z;
        cpos1.cpos.a = pt.a;
        cpos1.cpos.b = pt.b;
        cpos1.cpos.c = pt.c;
        definetraj.AddMovJ(cpos1, speed, acc, c2::ZoneType::Fine, zone);
        arm.StartBrushTeethTrajectoryJoint(definetraj);
    }
    // 旋转到准确位置
    int rotate1 = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\Tcpclient.py\"");
    if (rotate1 != 0)
    {
        std::cerr << "Python script TcpclientRight.py execution failed!" << std::endl;
        return -1;
    }

    std::ifstream forceinfilerepair1("D:\\UsmileProject\\hand_eye_calibration\\transformsrepairedforceupleft.txt");

    if (!forceinfilerepair1.is_open())
    {
        std::cerr << "Failed to open transformsrepairedforceupleft.txt" << std::endl;
        return -1;
    }
    double dxrepairf1, dyrepairf1, dzrepairf1, rxrepairf1, ryrepairf1, rzrepairf1;
    while (forceinfilerepair1 >> dxrepairf1 >> dyrepairf1 >> dzrepairf1 >> rxrepairf1 >> ryrepairf1 >> rzrepairf1)
    {
        brushpointsoffsetrepairforce1.push_back({dxrepairf1, dyrepairf1, dzrepairf1, rxrepairf1, ryrepairf1, rzrepairf1});
    }
    forceinfilerepair1.close();

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@跑新的轨迹@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    for (size_t group = 0; group < 1; ++group)
    {
        size_t start_index = group * 8;
        size_t end_index = start_index + 8;

        if (start_index >= brushpointsoffsetrepairforce1.size())
            break;

        for (int repeat = 0; repeat < n; ++repeat)
        {
            for (size_t i = start_index; i < end_index && i < brushpointsoffsetrepairforce1.size(); ++i)
            {
                auto &offset = brushpointsoffsetrepairforce1[i];
                c2::Point cpos;
                cpos.type = c2::PointType::Cart;
                cpos.cpos.x = offset.x;
                cpos.cpos.y = offset.y;
                cpos.cpos.z = offset.z;
                cpos.cpos.a = offset.a;
                cpos.cpos.b = offset.b;
                cpos.cpos.c = offset.c;
                // std::cout << "positive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
                c2::MovJointSegments brushing;
                brushing.AddMovJ(cpos, forcespeeda, forceacca, c2::ZoneType::Fine, zone);
                arm.StartBrushTeethTrajectoryJoint(brushing);
            }

            // for (size_t i = end_index - 1; i >= start_index && i < brushpointsoffsetrepairforce1.size(); --i)
            // {
            //     auto &offset = brushpointsoffsetrepairforce1[i];
            //     c2::Point cpos;
            //     cpos.type = c2::PointType::Cart;
            //     cpos.cpos.x = offset.x;
            //     cpos.cpos.y = offset.y;
            //     cpos.cpos.z = offset.z;
            //     cpos.cpos.a = offset.a;
            //     cpos.cpos.b = offset.b;
            //     cpos.cpos.c = offset.c;
            //     // std::cout << "negtive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
            //     c2::MovJointSegments brushing;
            //     brushing.AddMovJ(cpos, forcespeeda, forceacca, c2::ZoneType::Fine, zone);
            //     arm.StartBrushTeethTrajectoryJoint(brushing);
            // }
        }
    }

    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@左内侧@@@@@@@@@@@@@@@@@@@@@@@
    for (const auto &pt : points)
    {
        c2::MovJointSegments definetraj;
        c2::Point cpos1;
        cpos1.type = c2::PointType::Cart;
        cpos1.cpos.x = pt.x;
        cpos1.cpos.y = pt.y;
        cpos1.cpos.z = pt.z;
        cpos1.cpos.a = pt.a;
        cpos1.cpos.b = pt.b;
        cpos1.cpos.c = pt.c;
        definetraj.AddMovJ(cpos1, speed, acc, c2::ZoneType::Fine, zone);
        arm.StartBrushTeethTrajectoryJoint(definetraj);
    }
    // 旋转到准确位置
    int rotate5 = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientRight.py\"");
    if (rotate5 != 0)
    {
        std::cerr << "Python script TcpclientRight.py execution failed!" << std::endl;
        return -1;
    }

    std::ifstream forceinfilerepair5("D:\\UsmileProject\\hand_eye_calibration\\transformsrepairedforceinsideleft.txt");

    if (!forceinfilerepair5.is_open())
    {
        std::cerr << "Failed to open transformsrepairedforceinsideleft.txt" << std::endl;
        return -1;
    }
    double dxrepairf5, dyrepairf5, dzrepairf5, rxrepairf5, ryrepairf5, rzrepairf5;
    while (forceinfilerepair5 >> dxrepairf5 >> dyrepairf5 >> dzrepairf5 >> rxrepairf5 >> ryrepairf5 >> rzrepairf5)
    {
        brushpointsoffsetrepairforce5.push_back({dxrepairf5, dyrepairf5, dzrepairf5, rxrepairf5, ryrepairf5, rzrepairf5});
    }
    forceinfilerepair5.close();

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@跑新的轨迹@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    for (size_t group = 0; group < 1; ++group)
    {
        size_t start_index = group * 8;
        size_t end_index = start_index + 8;

        if (start_index >= brushpointsoffsetrepairforce5.size())
            break;

        for (int repeat = 0; repeat < n; ++repeat)
        {
            for (size_t i = start_index; i < end_index && i < brushpointsoffsetrepairforce5.size(); ++i)
            {
                auto &offset = brushpointsoffsetrepairforce5[i];
                c2::Point cpos;
                cpos.type = c2::PointType::Cart;
                cpos.cpos.x = offset.x;
                cpos.cpos.y = offset.y;
                cpos.cpos.z = offset.z;
                cpos.cpos.a = offset.a;
                cpos.cpos.b = offset.b;
                cpos.cpos.c = offset.c;
                // std::cout << "positive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
                c2::MovJointSegments brushing;
                brushing.AddMovJ(cpos, forcespeeda, forceacca, c2::ZoneType::Fine, zone);
                arm.StartBrushTeethTrajectoryJoint(brushing);
            }

            // for (size_t i = end_index - 1; i >= start_index && i < brushpointsoffsetrepairforce5.size(); --i)
            // {
            //     auto &offset = brushpointsoffsetrepairforce5[i];
            //     c2::Point cpos;
            //     cpos.type = c2::PointType::Cart;
            //     cpos.cpos.x = offset.x;
            //     cpos.cpos.y = offset.y;
            //     cpos.cpos.z = offset.z;
            //     cpos.cpos.a = offset.a;
            //     cpos.cpos.b = offset.b;
            //     cpos.cpos.c = offset.c;
            //     // std::cout << "negtive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
            //     c2::MovJointSegments brushing;
            //     brushing.AddMovJ(cpos, forcespeeda, forceacca, c2::ZoneType::Fine, zone);
            //     arm.StartBrushTeethTrajectoryJoint(brushing);
            // }
        }
        // 揚起拂刷
        // int python_resultsss = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientRightBrushUp.py\"");
        // std::this_thread::sleep_for(std::chrono::seconds(2));

        // int python_resultssss = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientRight.py\"");
        // std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    // // //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@中心区@@@@@@@@@@@@@@@@@@@@@@@
    // std::vector<PointData> pointsnew;
    // pointsnew = {
    //     // 牙齿和牙刷拍摄位置
    //     {25.411, 452.19, 422.686, -179.534, -0.215, 90.558},
    // };

    // // // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@跑到起始位置@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    // for (const auto &pt : pointsnew)
    // {
    //     c2::MovJointSegments definetraj;
    //     c2::Point cpos1;
    //     cpos1.type = c2::PointType::Cart;
    //     cpos1.cpos.x = pt.x;
    //     cpos1.cpos.y = pt.y;
    //     cpos1.cpos.z = pt.z;
    //     cpos1.cpos.a = pt.a;
    //     cpos1.cpos.b = pt.b;
    //     cpos1.cpos.c = pt.c;
    //     definetraj.AddMovJ(cpos1, speed, acc, c2::ZoneType::Fine, zone);
    //     arm.StartBrushTeethTrajectoryJoint(definetraj);
    // }

    // int rotatel = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\TcpclientLeft.py\"");
    // if (rotatel != 0)
    // {
    //     std::cerr << "Python script TcpclientLeft.py execution failed!" << std::endl;
    //     return -1;
    // }

    // //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@加载修复好固定力控的数据@@@@@@@@@@@@@@@@@@@@@@@2
    // std::ifstream forceinfilerepair6("D:\\UsmileProject\\hand_eye_calibration\\transformsrepairedforceCenter.txt");

    // if (!forceinfilerepair6.is_open())
    // {
    //     std::cerr << "Failed to open transformsrepairedforceinsideright.txt" << std::endl;
    //     return -1;
    // }
    // double dxrepairf6, dyrepairf6, dzrepairf6, rxrepairf6, ryrepairf6, rzrepairf6;
    // while (forceinfilerepair6 >> dxrepairf6 >> dyrepairf6 >> dzrepairf6 >> rxrepairf6 >> ryrepairf6 >> rzrepairf6)
    // {
    //     brushpointsoffsetrepairforce6.push_back({dxrepairf6, dyrepairf6, dzrepairf6, rxrepairf6, ryrepairf6, rzrepairf6});
    // }
    // forceinfilerepair6.close();

    // // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@跑新的轨迹@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    // int num = 3;
    // c2::Speed forcespeedas{380, 380, 380};
    // c2::Acc forceaccas{140, 140, 140};

    // for (size_t group = 0; group < 4; ++group)
    // {
    //     size_t start_index = group * 2;
    //     size_t end_index = start_index + 2;

    //     if (start_index >= brushpointsoffsetrepairforce6.size())
    //         break;

    //     for (int repeat = 0; repeat < num; ++repeat)
    //     {
    //         for (size_t i = start_index; i < end_index && i < brushpointsoffsetrepairforce6.size(); ++i)
    //         {
    //             auto &offset = brushpointsoffsetrepairforce6[i];
    //             c2::Point cpos;
    //             cpos.type = c2::PointType::Cart;
    //             cpos.cpos.x = offset.x;
    //             cpos.cpos.y = offset.y;
    //             cpos.cpos.z = offset.z;
    //             cpos.cpos.a = offset.a;
    //             cpos.cpos.b = offset.b;
    //             cpos.cpos.c = offset.c;
    //             // std::cout << "positive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
    //             c2::MovJointSegments brushing;
    //             brushing.AddMovJ(cpos, forcespeedas, forceaccas, c2::ZoneType::Fine, zone);
    //             arm.StartBrushTeethTrajectoryJoint(brushing);
    //         }

    //         for (size_t i = end_index - 1; i >= start_index && i < brushpointsoffsetrepairforce6.size(); --i)
    //         {
    //             auto &offset = brushpointsoffsetrepairforce6[i];
    //             c2::Point cpos;
    //             cpos.type = c2::PointType::Cart;
    //             cpos.cpos.x = offset.x;
    //             cpos.cpos.y = offset.y;
    //             cpos.cpos.z = offset.z;
    //             cpos.cpos.a = offset.a;
    //             cpos.cpos.b = offset.b;
    //             cpos.cpos.c = offset.c;
    //             // std::cout << "negtive " << group + 1 << " th" << i - start_index + 1 << "points" << std::endl;
    //             c2::MovJointSegments brushing;
    //             brushing.AddMovJ(cpos, forcespeedas, forceaccas, c2::ZoneType::Fine, zone);
    //             arm.StartBrushTeethTrajectoryJoint(brushing);
    //         }
    //     }
    // }




    // // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@跑完回归原始位置@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    std::vector<PointData> pointsnewly;
    pointsnewly = {
        {25.411, 452.19, 422.686, -179.534, -0.215, 90.558},
    };
    for (const auto &pt : pointsnewly)
    {
        c2::MovJointSegments definetraj;
        c2::Point cpos1;
        cpos1.type = c2::PointType::Cart;
        cpos1.cpos.x = pt.x;
        cpos1.cpos.y = pt.y;
        cpos1.cpos.z = pt.z;
        cpos1.cpos.a = pt.a;
        cpos1.cpos.b = pt.b;
        cpos1.cpos.c = pt.c;
        definetraj.AddMovJ(cpos1, speed, acc, c2::ZoneType::Fine, zone);
        arm.StartBrushTeethTrajectoryJoint(definetraj);
    }
    int rotatelast = std::system("cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\UsmileProject\\hand_eye_calibration\\Tcpclient.py\"");
    if (rotatelast != 0)
    {
        std::cerr << "Python script Tcpclient.py execution failed!" << std::endl;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    camera.~CameraCapture();

    std::cout << "changeRobotMode: " << arm.changeRobotMode(c2::UserCommand::ToReady) << std::endl;

    return 0;
}
