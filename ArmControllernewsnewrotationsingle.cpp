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
        // p.a, p.b, p.c 是 ZYX 欧拉角，单位为弧度
        double rx = p.a, ry = p.b, rz = p.c;
        // 构造旋转矩阵（ZYX顺序）
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
    ArmController arm;
    CameraCapture camera("169.254.7.168");

    std::cout << "isconnected: " << arm.isconnected() << std::endl;

    std::cout << "changeRobotMode: " << arm.changeRobotMode(c2::UserCommand::ToAuto) << std::endl;


    // 自动抓图轨迹
    c2::Zone zone;
    zone.dis = 0;
    zone.per = 0;

    c2::Speed speed{60, 60, 60};
    c2::Acc acc{20, 20, 20};
    struct PointData
    {
        double x, y, z, a, b, c;
    };

    std::vector<PointData> points;
    points = {
        {25.411, 452.19, 422.686, -179.534, -0.215, 90.558}
    };

    // 使用for循环添加每个点
    int count = 1;
    int imgcount = 1;

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
        
        camera.captureAndSave(imgcount);
    }
    camera.~CameraCapture();
    
    // 用单张点云图和已知模型的配准后轨迹
    std::cout << "Running Python script of ICP for model" << std::endl;
    int python_result = std::system("cmd /c \"D:\\line\\pythonProject\\.venv\\Scripts\\activate && python D:\\hand_eye_calibration\\GeneratePathOffset66init.py\"");
    if (python_result != 0)
    {
        std::cerr << "Python script execution failed!" << std::endl;
        return -1;
    }

    // 走机械臂末端的轨迹
    std::cout << "Running Python script to generate brush offsets..." << std::endl;
    // int python_result1 = std::system("cmd /c \"D:\\line\\pythonProject\\.venv\\Scripts\\activate && python D:\\hand_eye_calibration\\GeneratePathOffset66.py\"");
    int python_result1 = std::system("cmd /c \"D:\\line\\pythonProject\\.venv\\Scripts\\activate && python D:\\hand_eye_calibration\\GeneratePathOffset66Brush.py\"");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    int python_result11 = std::system("cmd /c \"D:\\line\\pythonProject\\.venv\\Scripts\\activate && python D:\\hand_eye_calibration\\TCPRotation10.py\"");

    if (python_result1 != 0)
    {
        std::cerr << "Python script execution failed!" << std::endl;
        return -1;
    }

    c2::Speed brushspeed{6, 6, 6};
    c2::Acc brushacc{3, 3, 3};

    //从拍摄的牙齿和牙刷位置开始作为刷牙起始点
    std::vector<PointData> brushpoints;
    brushpoints = {{25.411, 452.19, 422.686, -179.534, -0.215, 90.558}};


    // // 读取从python生成的轨迹点
    std::vector<PointData> brushpointsoffset;
    std::vector<PointData> brushpointsoffsetrepair;

    std::ifstream infile("D:\\hand_eye_calibration\\saved_ply\\ee_poses.txt");

    if (!infile.is_open())
    {
        std::cerr << "Failed to open brush_offset.txt" << std::endl;
        return -1;
    }

    double dx, dy, dz, rx, ry, rz;
    while (infile >> dx >> dy >> dz >> rx >> ry >> rz)
    {
        std::cout << "Read values: " << dx << " >> " << dy << " >> " << dz << ">> " << rx << ">> " << ry
                  << ">> " << rz << std::endl;
        brushpointsoffset.push_back({dx, dy, dz, rx, ry, rz});
    }

    infile.close();


    // 翻转
    std::vector<decltype(brushpointsoffset)::value_type> brushpointsoffsetinverse;
    for (auto it = brushpointsoffset.rbegin(); it != brushpointsoffset.rend(); ++it)
    {
        brushpointsoffsetinverse.push_back(*it);
    }

    c2::MovJointSegments brushingtrajectory;
    c2::MovJointSegments brushingtrajectoryinverse;

    // 添加起点
    if (!brushpoints.empty())
    {
        c2::Point cposbrush;
        cposbrush.type = c2::PointType::Cart;
        cposbrush.cpos.x = brushpoints[0].x;
        cposbrush.cpos.y = brushpoints[0].y;
        cposbrush.cpos.z = brushpoints[0].z;
        cposbrush.cpos.a = brushpoints[0].a;
        cposbrush.cpos.b = brushpoints[0].b;
        cposbrush.cpos.c = brushpoints[0].c;
        brushingtrajectory.AddMovJ(cposbrush, brushspeed, brushacc, c2::ZoneType::Fine, zone);
    }

    // // 添加后续点（points前三位 + pointsoffset对应值，后三位保持与points相同）（后续需要修改python代码实现）
    if (!brushpoints.empty() && !brushpointsoffset.empty())
    {
        int countvalue = 11;
        for (const auto &offset : brushpointsoffset)
        {
            c2::MovJointSegments brushingtrajectorys;
            c2::Point cpos;
            cpos.type = c2::PointType::Cart;

            cpos.cpos.x = offset.x;
            cpos.cpos.y = offset.y;
            cpos.cpos.z = offset.z;
            cpos.cpos.a = offset.a;
            cpos.cpos.b = offset.b;
            cpos.cpos.c = offset.c;

            brushingtrajectorys.AddMovJ(cpos, brushspeed, brushacc, c2::ZoneType::Fine, zone);
            brushingtrajectory.AddMovJ(cpos, brushspeed, brushacc, c2::ZoneType::Fine, zone);

            arm.StartBrushTeethTrajectoryJoint(brushingtrajectorys);
            camera.captureAndSave(countvalue);
      
            countvalue += 1;
        }
    }

    // 运行修正后的代码
    int python_result111 = std::system("cmd /c \"D:\\line\\pythonProject\\.venv\\Scripts\\activate && python D:\\hand_eye_calibration\\AutoDetectDiff6.py\"");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    
    int python_result116 = std::system("cmd /c \"D:\\line\\pythonProject\\.venv\\Scripts\\activate && python D:\\hand_eye_calibration\\GenerateNewPath.py\"");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::ifstream infilerepair("D:\\hand_eye_calibration\\transformsrepaired.txt");

    if (!infilerepair.is_open())
    {
        std::cerr << "Failed to open brush_offset.txt" << std::endl;
        return -1;
    }


    double dxrepair, dyrepair, dzrepair, rxrepair, ryrepair, rzrepair;
    while (infilerepair >> dxrepair >> dyrepair >> dzrepair >> rxrepair >> ryrepair >> rzrepair)
    {
        std::cout << "Read repair values: " << dxrepair << " >> " << dyrepair << " >> " << dzrepair << ">> " << rxrepair << ">> " << ryrepair
                  << ">> " << rzrepair << std::endl;
        brushpointsoffsetrepair.push_back({dxrepair, dyrepair, dzrepair, rxrepair, ryrepair, rzrepair});
    }

    infilerepair.close();


    c2::Speed brushspeedrepair{80, 80, 80};
    c2::Acc brushaccrepair{40, 40, 40};

    // 执行补偿后的轨迹
    if (!brushpoints.empty() && !brushpointsoffsetrepair.empty())
    {
        for (const auto &offset : brushpointsoffsetrepair)
        {
            c2::MovJointSegments brushingtrajectoryssrepair;
            c2::Point cpos;
            cpos.type = c2::PointType::Cart;

            cpos.cpos.x = offset.x;
            cpos.cpos.y = offset.y;
            cpos.cpos.z = offset.z;
            cpos.cpos.a = offset.a;
            cpos.cpos.b = offset.b;
            cpos.cpos.c = offset.c;

            brushingtrajectoryssrepair.AddMovJ(cpos, brushspeedrepair, brushaccrepair, c2::ZoneType::Fine, zone);
            arm.StartBrushTeethTrajectoryJoint(brushingtrajectoryssrepair);
        }
    }


    std::cout << "changeRobotMode: " << arm.changeRobotMode(c2::UserCommand::ToReady) << std::endl;

    // arm.disconnect();

    return 0;
}
