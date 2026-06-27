#include <cmath>
#include <conio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "DobotTcpDemo.h"
#include <windows.h>
#include "nlohmann/json.hpp"

using json = nlohmann::json;
namespace fs = std::filesystem;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Vector3
{
    double x, y, z;
};

Vector3 transformVectorAToB(Vector3 vA, double rx, double ry, double rz)
{
    double ax = rx * M_PI / 180.0;
    double ay = ry * M_PI / 180.0;
    double az = rz * M_PI / 180.0;

    double cx = cos(ax), sx = sin(ax);
    double cy = cos(ay), sy = sin(ay);
    double cz = cos(az), sz = sin(az);

    double r11 = cy * cz;
    double r12 = cz * sx * sy - cx * sz;
    double r13 = sx * sz + cx * cz * sy;

    double r21 = cy * sz;
    double r22 = cx * cz + sx * sy * sz;
    double r23 = cx * sy * sz - cz * sx;

    double r31 = -sy;
    double r32 = cy * sx;
    double r33 = cx * cy;

    Vector3 vB;
    vB.x = r11 * vA.x + r21 * vA.y + r31 * vA.z;
    vB.y = r12 * vA.x + r22 * vA.y + r32 * vA.z;
    vB.z = r13 * vA.x + r23 * vA.y + r33 * vA.z;

    std::cout << vB.x << " ," << vB.y << " ," << vB.z << " ," << std::endl;

    return vB;
}

double xA = 0, yA = 0, zA = 0, rxA = 0, ryA = 0, rzA = 0;
double xB = 0, yB = 0, zB = 0, rxB = 0, ryB = 0, rzB = 0; // B绕Z旋转90度
double dxm = 0, dym = 0, dzm = 0, drxm = 0, drym = 0, drzm = 0;

const double PI = 3.14159265358979323846;
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;

// 将欧拉角（度）转换为旋转矩阵
Eigen::Matrix3d eulerToRotationMatrix(double rx_deg, double ry_deg, double rz_deg)
{
    // 转换为弧度
    double rx = rx_deg * DEG_TO_RAD;
    double ry = ry_deg * DEG_TO_RAD;
    double rz = rz_deg * DEG_TO_RAD;

    Eigen::Matrix3d R_x, R_y, R_z;

    R_x << 1, 0, 0,
        0, cos(rx), -sin(rx),
        0, sin(rx), cos(rx);

    R_y << cos(ry), 0, sin(ry),
        0, 1, 0,
        -sin(ry), 0, cos(ry);

    R_z << cos(rz), -sin(rz), 0,
        sin(rz), cos(rz), 0,
        0, 0, 1;

    return R_z * R_y * R_x; // ZYX顺序
}

// 将旋转矩阵转换为欧拉角（度）
void rotationMatrixToEuler(const Eigen::Matrix3d &R, double &rx_deg, double &ry_deg, double &rz_deg)
{
    double rx, ry, rz; // 弧度

    ry = atan2(-R(2, 0), sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));

    if (fabs(cos(ry)) > 1e-6)
    {
        rx = atan2(R(2, 1) / cos(ry), R(2, 2) / cos(ry));
        rz = atan2(R(1, 0) / cos(ry), R(0, 0) / cos(ry));
    }
    else
    {
        // 万向锁情况
        rz = 0;
        if (ry > 0)
        {
            rx = atan2(R(0, 1), R(1, 1));
        }
        else
        {
            rx = atan2(-R(0, 1), -R(1, 1));
        }
    }

    // 转换为度
    rx_deg = rx * RAD_TO_DEG;
    ry_deg = ry * RAD_TO_DEG;
    rz_deg = rz * RAD_TO_DEG;
}

// 计算B相对于A的变换（角度单位为度）
// 输入：A和B在世界坐标系（机械臂基坐标系）下的姿态（位置单位任意，角度单位为度）
// 输出：B相对于A的平移和旋转（角度输出为度）
void computeRelativeTransform(double xA, double yA, double zA, double rxA_deg, double ryA_deg, double rzA_deg,
                              double xB, double yB, double zB, double rxB_deg, double ryB_deg, double rzB_deg,
                              double &dx, double &dy, double &dz,
                              double &drx_deg, double &dry_deg, double &drz_deg)
{
    // 构造A和B的变换矩阵（内部自动转换角度）
    Eigen::Matrix3d R_A = eulerToRotationMatrix(rxA_deg, ryA_deg, rzA_deg);
    Eigen::Matrix3d R_B = eulerToRotationMatrix(rxB_deg, ryB_deg, rzB_deg);

    Eigen::Vector3d t_A(xA, yA, zA);
    Eigen::Vector3d t_B(xB, yB, zB);

    // 计算从A到B的相对变换
    // T_A_B = inv(T_A) * T_B
    Eigen::Matrix3d R_rel = R_A.transpose() * R_B;
    Eigen::Vector3d t_rel = R_A.transpose() * (t_B - t_A);

    // 输出相对平移
    dx = t_rel(0);
    dy = t_rel(1);
    dz = t_rel(2);

    // 将相对旋转矩阵转换为欧拉角（度）
    rotationMatrixToEuler(R_rel, drx_deg, dry_deg, drz_deg);
}

// ================= 辅助函数：角度转换与旋转矩阵 =================
inline double degToRad(double degrees)
{
    return degrees * M_PI / 180.0;
}

Eigen::Matrix3d eulerDegToRotationMatrix(double rx_deg, double ry_deg, double rz_deg)
{
    double rx = degToRad(rx_deg);
    double ry = degToRad(ry_deg);
    double rz = degToRad(rz_deg);
    Eigen::AngleAxisd rollAngle(rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rz, Eigen::Vector3d::UnitZ());
    return (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
}

// ================= 1. 路径映射规则函数 =================
std::string getTargetFileName(const std::string &folderName)
{
    if (folderName == "center")
        return "center.txt";
    if (folderName == "sideahead")
        return "sideahead.txt";
    std::string direction = "";
    std::string position = "";
    if (folderName.find("left") != std::string::npos)
        direction = "left";
    else if (folderName.find("right") != std::string::npos)
        direction = "right";
    if (folderName.find("inside") != std::string::npos)
        position = "inside";
    else if (folderName.find("side") != std::string::npos)
        position = "side";
    else if (folderName.find("up") != std::string::npos)
        position = "up";
    if (!direction.empty() && !position.empty())
    {
        return position + direction + ".txt";
    }
    return folderName + ".txt";
}

// ================= 2. 键盘微调逻辑 (严格匹配上传文件向量) =================
Eigen::Vector3d getManualOffset(DobotTcpDemo *demo, double refX, double refY, double refZ)
{
    double gx, gy, gz, grx, gry, grz;
    while (!demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    Dobot::CDescartesPoint curPose{gx, gy, gz, grx, gry, grz};
    Eigen::Vector3d totalOffset(0, 0, 0);

    // Eigen::Matrix3d rotationMatrixs = eulerDegToRotationMatrix(curPose.rx, curPose.ry, curPose.rz);
    // Eigen::Vector3d brushDirs = rotationMatrixs.col(2);
    // brushDirs.normalize();

    std::cout << "\n===== 键盘微调模式 (计算轨迹偏移) =====\n"
              << "W/S : 沿 Y 偏移 | A/D : 沿 X 偏移 | Q/E : 沿 Z 偏移\n"
              << "Z/X : 沿牙刷方向 (BrushDir) 进退\n"
              << "Enter : 结束微调并应用偏移\n";

    while (true)
    {
        if (_kbhit())
        {
            char key = _getch();
            double dx = 0, dy = 0, dz = 0;

            if (key == 'w')
            {
                dx = -0.017294;
                dy = -0.016502;
                dz = -0.999714;
            }
            else if (key == 's')
            {
                dx = 0.017294;
                dy = 0.016502;
                dz = 0.999714;
            }
            else if (key == 'a')
            {
                dx = 0.827884;
                dy = 0.560404;
                dz = -0.023572;
            }
            else if (key == 'd')
            {
                dx = -0.827884;
                dy = -0.560404;
                dz = 0.023572;
            }
            else if (key == 'q')
            {
                dx = 0.560633;
                dy = -0.828055;
                dz = 0.003970;
            }
            else if (key == 'e')
            {
                dx = -0.560633;
                dy = 0.828055;
                dz = -0.003970;
            }
            // else if (key == 'z') { dx = brushDirs.x(); dy = brushDirs.y(); dz = brushDirs.z(); }
            // else if (key == 'x') { dx = -brushDirs.x(); dy = -brushDirs.y(); dz = -brushDirs.z(); }
            else if (key == 13)
            {
                std::cout << "\n微调结束\n";
                break;
            }
            else
                continue;

            curPose.x += dx;
            curPose.y += dy;
            curPose.z += dz;
            demo->moveRobotC(curPose, curPose);

            // 实时打印当前相对基准点的偏移
            Eigen::Vector3d currentRelativeOffset(curPose.x - refX, curPose.y - refY, curPose.z - refZ);
            std::cout << "\r当前累计轨迹偏移 [mm]: " << currentRelativeOffset.transpose() << "    " << std::flush;
        }
        Sleep(10);
    }

    // 返回最终计算出的偏移量
    return Eigen::Vector3d(curPose.x - refX, curPose.y - refY, curPose.z - refZ);
}

// ================= 3. 核心轨迹执行函数 =================
void processFolderTrajectory(DobotTcpDemo *demo, const std::string &folderPath, const std::string &folderName, Dobot::MovSParams params, Eigen::Vector3d offset, bool rebrushvalue)
{
    std::string fileName = getTargetFileName(folderName);
    std::string trajectoryPath = folderPath + "\\" + fileName;
    std::string indexPath = folderPath + "\\all_segments.txt";
    std::string indexFilePath2 = folderPath + "\\support_points.txt";

    int brushvalue = 0; // 默认值
    std::cout << folderName << std::endl;

    std::vector<Dobot::CDescartesPoint> allPoints;
    std::ifstream trajFile(trajectoryPath);
    if (!trajFile.is_open())
        return;
    double x, y, z, rx, ry, rz;

    // 对所有点进行补偿
    while (trajFile >> x >> y >> z >> rx >> ry >> rz)
    {
        allPoints.push_back({x +offset.x(), y + offset.y(), z +offset.z(), rx, ry, rz});
    }
    trajFile.close();

    if (folderName == "rightside")
    {
        brushvalue = 1;
        Dobot::CDescartesPoint rotatetooljoint{};
        rotatetooljoint.x = 0.1;
        rotatetooljoint.y = 0;
        rotatetooljoint.z = 0;
        rotatetooljoint.rx = 0;
        rotatetooljoint.ry = -45;
        rotatetooljoint.rz = 0;

        demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (folderName == "leftside")
    {
        brushvalue = 2;
        Dobot::CDescartesPoint rotatetooljoint{};
        rotatetooljoint.x = 0.1;
        rotatetooljoint.y = 0;
        rotatetooljoint.z = 0;
        rotatetooljoint.rx = 0;
        rotatetooljoint.ry = 45;
        rotatetooljoint.rz = 0;

        demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (folderName == "rightinside")
    {
        brushvalue = 3;
        Dobot::CDescartesPoint rotatetooljoint{};
        rotatetooljoint.x = 0.1;
        rotatetooljoint.y = 0;
        rotatetooljoint.z = 0;
        rotatetooljoint.rx = 0;
        rotatetooljoint.ry = 45;
        rotatetooljoint.rz = 0;

        demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (folderName == "leftinside")
    {
        brushvalue = 4;
        Dobot::CDescartesPoint rotatetooljoint{};
        rotatetooljoint.x = 0.1;
        rotatetooljoint.y = 0;
        rotatetooljoint.z = 0;
        rotatetooljoint.rx = 0;
        rotatetooljoint.ry = -45;
        rotatetooljoint.rz = 0;

        demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (folderName == "sideahead")
    {
        brushvalue = 6;
        Dobot::CDescartesPoint rotatetooljoint{};
        rotatetooljoint.x = 0.1;
        rotatetooljoint.y = 0;
        rotatetooljoint.z = 0;
        rotatetooljoint.rx = 0;
        rotatetooljoint.ry = -45;
        rotatetooljoint.rz = 0;

        demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 後面複製過來吧（先走前面的軌跡）
        std::string indexFilePath3 = "../defaultconfig/rightside/ee_poses.txt";
        std::ifstream picked_path_infile(indexFilePath3);
        std::vector<Dobot::CDescartesPoint> pickedPathPoints;
        if (!picked_path_infile.is_open())
        {
            std::cerr << "无法打开前牙引导轨迹文件: " << indexFilePath3 << std::endl;
        }
        double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
        while (picked_path_infile >> dx >> dy >> dz >> rx >> ry >> rz)
        {
            Dobot::CDescartesPoint cp{};
            cp.x = dx;
            cp.y = dy;
            cp.z = dz + 32;
            cp.rx = rx;
            cp.ry = ry;
            cp.rz = rz;
            pickedPathPoints.push_back(cp);
        }

        picked_path_infile.close();
        if (pickedPathPoints.empty())
        {
            std::cerr << "前牙引导轨迹为空: " << indexFilePath3 << std::endl;
        }

        auto runPickedPathToFrontTeeth = [&](const std::string &stageTag)
        {
            Dobot::MovSParams pickedParams;
            pickedParams.tool = 0;
            pickedParams.user = 0;
            pickedParams.v = 80;
            pickedParams.a = 80;
            pickedParams.freq = 0.2;

            std::cout << "[" << stageTag << "] 先到前牙引导轨迹起点..." << std::endl;
            demo->moveRobotC(pickedPathPoints.front(), pickedPathPoints.front());
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            std::cout << "[" << stageTag << "] 执行前牙引导轨迹..." << std::endl;
            demo->movsDemoC(pickedPathPoints, pickedParams);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        };

        runPickedPathToFrontTeeth("往复刷牙前");
        std::this_thread::sleep_for(std::chrono::seconds(1));

        Dobot::CDescartesPoint firstPosesk{};
        firstPosesk.x = allPoints[0].x;
        firstPosesk.y = allPoints[0].y;
        firstPosesk.z = allPoints[0].z;
        firstPosesk.rx = allPoints[0].rx;
        firstPosesk.ry = allPoints[0].ry;
        firstPosesk.rz = allPoints[0].rz;
        Eigen::Matrix3d rotationMatrixssk = eulerDegToRotationMatrix(firstPosesk.rx, firstPosesk.ry, firstPosesk.rz);
        Eigen::Vector3d brushDirssk = rotationMatrixssk.col(2);
        brushDirssk.normalize();

        Dobot::CDescartesPoint pointstartsk{};
        pointstartsk.x = firstPosesk.x + -brushDirssk.x() * 68;
        pointstartsk.y = firstPosesk.y + -brushDirssk.y() * 68;
        pointstartsk.z = firstPosesk.z + -brushDirssk.z() * 68;
        pointstartsk.rx = firstPosesk.rx;
        pointstartsk.ry = firstPosesk.ry;
        pointstartsk.rz = firstPosesk.rz;

        xA = pickedPathPoints.back().x;
        yA = pickedPathPoints.back().y;
        zA = pickedPathPoints.back().z;
        rxA = pickedPathPoints.back().rx;
        ryA = pickedPathPoints.back().ry;
        rzA = pickedPathPoints.back().rz;

        xB = pointstartsk.x;
        yB = pointstartsk.y;
        zB = pointstartsk.z;
        rxB = pointstartsk.rx;
        ryB = pointstartsk.ry;
        rzB = pointstartsk.rz;

        computeRelativeTransform(xA, yA, zA, rxA, ryA, rzA,
                                 xB, yB, zB, rxB, ryB, rzB,
                                 dxm, dym, dzm, drxm, drym, drzm);

        Dobot::CDescartesPoint rotatetooljointjump6{};
        rotatetooljointjump6.x = dxm;
        rotatetooljointjump6.y = dym;
        rotatetooljointjump6.z = dzm;
        rotatetooljointjump6.rx = drxm;
        rotatetooljointjump6.ry = drym;
        rotatetooljointjump6.rz = drzm;
        demo->RelMovJDemo(rotatetooljointjump6, 0, 0, 20, 50, 100);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        demo->moveRobotC(pointstartsk, pointstartsk);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if ((folderName == "center"))
    {
        brushvalue = 5;
    }
    std::cout << brushvalue << std::endl;

    std::ifstream indexFile(indexPath);
    if (!indexFile.is_open())
    {
        std::cerr << "无法打开索引文件: " << indexPath << std::endl;
    }

    // 首先读取所有行到vector中，以便知道总行数
    std::vector<Dobot::CDescartesPoint> selectedPointslast;
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(indexFile, line))
    {
        lines.push_back(line);
    }
    indexFile.close();

    int totalLines = lines.size();
    int currentLine = 0;

    for (const auto &line : lines)
    {
        currentLine++;
        bool isLastLine = (currentLine == totalLines);

        std::istringstream iss(line);
        int idx;
        std::vector<Dobot::CDescartesPoint> selectedPoints;

        while (iss >> idx)
        {
            if (idx >= 0 && idx < (int)allPoints.size())
            {
                selectedPoints.push_back(allPoints[idx]);
            }
            else
            {
                std::cerr << "\n第 " << currentLine << " 行索引 " << idx << " 超出范围 (0-"
                          << allPoints.size() - 1 << ")" << std::endl;
            }
        }

        if (!selectedPoints.empty())
        {
            // 安全到达
            if (brushvalue != 5 && brushvalue != 6)
            {

                selectedPoints[0].z += 10;
                demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                Dobot::CDescartesPoint rotatetooljointjumps{};
                rotatetooljointjumps.x = 0;
                rotatetooljointjumps.y = 0;
                rotatetooljointjumps.z = -10;
                rotatetooljointjumps.rx = 0;
                rotatetooljointjumps.ry = 0;
                rotatetooljointjumps.rz = 0;
                demo->RelMovJDemo(rotatetooljointjumps, 0, 5, 20, 50, 100);

                selectedPoints[0].z -= 10;
                demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                demo->movsDemoC(selectedPoints, params);
                std::this_thread::sleep_for(std::chrono::seconds(2));

                // 根据是否是最后一行选择不同的动作
                if (!isLastLine)
                {
                    // 非最后一行：执行原来的向后移动
                    // std::cout << "hello 1 " << std::endl;
                    Dobot::CDescartesPoint rotatetooljointjumpss{};
                    rotatetooljointjumpss.x = -15;
                    rotatetooljointjumpss.y = 0;
                    rotatetooljointjumpss.z = 0;
                    rotatetooljointjumpss.rx = 0;
                    rotatetooljointjumpss.ry = 0;
                    rotatetooljointjumpss.rz = 0;
                    demo->RelMovJDemo(rotatetooljointjumpss, 0, 5, 20, 50, 100);
                }
                else
                {
                    // std::cout << "hello " << std::endl;
                    Dobot::CDescartesPoint rotatetooljointleave{};
                    rotatetooljointleave.x = 0;
                    rotatetooljointleave.y = 20;
                    rotatetooljointleave.z = 0;
                    rotatetooljointleave.rx = 0;
                    rotatetooljointleave.ry = 0;
                    rotatetooljointleave.rz = 0;
                    demo->RelMovJDemo(rotatetooljointleave, 0, 5, 20, 50, 100);
                }
            }
            else if (brushvalue == 5)
            {
                // std::cout << "hello center" << std::endl;
                for (const auto &point : selectedPoints)
                {
                    demo->moveRobotC(point, point);

                    Dobot::CDescartesPoint rotatetooljointjumpss{};
                    rotatetooljointjumpss.x = 0;
                    rotatetooljointjumpss.y = -5;
                    rotatetooljointjumpss.z = 0;
                    rotatetooljointjumpss.rx = 0;
                    rotatetooljointjumpss.ry = 0;
                    rotatetooljointjumpss.rz = 0;
                    demo->RelMovJDemo(rotatetooljointjumpss, 0, 5, 20, 50, 100);
                    rotatetooljointjumpss.y = 10;
                    demo->RelMovJDemo(rotatetooljointjumpss, 0, 5, 20, 50, 100);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
            else
            {

                // 開始刷牙
                selectedPoints[0].z += 50;
                demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                Dobot::CDescartesPoint rotatetooljointjumps{};
                rotatetooljointjumps.x = 30;
                rotatetooljointjumps.y = 0;
                rotatetooljointjumps.z = -30;
                rotatetooljointjumps.rx = 0;
                rotatetooljointjumps.ry = 0;
                rotatetooljointjumps.rz = 0;
                demo->RelMovJDemo(rotatetooljointjumps, 0, 5, 20, 50, 100);

                selectedPoints[0].z -= 50;
                demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                Dobot::MovSParams params1;
                params1.tool = 0;
                params1.user = 0;
                params1.v = 80;
                params1.a = 80;
                params1.freq = 0.2;

                demo->movsDemoC(selectedPoints, params1);
                std::this_thread::sleep_for(std::chrono::seconds(2));

                // 根据是否是最后一行选择不同的动作
                if (!isLastLine)
                {
                    // 非最后一行：执行原来的向后移动
                    Dobot::CDescartesPoint rotatetooljointjumpss{};
                    rotatetooljointjumpss.x = -15;
                    rotatetooljointjumpss.y = 0;
                    rotatetooljointjumpss.z = 0;
                    rotatetooljointjumpss.rx = 0;
                    rotatetooljointjumpss.ry = 0;
                    rotatetooljointjumpss.rz = 0;
                    demo->RelMovJDemo(rotatetooljointjumpss, 0, 5, 20, 50, 100);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                else
                {

                    Dobot::CDescartesPoint rotatetooljointleave{};
                    rotatetooljointleave.x = 0;
                    rotatetooljointleave.y = 20;
                    rotatetooljointleave.z = 0;
                    rotatetooljointleave.rx = 0;
                    rotatetooljointleave.ry = 0;
                    rotatetooljointleave.rz = 0;
                    demo->RelMovJDemo(rotatetooljointleave, 0, 5, 20, 50, 100);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }

        selectedPointslast = selectedPoints;
    }

    if (brushvalue != 6)
    {
        Dobot::CDescartesPoint pointend{};
        pointend.x = selectedPointslast.back().x;
        pointend.y = selectedPointslast.back().y;
        pointend.z = selectedPointslast.back().z + 60;
        pointend.rx = selectedPointslast.back().rx;
        pointend.ry = selectedPointslast.back().ry;
        pointend.rz = selectedPointslast.back().rz;
        demo->moveRobotC(pointend, pointend);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else
    {
        Dobot::CDescartesPoint rotatetooljointup{};
        rotatetooljointup.x = 0;
        rotatetooljointup.y = 0;
        rotatetooljointup.z = -18;
        rotatetooljointup.rx = 0;
        rotatetooljointup.ry = 0;
        rotatetooljointup.rz = 0;

        demo->RelMovJDemo(rotatetooljointup, 0, 0, 20, 50, 100);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    Dobot::CDescartesPoint pointsafe{};
    pointsafe.x = 264.8929;
    pointsafe.y = -285.1852;
    pointsafe.z = 491.0669;
    pointsafe.rx = -179.7725;
    pointsafe.ry = -1.3507;
    pointsafe.rz = -145.9055;

    demo->moveRobotC(pointsafe, pointsafe);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (rebrushvalue)
    {
        // 先调整姿态
        if (folderName == "rightside")
        {
            brushvalue = 1;
            Dobot::CDescartesPoint rotatetooljoint{};
            rotatetooljoint.x = 0.1;
            rotatetooljoint.y = 0;
            rotatetooljoint.z = 0;
            rotatetooljoint.rx = 0;
            rotatetooljoint.ry = -45;
            rotatetooljoint.rz = 0;

            demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        else if (folderName == "leftside")
        {
            brushvalue = 2;
            Dobot::CDescartesPoint rotatetooljoint{};
            rotatetooljoint.x = 0.1;
            rotatetooljoint.y = 0;
            rotatetooljoint.z = 0;
            rotatetooljoint.rx = 0;
            rotatetooljoint.ry = 45;
            rotatetooljoint.rz = 0;

            demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        else if (folderName == "rightinside")
        {
            brushvalue = 3;
            Dobot::CDescartesPoint rotatetooljoint{};
            rotatetooljoint.x = 0.1;
            rotatetooljoint.y = 0;
            rotatetooljoint.z = 0;
            rotatetooljoint.rx = 0;
            rotatetooljoint.ry = 45;
            rotatetooljoint.rz = 0;

            demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        else if (folderName == "leftinside")
        {
            brushvalue = 4;
            Dobot::CDescartesPoint rotatetooljoint{};
            rotatetooljoint.x = 0.1;
            rotatetooljoint.y = 0;
            rotatetooljoint.z = 0;
            rotatetooljoint.rx = 0;
            rotatetooljoint.ry = -45;
            rotatetooljoint.rz = 0;

            demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        else if (folderName == "sideahead")
        {
            brushvalue = 6;
            Dobot::CDescartesPoint rotatetooljoint{};
            rotatetooljoint.x = 0.1;
            rotatetooljoint.y = 0;
            rotatetooljoint.z = 0;
            rotatetooljoint.rx = 0;
            rotatetooljoint.ry = -45;
            rotatetooljoint.rz = 0;

            demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

            std::this_thread::sleep_for(std::chrono::seconds(1));

            // 後面複製過來吧（先走前面的軌跡）
            std::string indexFilePath3 = "../defaultconfig/rightside/ee_poses.txt";
            std::ifstream picked_path_infile(indexFilePath3);
            std::vector<Dobot::CDescartesPoint> pickedPathPoints;
            if (!picked_path_infile.is_open())
            {
                std::cerr << "无法打开前牙引导轨迹文件: " << indexFilePath3 << std::endl;
            }
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            while (picked_path_infile >> dx >> dy >> dz >> rx >> ry >> rz)
            {
                Dobot::CDescartesPoint cp{};
                cp.x = dx;
                cp.y = dy;
                cp.z = dz + 32;
                cp.rx = rx;
                cp.ry = ry;
                cp.rz = rz;
                pickedPathPoints.push_back(cp);
            }

            picked_path_infile.close();
            if (pickedPathPoints.empty())
            {
                std::cerr << "前牙引导轨迹为空: " << indexFilePath3 << std::endl;
            }

            auto runPickedPathToFrontTeeth = [&](const std::string &stageTag)
            {
                Dobot::MovSParams pickedParams;
                pickedParams.tool = 0;
                pickedParams.user = 0;
                pickedParams.v = 80;
                pickedParams.a = 80;
                pickedParams.freq = 0.2;

                std::cout << "[" << stageTag << "] 先到前牙引导轨迹起点..." << std::endl;
                demo->moveRobotC(pickedPathPoints.front(), pickedPathPoints.front());
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                std::cout << "[" << stageTag << "] 执行前牙引导轨迹..." << std::endl;
                demo->movsDemoC(pickedPathPoints, pickedParams);
                std::this_thread::sleep_for(std::chrono::seconds(2));
            };

            runPickedPathToFrontTeeth("浮刷前");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        std::ifstream indexFiles(indexFilePath2);
        if (!indexFiles.is_open())
        {
            std::cerr << "无法打开点位索引文件: " << indexFilePath2 << std::endl;
        }

        // 1. 预读所有 ID 确保文件有效
        std::vector<int> targetIds;
        std::string line2;
        while (std::getline(indexFiles, line2))
        {
            if (line2.empty())
                continue;
            try
            {
                targetIds.push_back(std::stoi(line2));
            }
            catch (...)
            {
                continue; // 跳过空行或非法字符
            }
        }
        indexFiles.close();

        if (targetIds.empty())
        {
            std::cout << "索引文件为空，无需运动。" << std::endl;
        }

        // 3. 循环处理每一个 ID
        for (size_t i = 0; i < targetIds.size(); ++i)
        {
            int currentTargetId = targetIds[i];

            // 索引合法性检查
            if (currentTargetId < 0 || currentTargetId >= (int)allPoints.size())
            {
                std::cerr << "跳过非法索引: " << currentTargetId << std::endl;
                continue;
            }

            std::cout << "--- 正在处理第 " << i + 1 << " 个任务，目标点 ID: " << currentTargetId << " ---" << std::endl;

            if (i == 0 && brushvalue == 6)
            {
                Dobot::CDescartesPoint firstPosesk{};
                firstPosesk.x = allPoints[currentTargetId].x;
                firstPosesk.y = allPoints[currentTargetId].y;
                firstPosesk.z = allPoints[currentTargetId].z;
                firstPosesk.rx = allPoints[currentTargetId].rx;
                firstPosesk.ry = allPoints[currentTargetId].ry;
                firstPosesk.rz = allPoints[currentTargetId].rz;
                Eigen::Matrix3d rotationMatrixssk = eulerDegToRotationMatrix(firstPosesk.rx, firstPosesk.ry, firstPosesk.rz);
                Eigen::Vector3d brushDirssk = rotationMatrixssk.col(2);
                brushDirssk.normalize();

                Dobot::CDescartesPoint pointstartsk{};
                pointstartsk.x = firstPosesk.x + -brushDirssk.x() * 68;
                pointstartsk.y = firstPosesk.y + -brushDirssk.y() * 68;
                pointstartsk.z = firstPosesk.z + -brushDirssk.z() * 68;
                pointstartsk.rx = firstPosesk.rx;
                pointstartsk.ry = firstPosesk.ry;
                pointstartsk.rz = firstPosesk.rz;

                std::string indexFilePath3 = "../defaultconfig/rightside/ee_poses.txt";
                std::ifstream picked_path_infile(indexFilePath3);
                std::vector<Dobot::CDescartesPoint> pickedPathPoints;
                if (!picked_path_infile.is_open())
                {
                    std::cerr << "无法打开前牙引导轨迹文件: " << indexFilePath3 << std::endl;
                }
                double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
                while (picked_path_infile >> dx >> dy >> dz >> rx >> ry >> rz)
                {
                    Dobot::CDescartesPoint cp{};
                    cp.x = dx;
                    cp.y = dy;
                    cp.z = dz + 32;
                    cp.rx = rx;
                    cp.ry = ry;
                    cp.rz = rz;
                    pickedPathPoints.push_back(cp);
                }

                picked_path_infile.close();

                xA = pickedPathPoints.back().x;
                yA = pickedPathPoints.back().y;
                zA = pickedPathPoints.back().z;
                rxA = pickedPathPoints.back().rx;
                ryA = pickedPathPoints.back().ry;
                rzA = pickedPathPoints.back().rz;

                xB = pointstartsk.x;
                yB = pointstartsk.y;
                zB = pointstartsk.z;
                rxB = pointstartsk.rx;
                ryB = pointstartsk.ry;
                rzB = pointstartsk.rz;

                computeRelativeTransform(xA, yA, zA, rxA, ryA, rzA,
                                         xB, yB, zB, rxB, ryB, rzB,
                                         dxm, dym, dzm, drxm, drym, drzm);

                Dobot::CDescartesPoint rotatetooljointjump6{};
                rotatetooljointjump6.x = dxm;
                rotatetooljointjump6.y = dym;
                rotatetooljointjump6.z = dzm;
                rotatetooljointjump6.rx = drxm;
                rotatetooljointjump6.ry = drym;
                rotatetooljointjump6.rz = drzm;
                demo->RelMovJDemo(rotatetooljointjump6, 0, 0, 20, 50, 100);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                demo->moveRobotC(pointstartsk, pointstartsk);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            allPoints[currentTargetId].z += 50;
            demo->moveRobotC(allPoints[currentTargetId], allPoints[currentTargetId]);
            std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待稳定

            Dobot::CDescartesPoint rotatetooljointjumps{};
            Dobot::CDescartesPoint rotatetooljointjumpss{};
            if (folderName == "rightside")
            {
                rotatetooljointjumps.x = 20;
                rotatetooljointjumps.y = 0;
                rotatetooljointjumps.z = -38;
                rotatetooljointjumps.rx = 0;
                rotatetooljointjumps.ry = 0;
                rotatetooljointjumps.rz = 0;

                rotatetooljointjumpss.x = -35;
                rotatetooljointjumpss.y = 0;
                rotatetooljointjumpss.z = 0;
                rotatetooljointjumpss.rx = 0;
                rotatetooljointjumpss.ry = 0;
                rotatetooljointjumpss.rz = 0;
            }
            else if (folderName == "leftside")
            {
                rotatetooljointjumps.x = -20;
                rotatetooljointjumps.y = 0;
                rotatetooljointjumps.z = -38;
                rotatetooljointjumps.rx = 0;
                rotatetooljointjumps.ry = 0;
                rotatetooljointjumps.rz = 0;

                rotatetooljointjumpss.x = 35;
                rotatetooljointjumpss.y = 0;
                rotatetooljointjumpss.z = 0;
                rotatetooljointjumpss.rx = 0;
                rotatetooljointjumpss.ry = 0;
                rotatetooljointjumpss.rz = 0;
            }
            else if (folderName == "rightinside")
            {
                rotatetooljointjumps.x = -20;
                rotatetooljointjumps.y = 0;
                rotatetooljointjumps.z = -38;
                rotatetooljointjumps.rx = 0;
                rotatetooljointjumps.ry = 0;
                rotatetooljointjumps.rz = 0;

                rotatetooljointjumpss.x = 35;
                rotatetooljointjumpss.y = 0;
                rotatetooljointjumpss.z = 0;
                rotatetooljointjumpss.rx = 0;
                rotatetooljointjumpss.ry = 0;
                rotatetooljointjumpss.rz = 0;
            }
            else if (folderName == "leftinside")
            {
                rotatetooljointjumps.x = 20;
                rotatetooljointjumps.y = 0;
                rotatetooljointjumps.z = -38;
                rotatetooljointjumps.rx = 0;
                rotatetooljointjumps.ry = 0;
                rotatetooljointjumps.rz = 0;

                rotatetooljointjumpss.x = -35;
                rotatetooljointjumpss.y = 0;
                rotatetooljointjumpss.z = 0;
                rotatetooljointjumpss.rx = 0;
                rotatetooljointjumpss.ry = 0;
                rotatetooljointjumpss.rz = 0;
            }
            else if (folderName == "sideahead")
            {
                rotatetooljointjumps.x = 28;
                rotatetooljointjumps.y = 0;
                rotatetooljointjumps.z = -20;
                rotatetooljointjumps.rx = 0;
                rotatetooljointjumps.ry = 0;
                rotatetooljointjumps.rz = 0;

                rotatetooljointjumpss.x = -35;
                rotatetooljointjumpss.y = 0;
                rotatetooljointjumpss.z = 0;
                rotatetooljointjumpss.rx = 0;
                rotatetooljointjumpss.ry = 0;
                rotatetooljointjumpss.rz = 0;
            }

            demo->RelMovJDemo(rotatetooljointjumps, 0, 5, 20, 50, 100);

            allPoints[currentTargetId].z -= 52;
            demo->moveRobotC(allPoints[currentTargetId], allPoints[currentTargetId]);
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // B. 执行您要求的特殊动作 (向 X 负方向移动 15mm)

            std::cout << "到达点 " << currentTargetId << "，执行回退补偿动作..." << std::endl;
            demo->RelMovJDemo(rotatetooljointjumpss, 0, 5, 20, 50, 100);

            // C. 动作间隙停顿
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // 如果读到最后一行，自动结束循环
            if (i == targetIds.size() - 1)
            {
                std::cout << "已完成最后一行索引，流程结束。" << std::endl;
            }
            selectedPointslast.back() = allPoints[currentTargetId];
        }

        if (brushvalue == 6)
        {
            Dobot::CDescartesPoint rotatetooljointup{};
            rotatetooljointup.x = 0;
            rotatetooljointup.y = 0;
            rotatetooljointup.z = -18;
            rotatetooljointup.rx = 0;
            rotatetooljointup.ry = 0;
            rotatetooljointup.rz = 0;

            demo->RelMovJDemo(rotatetooljointup, 0, 0, 20, 50, 100);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        else
        {

            Dobot::CDescartesPoint pointends{};
            pointends.x = selectedPointslast.back().x;
            pointends.y = selectedPointslast.back().y;
            pointends.z = selectedPointslast.back().z + 60;
            pointends.rx = selectedPointslast.back().rx;
            pointends.ry = selectedPointslast.back().ry;
            pointends.rz = selectedPointslast.back().rz;
            demo->moveRobotC(pointends, pointends);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    else
    {
        std::cout << "不需要浮刷。" << std::endl;
    }
}

// ================= 4. 主程序 =================
int main()
{
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    DobotTcpDemo *demo = new DobotTcpDemo();

    double configBrushSpeed = 80.0;
    std::ifstream configFile("D:\\UsmileProject\\hand_eye_calibration\\UsmileUi\\defaultconfig\\config.json");
    if (configFile.is_open())
    {
        try
        {
            json j;
            configFile >> j;
            configBrushSpeed = j.value("brushSpeed", 50.0);
        }
        catch (...)
        {
        }
        configFile.close();
    }

    // 先运动到安全点
    Dobot::CDescartesPoint pointsafe{};
    pointsafe.x = 264.8929;
    pointsafe.y = -285.1852;
    pointsafe.z = 491.0669;
    pointsafe.rx = -179.7725;
    pointsafe.ry = -1.3507;
    pointsafe.rz = -145.9055;
    demo->moveRobotC(pointsafe, pointsafe);
    std::wcout << L"机械臂到达安全点，请确认按Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    // 标定机械臂末端
    double modifiedupx = -6.9142260000002125 + 14.163013999999862;
    double modifiedupy = -2.0350259999999025 - 24.190834000000166;
    double modifiedupz = 101.3919;
    double modifiedup = 0;

    Dobot::CDescartesPoint pointa{};
    pointa.x = 264.89290 + modifiedupx;
    pointa.y = -285.1852 + modifiedupy;
    pointa.z = 391.0669 + modifiedup + modifiedupz;
    pointa.rx = -179.7725;
    pointa.ry = -1.3507;
    pointa.rz = -145.9055;
    demo->moveRobotC(pointa, pointa);
    std::wcout << L"机械臂到达安全点pointa，请确认按Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@微調牙刷@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    const std::string Brush_offset = "../defaultconfig/brushoffsets.json";
    std::cout << "是否進行牙刷微調";
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    // 先挪动然后再微补
    std::ifstream inputFile(Brush_offset);

    Eigen::Vector3d totalOffsetTCP;
    if (inputFile.is_open())
    {
        json loadedJson;
        inputFile >> loadedJson;
        inputFile.close();

        // 讀取偏移量
        double offsetX = loadedJson["brushxoffset"];
        double offsetY = loadedJson["brushyoffset"];
        double offsetZ = loadedJson["brushzoffset"];

        double offsetXs = loadedJson["brushxoffsets"];
        double offsetYs = loadedJson["brushyoffsets"];
        double offsetZs = loadedJson["brushzoffsets"];

        // 这里应该选用右上的第一个点
        std::ifstream trajFiles("../defaultconfig/rightside/rightup.txt");
        if (!trajFiles.is_open())
            return -1;

        double xs, ys, zs, rxs, rys, rzs;

        std::vector<Dobot::CDescartesPoint> allPointss;
        while (trajFiles >> xs >> ys >> zs >> rxs >> rys >> rzs)
        {
            allPointss.push_back({xs, ys, zs, rxs, rys, rzs});
        }
        trajFiles.close();

        Dobot::CDescartesPoint pointa{};
        pointa.x = allPointss[0].x;
        pointa.y = allPointss[0].y;
        pointa.z = allPointss[0].z;
        pointa.rx = allPointss[0].rx;
        pointa.ry = allPointss[0].ry;
        pointa.rz = allPointss[0].rz;
        demo->moveRobotC(pointa, pointa);
        std::cout << "机械臂到达起始点" << std::endl;
        std::cout << "开始牙刷微調。" << std::endl;
        std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

        Eigen::Vector3d totalOffset = getManualOffset(demo, pointa.x, pointa.y, pointa.z);

        // 转移到机械臂末端的误差
        Vector3 vecA = {totalOffset.x(), totalOffset.y(), totalOffset.z()};
        Vector3 vecB = transformVectorAToB(vecA, allPointss[0].rx, allPointss[0].ry, allPointss[0].rz);
        std::cout << "向量在坐标系 B 下的值为：" << std::endl;
        std::cout << "X: " << vecB.x << "\nY: " << vecB.y << "\nZ: " << vecB.z << std::endl;

        double tcpx = vecB.x;
        double tcpy = vecB.y;
        double tcpz = vecB.z;

        totalOffsetTCP ={tcpx,tcpy,tcpz};
     
    }

    // 2. 设置轨迹运行参数
    Dobot::MovSParams params{};
    params.speed = configBrushSpeed;
    params.v = 80;
    params.a = 80;
    params.freq = 0.2;

    std::string rootDir = "D:\\UsmileProject\\hand_eye_calibration\\UsmileUi\\modifytrajectory";
    if (!fs::exists(rootDir))
        return -1;

    std::cout << "\n完成刷牙轨迹，是否需要浮刷（Y/N）? ";
    char choice;
    std::cin >> choice;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
    bool rebrush = false;
    if (choice == 'y' || choice == 'Y')
    {
        rebrush = true;
    }

    // 3. 遍历并执行带偏移的轨迹
    for (const auto &entry : fs::directory_iterator(rootDir))
    {
        // 先运动到安全点
        demo->moveRobotC(pointsafe, pointsafe);

        if (entry.is_directory())
        {

            std::cout << "\n>> " << entry.path().filename().string() << " Enter..." << std::endl;
            processFolderTrajectory(demo, entry.path().string(), entry.path().filename().string(), params, totalOffsetTCP, rebrush);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    demo->moveRobotC(pointsafe, pointsafe);

    delete demo;
    return 0;
}