#include <cmath>
#include <conio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <thread>
#include <chrono>
#include <limits>
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

Dobot::CDescartesPoint makeSideAheadGuideExitPose()
{
    Dobot::CDescartesPoint p{};
    p.x = 58.8848;
    p.y = -393.7958;
    p.z = 528.6063;
    p.rx = -179.6860;
    p.ry = 43.6490;
    p.rz = -160.6180;
    return p;
}

void liftRelativeMmForBrush(DobotTcpDemo *demo, double mm)
{
    Dobot::CDescartesPoint liftRel{};
    liftRel.x = 0;
    liftRel.y = 0;
    liftRel.z = -mm;
    liftRel.rx = 0;
    liftRel.ry = 0;
    liftRel.rz = 0;
    demo->RelMovJDemo(liftRel, 0, 5, 20, 50, 100);
}

void prepareRegionEntry(DobotTcpDemo *demo,
                        Dobot::CDescartesPoint &regionRot,
                        const char *regionName,
                        double liftMm)
{
    std::cout << "[" << regionName << "] 入点：相对上抬 " << liftMm << "mm (tool5)..." << std::endl;
    liftRelativeMmForBrush(demo, liftMm);

    std::cout << "[" << regionName << "] 入点：旋转区域姿态 (tool5)..." << std::endl;
    demo->RelMovJDemo(regionRot, 0, 5, 20, 50, 100);
}

bool applyBrushTcpFromJson(DobotTcpDemo *demo, const std::string &brushOffsetPath)
{
    std::ifstream inputFile(brushOffsetPath);
    if (!inputFile.is_open())
    {
        std::cerr << "无法打开刷头偏移文件: " << brushOffsetPath << std::endl;
        return false;
    }
    json loadedJson;
    inputFile >> loadedJson;
    inputFile.close();

    const double offsetXs = loadedJson.value("brushxoffsets", 0.0);
    const double offsetYs = loadedJson.value("brushyoffsets", 0.0);
    const double offsetZs = loadedJson.value("brushzoffsets", 0.0);

    const double tcpx = -9.748236 - offsetXs;
    const double tcpy = -186.312977 - offsetYs;
    const double tcpz = 223.252632 - offsetZs;
    const std::string tcpvalue = "{" + std::to_string(tcpx) + "," +
                                 std::to_string(tcpy) + "," +
                                 std::to_string(tcpz) + ",0,0,0}";
    demo->setToolDemo(5, tcpvalue);
    return true;
}

void moveToSafeViaWaypointsForBrush(DobotTcpDemo *demo)
{
    Dobot::CDescartesPoint pointpartone{};
    pointpartone.x = 606.9460;
    pointpartone.y = -64.8707;
    pointpartone.z = 426.0307;
    pointpartone.rx = 175.3696;
    pointpartone.ry = 43.8935;
    pointpartone.rz = 109.5293;

    Dobot::CDescartesPoint pointtwo{};
    pointtwo.x = 482.4471;
    pointtwo.y = 32.6242;
    pointtwo.z = 402.5695;
    pointtwo.rx = 179.9249;
    pointtwo.ry = 1.0609;
    pointtwo.rz = -145.5932;

    Dobot::CDescartesPoint pointsafe{};
    pointsafe.x = 264.8929;
    pointsafe.y = -285.1852;
    pointsafe.z = 491.0669;
    pointsafe.rx = -179.7725;
    pointsafe.ry = -1.3507;
    pointsafe.rz = -145.9055;

    demo->moveRobotC(pointpartone, pointpartone);
    demo->moveRobotC(pointtwo, pointtwo);
    demo->moveRobotC(pointsafe, pointsafe);
}

bool loadSideAheadGuidePath(std::vector<Dobot::CDescartesPoint> &out)
{
    const std::string indexFilePath3 = "../defaultconfig/rightside/ee_poses.txt";
    std::ifstream picked_path_infile(indexFilePath3);
    if (!picked_path_infile.is_open())
    {
        std::cerr << "无法打开前牙引导轨迹文件: " << indexFilePath3 << std::endl;
        return false;
    }

    out.clear();
    double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
    while (picked_path_infile >> dx >> dy >> dz >> rx >> ry >> rz)
    {
        Dobot::CDescartesPoint cp{};
        cp.x = dx;
        cp.y = dy;
        cp.z = dz + 30;
        cp.rx = rx;
        cp.ry = ry;
        cp.rz = rz;
        out.push_back(cp);
    }
    picked_path_infile.close();
    if (out.empty())
    {
        std::cerr << "前牙引导轨迹为空: " << indexFilePath3 << std::endl;
        return false;
    }
    return true;
}

Dobot::CDescartesPoint computeBrushApproachStart(const Dobot::CDescartesPoint &firstPose, double backMm)
{
    Eigen::Matrix3d rotationMatrix = eulerDegToRotationMatrix(firstPose.rx, firstPose.ry, firstPose.rz);
    Eigen::Vector3d brushDir = rotationMatrix.col(2);
    brushDir.normalize();

    Dobot::CDescartesPoint pointstart{};
    pointstart.x = firstPose.x + -brushDir.x() * backMm;
    pointstart.y = firstPose.y + -brushDir.y() * backMm;
    pointstart.z = firstPose.z + -brushDir.z() * backMm;
    pointstart.rx = firstPose.rx;
    pointstart.ry = firstPose.ry;
    pointstart.rz = firstPose.rz;
    return pointstart;
}

void executeSideAheadExit(DobotTcpDemo *demo)
{
    Dobot::CDescartesPoint postUp{};
    postUp.x = 0;
    postUp.y = 0;
    postUp.z = -18;
    postUp.rx = 0;
    postUp.ry = 0;
    postUp.rz = 0;
    std::cout << "[SideAhead] 退出：相对上抬 z-18mm..." << std::endl;
    demo->RelMovJDemo(postUp, 0, 5, 20, 50, 100);
    std::cout << "[SideAhead] 退出：经安全途经点 → pointsafe..." << std::endl;
    moveToSafeViaWaypointsForBrush(demo);
}

void runSideAheadGuideWithTransition(DobotTcpDemo *demo,
                                     const std::vector<Dobot::CDescartesPoint> &pickedPathPoints,
                                     const std::string &stageTag,
                                     const Dobot::CDescartesPoint *trajStartAfterGuide)
{
    Dobot::MovSParams pickedParams{};
    pickedParams.tool = 0;
    pickedParams.user = 0;
    pickedParams.v = 80;
    pickedParams.a = 80;
    pickedParams.freq = 0.2;

    std::cout << "[" << stageTag << "] 先到前牙引导轨迹起点..." << std::endl;
    demo->moveRobotC(pickedPathPoints.front(), pickedPathPoints.front());
    std::cout << "[" << stageTag << "] 执行前牙引导轨迹 (" << pickedPathPoints.size() << " 点)..."
              << std::endl;
    demo->movsDemoC(pickedPathPoints, pickedParams);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    if (trajStartAfterGuide == nullptr)
        return;

    Dobot::CDescartesPoint guideExitPose = makeSideAheadGuideExitPose();
    std::cout << "[" << stageTag << "] 引导完成，MovJ 过渡位 (" << guideExitPose.x << ", "
              << guideExitPose.y << ", " << guideExitPose.z << ")..." << std::endl;
    demo->moveRobotC(guideExitPose, guideExitPose);
    std::cout << "[" << stageTag << "] 过渡完成，自动前往刷牙轨迹起点 (退 38mm)..." << std::endl;
    demo->moveRobotC(*trajStartAfterGuide, *trajStartAfterGuide);
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
    while (trajFile >> x >> y >> z >> rx >> ry >> rz)
    {
        allPoints.push_back({x, y, z, rx, ry, rz});
    }
    trajFile.close();

    if (folderName == "rightup")
    {
        Dobot::CDescartesPoint rotatetooljoint{};
        rotatetooljoint.x = 0.1;
        rotatetooljoint.y = 0;
        rotatetooljoint.z = 0;
        rotatetooljoint.rx = 0;
        rotatetooljoint.ry = 0;
        rotatetooljoint.rz = 0;
        prepareRegionEntry(demo, rotatetooljoint, "rightup", 50.0);
    }
    else if (folderName == "leftup")
    {
        Dobot::CDescartesPoint rotatetooljoint{};
        rotatetooljoint.x = 0.1;
        rotatetooljoint.y = 0;
        rotatetooljoint.z = 0;
        rotatetooljoint.rx = 0;
        rotatetooljoint.ry = 0;
        rotatetooljoint.rz = 0;
        prepareRegionEntry(demo, rotatetooljoint, "leftup", 50.0);
    }
    else if (folderName == "rightside")
    {
        brushvalue = 1;
        Dobot::CDescartesPoint rotatetooljoint{};
        rotatetooljoint.x = 0.1;
        rotatetooljoint.y = 0;
        rotatetooljoint.z = 0;
        rotatetooljoint.rx = 0;
        rotatetooljoint.ry = -45;
        rotatetooljoint.rz = 0;

        prepareRegionEntry(demo, rotatetooljoint, "rightside", 50.0);
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

        prepareRegionEntry(demo, rotatetooljoint, "leftside", 50.0);
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

        prepareRegionEntry(demo, rotatetooljoint, "rightinside", 50.0);
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

        prepareRegionEntry(demo, rotatetooljoint, "leftinside", 50.0);
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

        std::vector<Dobot::CDescartesPoint> pickedPathPoints;
        if (!loadSideAheadGuidePath(pickedPathPoints))
        {
            std::cerr << "[SideAhead] 引导轨迹不可用，跳过引导/过渡" << std::endl;
        }
        else
        {
            prepareRegionEntry(demo, rotatetooljoint, "sideahead", 38.0);

            const Dobot::CDescartesPoint firstPosesk = allPoints[0];
            const Dobot::CDescartesPoint pointstartsk = computeBrushApproachStart(firstPosesk, 38.0);

            runSideAheadGuideWithTransition(demo, pickedPathPoints, "往复刷牙前", &pointstartsk);
        }
    }
    else if ((folderName == "center"))
    {
        brushvalue = 5;
        Dobot::CDescartesPoint rotatetooljoint{};
        rotatetooljoint.x = 0.01;
        rotatetooljoint.y = 0;
        rotatetooljoint.z = 0;
        rotatetooljoint.rx = 0;
        rotatetooljoint.ry = 0;
        rotatetooljoint.rz = 0;
        prepareRegionEntry(demo, rotatetooljoint, "center", 50.0);
    }
    std::cout << brushvalue << std::endl;

    std::ifstream indexFile(indexPath);
    if (!indexFile.is_open())
    {
        std::cerr << "无法打开索引文件: " << indexPath << std::endl;
        // return -1;
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
                // if (currentLine==0)
                // {
                    selectedPoints[0].z += 0.1;
                    demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
                    // std::this_thread::sleep_for(std::chrono::seconds(1));

                    Dobot::CDescartesPoint rotatetooljointjumps{};
                    rotatetooljointjumps.x = 0;
                    rotatetooljointjumps.y = 0;
                    rotatetooljointjumps.z = -10;
                    rotatetooljointjumps.rx = 0;
                    rotatetooljointjumps.ry = 0;
                    rotatetooljointjumps.rz = 0;
                    demo->RelMovJDemo(rotatetooljointjumps, 0, 5, 20, 50, 100);

                    selectedPoints[0].z -= 0.1;
                    demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
                // }
                // else
                // {
                //     demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
                // }

                // std::this_thread::sleep_for(std::chrono::seconds(1));

                demo->movsDemoC(selectedPoints, params);
                // std::this_thread::sleep_for(std::chrono::seconds(2));

                // 根据是否是最后一行选择不同的动作
                if (!isLastLine)
                {
                    // 非最后一行：执行原来的向后移动
                    std::cout << "hello 1 " << std::endl;
                    // Dobot::CDescartesPoint rotatetooljointjumpss{};
                    // rotatetooljointjumpss.x = -15;
                    // rotatetooljointjumpss.y = 0;
                    // rotatetooljointjumpss.z = 0;
                    // rotatetooljointjumpss.rx = 0;
                    // rotatetooljointjumpss.ry = 0;
                    // rotatetooljointjumpss.rz = 0;
                    // demo->RelMovJDemo(rotatetooljointjumpss, 0, 5, 20, 50, 100);
                }
                else
                {
                    std::cout << "hello " << std::endl;
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
                    // std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
            else
            {
                // if (currentLine==0)
                // {

                    // 開始刷牙
                    std::cout << "first ahead" <<std::endl;
                    selectedPoints[0].z += 0.1;
                    demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
                    // std::this_thread::sleep_for(std::chrono::seconds(1));

                    Dobot::CDescartesPoint rotatetooljointjumps{};
                    rotatetooljointjumps.x = 0;
                    rotatetooljointjumps.y = 0;
                    rotatetooljointjumps.z = -10;
                    rotatetooljointjumps.rx = 0;
                    rotatetooljointjumps.ry = 0;
                    rotatetooljointjumps.rz = 0;
                    demo->RelMovJDemo(rotatetooljointjumps, 0, 5, 20, 50, 100);

                    selectedPoints[0].z -= 0.1;
                    demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
                    // std::this_thread::sleep_for(std::chrono::seconds(1));
                // }
                // else
                // {
                //     std::cout << "second ahead" << std::endl;
                //     selectedPoints[0].z += 0.1;
                //     demo->moveRobotC(selectedPoints[0], selectedPoints[0]);

                //     selectedPoints[0].z -= 0.1;
                //     demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
                // }

                Dobot::MovSParams params1;
                params1.tool = 0;
                params1.user = 0;
                params1.v = 80;
                params1.a = 80;
                params1.freq = 0.2;

                demo->movsDemoC(selectedPoints, params1);
                // std::this_thread::sleep_for(std::chrono::seconds(2));

                // 根据是否是最后一行选择不同的动作
                if (!isLastLine)
                {
                    std::cout << "hello 1 " << std::endl;
                    // 非最后一行：执行原来的向后移动
                    // Dobot::CDescartesPoint rotatetooljointjumpss{};
                    // rotatetooljointjumpss.x = -15;
                    // rotatetooljointjumpss.y = 0;
                    // rotatetooljointjumpss.z = 0;
                    // rotatetooljointjumpss.rx = 0;
                    // rotatetooljointjumpss.ry = 0;
                    // rotatetooljointjumpss.rz = 0;
                    // demo->RelMovJDemo(rotatetooljointjumpss, 0, 5, 20, 50, 100);
                    // std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                else
                {
                    std::cout << "hello " << std::endl;

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
        // std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else
    {
        executeSideAheadExit(demo);
    }

    if (brushvalue != 6)
    {
        Dobot::CDescartesPoint pointsafe{};
        pointsafe.x = 264.8929;
        pointsafe.y = -285.1852;
        pointsafe.z = 491.0669;
        pointsafe.rx = -179.7725;
        pointsafe.ry = -1.3507;
        pointsafe.rz = -145.9055;

        demo->moveRobotC(pointsafe, pointsafe);
    }

    if (rebrushvalue)
    {
        // 先调整姿态
        if (folderName == "rightup")
        {
            Dobot::CDescartesPoint rotatetooljoint{};
            rotatetooljoint.x = 0.1;
            rotatetooljoint.y = 0;
            rotatetooljoint.z = 0;
            rotatetooljoint.rx = 0;
            rotatetooljoint.ry = 0;
            rotatetooljoint.rz = 0;
            prepareRegionEntry(demo, rotatetooljoint, "rightup", 50.0);
        }
        else if (folderName == "leftup")
        {
            Dobot::CDescartesPoint rotatetooljoint{};
            rotatetooljoint.x = 0.1;
            rotatetooljoint.y = 0;
            rotatetooljoint.z = 0;
            rotatetooljoint.rx = 0;
            rotatetooljoint.ry = 0;
            rotatetooljoint.rz = 0;
            prepareRegionEntry(demo, rotatetooljoint, "leftup", 50.0);
        }
        else if (folderName == "rightside")
        {
            brushvalue = 1;
            Dobot::CDescartesPoint rotatetooljoint{};
            rotatetooljoint.x = 0.1;
            rotatetooljoint.y = 0;
            rotatetooljoint.z = 0;
            rotatetooljoint.rx = 0;
            rotatetooljoint.ry = -45;
            rotatetooljoint.rz = 0;

            prepareRegionEntry(demo, rotatetooljoint, "rightside", 50.0);
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

            prepareRegionEntry(demo, rotatetooljoint, "leftside", 50.0);
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

            prepareRegionEntry(demo, rotatetooljoint, "rightinside", 50.0);
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

            prepareRegionEntry(demo, rotatetooljoint, "leftinside", 50.0);
        }
        else if (folderName == "center")
        {
            brushvalue = 5;
            Dobot::CDescartesPoint rotatetooljoint{};
            rotatetooljoint.x = 0.01;
            rotatetooljoint.y = 0;
            rotatetooljoint.z = 0;
            rotatetooljoint.rx = 0;
            rotatetooljoint.ry = 0;
            rotatetooljoint.rz = 0;
            prepareRegionEntry(demo, rotatetooljoint, "center", 50.0);
        }
        else if (folderName == "sideahead")
        {
            brushvalue = 6;
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
                Dobot::CDescartesPoint rotatetooljoint{};
                rotatetooljoint.x = 0.1;
                rotatetooljoint.y = 0;
                rotatetooljoint.z = 0;
                rotatetooljoint.rx = 0;
                rotatetooljoint.ry = -45;
                rotatetooljoint.rz = 0;

                std::vector<Dobot::CDescartesPoint> pickedPathPoints;
                if (loadSideAheadGuidePath(pickedPathPoints))
                {
                    prepareRegionEntry(demo, rotatetooljoint, "sideahead", 38.0);
                    const Dobot::CDescartesPoint firstPosesk = allPoints[currentTargetId];
                    const Dobot::CDescartesPoint pointstartsk =
                        computeBrushApproachStart(firstPosesk, 38.0);
                    runSideAheadGuideWithTransition(demo, pickedPathPoints, "浮刷前", &pointstartsk);
                }
            }

            allPoints[currentTargetId].z += 20;
            demo->moveRobotC(allPoints[currentTargetId], allPoints[currentTargetId]);
            // std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待稳定

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
                rotatetooljointjumps.z = -48;
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

            allPoints[currentTargetId].z -= 20;
            demo->moveRobotC(allPoints[currentTargetId], allPoints[currentTargetId]);

            // 强行走浮刷

            Eigen::Matrix3d rotationMatrix = eulerDegToRotationMatrix(allPoints[currentTargetId].rx, allPoints[currentTargetId].ry, allPoints[currentTargetId].rz);
            Eigen::Vector3d brushDir = rotationMatrix.col(2);
            brushDir.normalize();

            Dobot::CDescartesPoint rebrushpoints{};
            rebrushpoints.x = allPoints[currentTargetId].x + brushDir.x() * 6;
            rebrushpoints.y = allPoints[currentTargetId].y + brushDir.y() * 6;
            rebrushpoints.z = allPoints[currentTargetId].z + brushDir.z() * 6;
            rebrushpoints.rx = allPoints[currentTargetId].rx;
            rebrushpoints.ry = allPoints[currentTargetId].ry;
            rebrushpoints.rz = allPoints[currentTargetId].rz;

            demo->moveRobotC(rebrushpoints, rebrushpoints);

            // std::this_thread::sleep_for(std::chrono::seconds(1));

            // B. 执行您要求的特殊动作 (向 X 负方向移动 15mm)

            std::cout << "到达点 " << currentTargetId << "，执行回退补偿动作..." << std::endl;
            demo->RelMovJDemo(rotatetooljointjumpss, 0, 5, 20, 50, 100);

            // C. 动作间隙停顿
            // std::this_thread::sleep_for(std::chrono::seconds(1));

            // 如果读到最后一行，自动结束循环
            if (i == targetIds.size() - 1)
            {
                std::cout << "已完成最后一行索引，流程结束。" << std::endl;
            }
            selectedPointslast.back() = allPoints[currentTargetId];
        }

        if (brushvalue == 6)
        {
            executeSideAheadExit(demo);
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
            // std::this_thread::sleep_for(std::chrono::seconds(1));
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

    std::cout << "=== BrushTooth v20260620-2 (tool5 lift+rotate, no transition Enter) ===" << std::endl;
    applyBrushTcpFromJson(demo, "../defaultconfig/brushoffsets.json");

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

    // 设定标准参考点 (第一个牙刷到达的位置)
    const double REF_X = 264.8929;
    const double REF_Y = -285.1852;
    const double REF_Z = 391.0669;

    // 1. 进入键盘微调模式，手动对准后获取 Offset
    Eigen::Vector3d totalOffset{0, 0, 0};

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
            std::cout << "\n>> 开始区域: " << entry.path().filename().string() << std::endl;
            processFolderTrajectory(demo, entry.path().string(), entry.path().filename().string(), params, totalOffset, rebrush);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    demo->moveRobotC(pointsafe, pointsafe);

    delete demo;
    return 0;
}