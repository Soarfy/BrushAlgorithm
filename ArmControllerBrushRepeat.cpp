#include "VideoCapture.hpp"
#include <cmath>
#include <conio.h>
#include <crtdbg.h>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>
#include <limits>
#include "DobotTcpDemo.h"
#include <windows.h>
#include "kw-lib-all.h"
#include <string>
#include "nlohmann/json.hpp"
using json = nlohmann::json;

#define MODE 0
bool capturing = true;
NS_KW_USING

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
constexpr double RAD2DEG = 180.0 / M_PI;

/* ======================= 数据结构 ======================= */
struct PointData
{
    double x, y, z, a, b, c;
};

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
    Eigen::Matrix3d rotationMatrix = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
    return rotationMatrix;
}

Eigen::Vector3d getRotatedZAxisFromDegrees(double rx_deg, double ry_deg, double rz_deg)
{
    double rx = degToRad(rx_deg);
    double ry = degToRad(ry_deg);
    double rz = degToRad(rz_deg);
    Eigen::AngleAxisd roll(rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(rz, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw * pitch * roll;
    return q * Eigen::Vector3d::UnitZ();
}

/* ======================= 键盘微调函数 ======================= */
void fineTuneXYZ(DobotTcpDemo *demo, Dobot::CDescartesPoint &curPose,
                 Eigen::Vector3d &totalOffset)
{

    Eigen::Matrix3d rotationMatrixs = eulerDegToRotationMatrix(curPose.rx, curPose.ry, curPose.rz);
    Eigen::Vector3d brushDirs = rotationMatrixs.col(2);
    brushDirs.normalize();

    std::cout << "R pressed, brushDirs = " << brushDirs.x() << ", "
              << brushDirs.y() << ", " << brushDirs.z() << std::endl;

    std::cout << "\n===== XYZ 微调模式 =====\n"
              << "W/S : +Y / -Y\n"
              << "A/D : -X / +X\n"
              << "Q/E : +Z / -Z\n"
              << "Enter : 结束微调\n";
    while (true)
    {
        if (_kbhit())
        {

            std::cout << "\n===== XYZ 微调模式 =====\n"
                      << "W/S : +Y / -Y\n"
                      << "A/D : -X / +X\n"
                      << "Q/E : +Z / -Z\n"
                      << "Enter : 结束微调\n";
            char key = _getch();

            std::cout << "key" << key << std::endl;

            double dx = 0, dy = 0, dz = 0;

            if (key == 'w')
            {
                dx += -0.017294;
                dy += -0.016502;
                dz += -0.999714;
            }

            else if (key == 's')
            {
                dx += 0.017294;
                dy += 0.016502;
                dz += 0.999714;
            }
            else if (key == 'a')
            {
                dx += 0.827884;
                dy += 0.560404;
                dz += -0.023572;
            }
            else if (key == 'd')
            {
                dx += -0.827884;
                dy += -0.560404;
                dz += 0.023572;
            }
            else if (key == 'q')
            {
                dx += 0.560633;
                dy += -0.828055;
                dz += 0.003970;
            }
            else if (key == 'e')
            {
                dx += -0.560633;
                dy += 0.828055;
                dz += -0.003970;
            }
            else if (key == 'z')
            {
                std::cout << " hello " << brushDirs.x() << std::endl;
                dx += brushDirs.x();
                dy += brushDirs.y();
                dz += brushDirs.z();
            }
            else if (key == 'x')
            {
                dx += -brushDirs.x();
                dy += -brushDirs.y();
                dz += -brushDirs.z();
            }
            else if (key == 13)
            {
                std::cout << "微调结束\n";
                break;
            }
            else
            {
                continue;
            }

            totalOffset += Eigen::Vector3d(dx, dy, dz);

            curPose.x += dx;
            curPose.y += dy;
            curPose.z += dz;

            demo->moveRobotC(curPose, curPose);

            std::cout << "累计偏移 [mm]: " << totalOffset.transpose() << std::endl;
        }

        Sleep(10);
    }
}

int main()
{

    // ==================讀取json文件=======================
    std::string teethModelPath;
    std::string toothbrushPath;
    int brushIterations;
    double brushSpeed;
    int backAndForthCount;
    double pressureParameter;
    int brushDuration;

    try
    {
        // 打开文件
        std::ifstream file("D:\\UsmileProject\\hand_eye_calibration\\cofigfiles\\config.json");
        if (!file.is_open())
        {
            std::cerr << "无法打开 config.json 文件" << std::endl;
            return -1;
        }

        // 解析 JSON
        json j;
        file >> j;

        // 赋值给变量
        teethModelPath = j.at("teethModelPath").get<std::string>();
        toothbrushPath = j.at("toothbrushPath").get<std::string>();
        brushIterations = j.at("brushIterations").get<int>();
        brushSpeed = j.at("brushSpeed").get<double>();
        backAndForthCount = j.at("backAndForthCount").get<int>();
        pressureParameter = j.at("pressureParameter").get<double>();
        brushDuration = j.at("brushDuration").get<int>();

        // 打印测试
        std::cout << "teethModelPath: " << teethModelPath << std::endl;
        std::cout << "toothbrushPath: " << toothbrushPath << std::endl;
        std::cout << "brushIterations: " << brushIterations << std::endl;
        std::cout << "brushSpeed: " << brushSpeed << std::endl;
        std::cout << "backAndForthCount: " << backAndForthCount << std::endl;
        std::cout << "pressureParameter: " << pressureParameter << std::endl;
        std::cout << "brushDuration: " << brushDuration << std::endl;
    }

    catch (std::exception &e)
    {
        std::cerr << "JSON 解析错误: " << e.what() << std::endl;
        return -1;
    }

    SetConsoleOutputCP(CP_UTF8);
    CameraCapture *camera = new CameraCapture("169.254.7.168");
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

    SetConsoleCP(CP_UTF8);
    DobotTcpDemo *demo = new DobotTcpDemo();

    struct PointData
    {
        double x, y, z, a, b, c;
    };

    struct BrushVector
    {
        double x, y, z;
    };

    const std::string Force_FILE_PATH = "D:\\UsmileProject\\hand_eye_calibration\\sideright.txt";

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@回到初始态@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    Dobot::CDescartesPoint pointa{};
    pointa.x = 264.8929;
    pointa.y = -285.1852;
    pointa.z = 391.0669;
    pointa.rx = -179.7725;
    pointa.ry = -1.3507;
    pointa.rz = -145.9055;
    demo->moveRobotC(pointa, pointa);
    std::cout << "机械臂到达起始点" << std::endl;

    Dobot::CDescartesPoint pointsafe{};
    pointsafe.x = 264.8929;
    pointsafe.y = -285.1852;
    pointsafe.z = 491.0669;
    pointsafe.rx = -179.7725;
    pointsafe.ry = -1.3507;
    pointsafe.rz = -145.9055;
    bool userSatisfied = false;

    Dobot::CDescartesPoint rotatetooljoint{};
    rotatetooljoint.x = 0.1;
    rotatetooljoint.y = 0;
    rotatetooljoint.z = 0;
    rotatetooljoint.rx = 0;
    rotatetooljoint.ry = -30;
    rotatetooljoint.rz = 0;

    std::wcout << L"机械臂到达起始点，请确认按Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    // @@@@@@@@@@@@@@@@@@@@@@@@@@调整牙刷起始姿态@@@@@@@@@@@@@@@@@@
    demo->RelMovJDemo(rotatetooljoint, 0, 3, 20, 50, 100);
    std::cout << "牙刷初始姿态已经调整好，请确认Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    // ================= 读取轨迹 =================
    std::vector<PointData> brushpointsoffset_ee_poses;
    std::ifstream ee_poses_infile(Force_FILE_PATH);
    if (!ee_poses_infile.is_open())
    {
        std::cerr << "无法打开 ee_poses.txt" << std::endl;
        return -1;
    }
    double dx, dy, dz, rx, ry, rz;
    while (ee_poses_infile >> dx >> dy >> dz >> rx >> ry >> rz)
    {
        brushpointsoffset_ee_poses.push_back({dx, dy, dz, rx, ry, rz});
    }
    ee_poses_infile.close();
    if (brushpointsoffset_ee_poses.empty())
    {
        std::cerr << "轨迹为空！" << std::endl;
        return -1;
    }

    demo->moveRobotC(pointsafe, pointsafe);

    Dobot::CDescartesPoint firstPoses{};
    firstPoses.x = brushpointsoffset_ee_poses[0].x;
    firstPoses.y = brushpointsoffset_ee_poses[0].y;
    firstPoses.z = brushpointsoffset_ee_poses[0].z;
    firstPoses.rx = brushpointsoffset_ee_poses[0].a;
    firstPoses.ry = brushpointsoffset_ee_poses[0].b;
    firstPoses.rz = brushpointsoffset_ee_poses[0].c;
    Eigen::Matrix3d rotationMatrixss = eulerDegToRotationMatrix(firstPoses.rx, firstPoses.ry, firstPoses.rz);
    Eigen::Vector3d brushDirss = rotationMatrixss.col(2);
    brushDirss.normalize();

    Dobot::CDescartesPoint pointstarts{};
    pointstarts.x = firstPoses.x + -brushDirss.x() * 8;
    pointstarts.y = firstPoses.y + -brushDirss.y() * 8;
    pointstarts.z = firstPoses.z + -brushDirss.z() * 8;
    pointstarts.rx = firstPoses.rx;
    pointstarts.ry = firstPoses.ry;
    pointstarts.rz = firstPoses.rz;
    demo->moveRobotC(pointstarts, pointstarts);

    std::cout << "微调结束回到安全点，请确认按Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    demo->moveRobotC(pointsafe, pointsafe);

    std::cout << "运行完整力控轨迹（每行一次movsDemoC）" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    Dobot::MovSParams params1;
    params1.tool = 0;
    params1.user = 0;
    params1.v = 80;
    params1.a = 80;
    params1.freq = 0.8;

    std::string indexFilePath = "D:\\UsmileProject\\hand_eye_calibration\\all_segments.txt";
    std::ifstream indexFile(indexFilePath);
    if (!indexFile.is_open())
    {
        std::cerr << "无法打开索引文件: " << indexFilePath << std::endl;
        return -1;
    }

    std::vector<Dobot::CDescartesPoint> descartesPointsforce;
    for (const auto &p : brushpointsoffset_ee_poses)
    {
        Dobot::CDescartesPoint cp{};
        cp.x = p.x;
        cp.y = p.y;
        cp.z = p.z;
        cp.rx = p.a;
        cp.ry = p.b;
        cp.rz = p.c;
        descartesPointsforce.push_back(cp);
    }

    std::string line;
    int lineNumber = 0;
    while (std::getline(indexFile, line))
    {
        lineNumber++;
        std::istringstream iss(line);
        int idx;
        std::vector<Dobot::CDescartesPoint> selectedPoints;

        std::cout << "第 " << lineNumber << " 行读取的索引: ";

        while (iss >> idx)
        {
            std::cout << idx << " "; // 打印读取的索引

            // ID 从0开始
            if (idx >= 0 && idx < (int)descartesPointsforce.size())
            {
                selectedPoints.push_back(descartesPointsforce[idx]);
            }
            else
            {
                std::cerr << "\n第 " << lineNumber << " 行索引 " << idx << " 超出范围 (0-"
                          << descartesPointsforce.size() - 1 << ")" << std::endl;
            }
        }

        std::cout << std::endl;

        if (!selectedPoints.empty())
        {

            demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
            demo->movsDemoC(selectedPoints, params1);
            // std::cout << "第一個點：" << selectedPoints[0].x << selectedPoints[0].y << selectedPoints[0].z <<std::endl;
            // std::cout << "运行完整力控轨迹（每行一次movsDemoC）" << std::endl;
            // std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

            // double gx, gy, gz, grx, gry, grz;
            // double targetx, targety, targetz;
            // targetx = selectedPoints.back().x;
            // targety = selectedPoints.back().y;
            // targetz = selectedPoints.back().z;

            // while (true)
            // {
            //     demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz);

            //     double diffx = fabs(gx - targetx);
            //     double diffy = fabs(gy - targety);
            //     double diffz = fabs(gz - targetz);

            //     // 打印差值，看看實際數值
            //     printf("diffx: %.10f, diffy: %.10f, diffz: %.10f\n", diffx, diffy, diffz);
            //     printf("gx: %.10f, targetx: %.10f\n", gx, targetx);

            //     if (diffx < 0.01 && diffy < 0.01 && diffz < 0.01)
            //     {
            //         printf("條件滿足，跳出循環！\n");
            //         break;
            //     }

            //     // std::this_thread::sleep_for(std::chrono::milliseconds(10*selectedPoints.size()));
            // }
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cout << "执行第 " << lineNumber << " 行轨迹，共 " << selectedPoints.size() << " 个点\n";
        }
    }

    indexFile.close();

    double gxq, gyq, gzq, grxq, gryq, grzq;
    double targetx, targety, targetz;
    targetx = pointsafe.x;
    targety = pointsafe.y;
    targetz = pointsafe.z;

    while (true)
    {
        demo->getCurrentPose(0, 0, gxq, gyq, gzq, grxq, gryq, grzq);

        double diffx = fabs(gxq - targetx);
        double diffy = fabs(gyq - targety);
        double diffz = fabs(gzq - targetz);

        // 打印差值，看看實際數值
        printf("diffx: %.10f, diffy: %.10f, diffz: %.10f\n", diffx, diffy, diffz);
        printf("gx: %.10f, targetx: %.10f\n", gx, targetx);

        if (diffx < 0.01 && diffy < 0.01 && diffz < 0.01)
        {
            printf("條件滿足，跳出循環！\n");
            break;
        }
    }

    // 退出
    demo->moveRobotC(pointsafe, pointsafe);

    std::cout << "正在退出程序請稍後：-）" << std::endl;
    obj->StopCapture();
    camera->~CameraCapture();
    demo->~DobotTcpDemo();

    delete camera;
    delete demo;

    obj = nullptr;
    camera = nullptr;
    demo = nullptr;

    return 0;
}
