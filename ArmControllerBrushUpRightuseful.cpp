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
#include <thread>
#include <atomic>
#include <chrono>
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
    Eigen::Vector3d brushDirsz = rotationMatrixs.col(2);
    Eigen::Vector3d brushDirsy = rotationMatrixs.col(1);
    Eigen::Vector3d brushDirsx = rotationMatrixs.col(0);
    brushDirsz.normalize();
    brushDirsy.normalize();
    brushDirsx.normalize();

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
                dz += -0.0039700;
            }
            else if (key == 'z')
            {
                dx += brushDirsz.x();
                dy += brushDirsz.y();
                dz += brushDirsz.z();
            }
            else if (key == 'x')
            {
                dx += -brushDirsz.x();
                dy += -brushDirsz.y();
                dz += -brushDirsz.z();
            }
            else if (key == 'c')
            {
                dx += brushDirsy.x();
                dy += brushDirsy.y();
                dz += brushDirsy.z();
            }
            else if (key == 'v')
            {
                dx += -brushDirsy.x();
                dy += -brushDirsy.y();
                dz += -brushDirsy.z();
            }
            else if (key == 'b')
            {
                dx += brushDirsx.x();
                dy += brushDirsx.y();
                dz += brushDirsx.z();
            }
            else if (key == 'n')
            {
                dx += -brushDirsx.x();
                dy += -brushDirsx.y();
                dz += -brushDirsx.z();
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
Eigen::Vector3d getManualOffset(DobotTcpDemo *demo, double refX, double refY, double refZ)
{
    double gx, gy, gz, grx, gry, grz;
    while (!demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    Dobot::CDescartesPoint curPose{refX, refY, refZ, -179.7725, -1.3507, -145.9055};
    Eigen::Vector3d totalOffset(0, 0, 0);

    std::cout << "\n===== 刷頭微調 =====\n"
              << "W/S : +Y / -Y\n"
              << "A/D : -X / +X\n"
              << "Q/E : +Z / -Z\n"
              << "Enter : 结束微调\n";

    while (true)
    {
        if (_kbhit())
        {
            std::cout << "\n===== 刷頭微調 =====\n"
                      << "W/S : +Y / -Y\n"
                      << "A/D : -X / +X\n"
                      << "Q/E : +Z / -Z\n"
                      << "Enter : 结束微调\n";

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
            std::cout << "\r当前牙刷累计轨迹偏移 [mm]: " << std::endl;
            demo->moveRobotC(curPose, curPose);

            // 实时打印当前相对基准点的偏移
            Eigen::Vector3d currentRelativeOffset(curPose.x - refX, curPose.y - refY, curPose.z - refZ);
            std::cout << "\r当前牙刷累计轨迹偏移 [mm]: " << currentRelativeOffset.transpose() << "    " << std::flush;
        }
        Sleep(10);
    }
    return Eigen::Vector3d(curPose.x - refX, curPose.y - refY, curPose.z - refZ);
}

struct Vector3
{
    double x, y, z;
};

Vector3 transformVectorAToB(Vector3 vA, double rx, double ry, double rz)
{
    double ax = degToRad(rx);
    double ay = degToRad(ry);
    double az = degToRad(rz);

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

// [MOD-FORCE-LOG-BEGIN] movs段力数据结构与输出路径
struct MovsForceSample
{
    int lineNo;
    int seq;
    float fx, fy, fz, mx, my, mz;
};
// [MOD-FORCE-LOG-END]

int main()
{
    // @@@@@@@@@@@@@@@@@@@@@@@@區別代碼@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    const std::string Force_FILE_PATH = "../defaultconfig/rightup/upright.txt";
    // [MOD-FORCE-LOG] 完整力控轨迹(movsDemoC)阶段的力数据日志路径
    const std::string Movs_FORCE_LOG_PATH = "../defaultconfig/rightup/movs_force_during_movss.txt";
    const std::string Movs_FORCE_LOG_PATHs = "../defaultconfig/rightup/movs_force_during_movs.txt";
    // const std::string Brush_offset = "../defaultconfig/rightup/brushoffsets.json";
    const std::string Brush_offset = "../defaultconfig/brushoffsets.json";
    const std::string Brush_offset_path = "../defaultconfig/rightup/brushoffsets_path.json";
    const std::string Brush_Config = "../defaultconfig/config.json";
    int value = 0;

    std::string command1 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\GenerateAnyPath.py " +
                           std::to_string(value) + "\"";

    std::string command2 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\GenerateAnyPathRepeat.py " +
                           std::to_string(value) + "\"";

    std::string command3 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\TCPRotation10OthersDorobot.py " +
                           std::to_string(value) + "\"";

    const std::string oldsegment = "../defaultconfig/rightup/all_segments.txt";
    std::string indexFilePath = "../defaultconfig/rightup/all_segments.txt";
    std::string indexFilePath2 = "../defaultconfig/rightup/support_points.txt";
    std::ofstream poseFile("../defaultconfig/rightup/current_pose_from_getpose.txt");
    const std::string eepath = "../defaultconfig/rightup/ee_poses.txt";

    Dobot::CDescartesPoint rotatetooljoint{};
    rotatetooljoint.x = 0.1;
    rotatetooljoint.y = 0;
    rotatetooljoint.z = 0;
    rotatetooljoint.rx = 0;
    rotatetooljoint.ry = 0;
    rotatetooljoint.rz = 0;

    Dobot::CDescartesPoint rotatetooljointjump{};
    rotatetooljointjump.x = 0;
    rotatetooljointjump.y = 0;
    rotatetooljointjump.z = -50;
    rotatetooljointjump.rx = 0;
    rotatetooljointjump.ry = 0;
    rotatetooljointjump.rz = 0;

    double modifiedup = 0;
    double modifiedupx =  -6.9142260000002125 +14.163013999999862;
    double modifiedupy = -2.0350259999999025 -24.190834000000166;
    double modifiedupz = 48.16690799999924 +7.156301999999357;

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
        std::ifstream file(Brush_Config);
        if (!file.is_open())
        {
            std::cerr << "无法打开 config.json 文件" << std::endl;
            return -1;
        }
        json j;
        file >> j;
        teethModelPath = j.at("teethModelPath").get<std::string>();
        toothbrushPath = j.at("toothbrushPath").get<std::string>();
        brushIterations = j.at("brushIterations").get<int>();
        brushSpeed = j.at("brushSpeed").get<double>();
        backAndForthCount = j.at("backAndForthCount").get<int>();
        pressureParameter = j.at("pressureParameter").get<double>();
        brushDuration = j.at("brushDuration").get<int>();

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

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@設備初始化@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    SetConsoleOutputCP(CP_UTF8);
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
        return -1;
    }
    SetConsoleCP(CP_UTF8);
    DobotTcpDemo *demo = new DobotTcpDemo();

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@回到初始态@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    Dobot::CDescartesPoint pointsafe{};
    pointsafe.x = 264.8929;
    pointsafe.y = -285.1852;
    pointsafe.z = 491.0669;
    pointsafe.rx = -179.7725;
    pointsafe.ry = -1.3507;
    pointsafe.rz = -145.9055;
    demo->moveRobotC(pointsafe, pointsafe);

    Dobot::CDescartesPoint pointa{};
    pointa.x = 264.8929+modifiedupx;
    pointa.y = -285.1852+modifiedupy;
    pointa.z = 391.0669 + modifiedup +modifiedupz;
    pointa.rx = -179.7725;
    pointa.ry = -1.3507;
    pointa.rz = -145.9055;
    demo->moveRobotC(pointa, pointa);

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@微調牙刷@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    const double REF_X = 264.8929+modifiedupx;
    const double REF_Y = -285.1852+modifiedupy;
    const double REF_Z = 391.0669 + modifiedup+ modifiedupz;
    std::cout << "是否進行牙刷微調？(y/n): ";
    char userInput;
    std::cin >> userInput;
    
    if (userInput == 'y' || userInput == 'Y')
    {
        Eigen::Vector3d totalOffset = getManualOffset(demo, REF_X, REF_Y, REF_Z);
        json offsetJson;
        offsetJson["brushxoffset"] = totalOffset.x();
        offsetJson["brushyoffset"] = totalOffset.y();
        offsetJson["brushzoffset"] = totalOffset.z();

        // 转移到机械臂末端的误差
        Vector3 vecA = {totalOffset.x(), totalOffset.y(), totalOffset.z()};
        Vector3 vecB = transformVectorAToB(vecA, -179.7725, -1.3507, -145.9055);
        std::cout << "向量在坐标系 B 下的值为：" << std::endl;
        std::cout << "X: " << vecB.x << "\nY: " << vecB.y << "\nZ: " << vecB.z << std::endl;

        offsetJson["brushxoffsets"] = vecB.x;
        offsetJson["brushyoffsets"] = vecB.y;
        offsetJson["brushzoffsets"] = vecB.z;

        std::ofstream file(Brush_offset);
        file << offsetJson.dump(4);
        file.close();

        std::cout << "牙刷微調完成，偏移量已保存。" << std::endl;

        double tcpx = -0.9317 - vecB.x;
        double tcpy = -185.519801 - vecB.y;
        double tcpz = 185.399197 - vecB.z;
        double tcprx = 0.0;
        double tcpry = 0.0;
        double tcprz = 0.0;

        std::string tcpvalue = "{" + std::to_string(tcpx) + "," +
                               std::to_string(tcpy) + "," +
                               std::to_string(tcpz) + "," +
                               std::to_string(tcprx) + "," +
                               std::to_string(tcpry) + "," +
                               std::to_string(tcprz) + "}";

        demo->setToolDemo(5, tcpvalue);
    }
    else
    {
        std::ifstream inputFile(Brush_offset);
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

            Dobot::CDescartesPoint pointa{};
            pointa.x = 264.8929+modifiedupx + offsetX;
            pointa.y = -285.1852+modifiedupy + offsetY;
            pointa.z = 391.0669 + modifiedupz + modifiedup + offsetZ;
            pointa.rx = -179.7725;
            pointa.ry = -1.3507;
            pointa.rz = -145.9055;
            demo->moveRobotC(pointa, pointa);
            std::cout << "机械臂到达起始点" << std::endl;
            std::cout << "跳過牙刷微調。" << std::endl;

            double tcpx = 0.474288 - offsetXs;
            double tcpy =-189.5420141 - offsetYs;
            double tcpz = 185.889558 - offsetZs;
            double tcprx = 0.0;
            double tcpry = 0.0;
            double tcprz = 0.0;

            std::string tcpvalue = "{" + std::to_string(tcpx) + "," +
                                   std::to_string(tcpy) + "," +
                                   std::to_string(tcpz) + "," +
                                   std::to_string(tcprx) + "," +
                                   std::to_string(tcpry) + "," +
                                   std::to_string(tcprz) + "}";

            demo->setToolDemo(5, tcpvalue);
        }
    }

    bool userSatisfied = false;

    // @@@@@@@@@@@@@@@@@@@@@@@@@@牙刷標定后軌跡選擇@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    std::cout << "是否生成新軌跡？(y/n): ";
    char choice;
    std::cin >> choice;

    std::cout << "選擇" << choice << std::endl;

    if (choice == 'y' || choice == 'Y')
    {
        std::cout << "生成的轨迹..." << std::endl;
        int python_result2 = std::system(command1.c_str());
        if (python_result2 == 0)
        {
            std::cout << "新轨迹生成成功！" << std::endl;
        }
        else
        {
            std::cout << "新轨迹生成失败！" << std::endl;
            return -1;
        }
    }
    else
    {
        std::cout << "正在使用已导入的轨迹..." << std::endl;
        int python_result1 = std::system(command2.c_str());
        if (python_result1 == 0)
        {
            std::cout << "已导入轨迹使用成功！" << std::endl;
            indexFilePath = oldsegment;
        }
        else
        {
            std::cout << "已导入轨迹使用失败！" << std::endl;
            return -1;
        }
    }

    // @@@@@@@@@@@@@@@@@@@@@@@@@@调整牙刷起始姿态@@@@@@@@@@@@@@@@@@

    // 先上去，再旋转，再下来
    std::cout << "牙刷初始旋转姿态调整，请确认Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
    demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
    demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);
    std::cout << "牙刷初始旋转姿态已经调整好，请确认Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    double gxdown, gydown, gzdown, grxdown, grydown, grzdown;
    while (!demo->getCurrentPose(0, 0, gxdown, gydown, gzdown, grxdown, grydown, grzdown))
    {
        std::cout << "获取姿态中。。。。。" << std::endl;
    }
    Dobot::CDescartesPoint firstPoseback{};
    firstPoseback.x = gxdown;
    firstPoseback.y = gydown;
    firstPoseback.z = gzdown - 50;
    firstPoseback.rx = grxdown;
    firstPoseback.ry = grydown;
    firstPoseback.rz = grzdown;
    demo->moveRobotC(firstPoseback, firstPoseback);
    std::cout << "牙刷初始姿态已经调整好，请确认Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    // ========== 从控制器读取真实位姿（GetPose） ==========
    double gx, gy, gz, grx, gry, grz;
    while (true)
    {
        if (demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
        {
            std::cout << gx << " " << gy << " " << gz << " "
                      << grx << " " << gry << " " << grz << std::endl;

            if (!std::isnan(gx) && !std::isnan(gy) && !std::isnan(gz) &&
                !std::isnan(grx) && !std::isnan(gry) && !std::isnan(grz))
            {

                if (poseFile.is_open())
                {
                    poseFile << gx << " " << gy << " " << gz << " "
                             << grx << " " << gry << " " << grz << std::endl;
                    poseFile.close();
                    std::cout << "当前刷头位置已经保存下去\n";
                }
                else
                {
                    std::cerr << "当前刷头位置已经保存失败\n";
                }
                break;
            }
            else
            {
                std::cerr << "获取当前刷头位置函数返回失败\n";
            }
        }
        else
        {
            std::cerr << "获取当前刷头位置失败，重新尝试\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
    }

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@生成的轨迹转移到机械臂末端@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    std::cout << "将轨迹转移到机械臂末端" << std::endl;
    int python_result11 = std::system(command3.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    firstPoseback.z += 20;
    demo->moveRobotC(firstPoseback, firstPoseback);

    // ================= 读取轨迹 =================

    std::ifstream ee_poses_infile(eepath);
    std::vector<PointData> brushpointsoffset_ee_poses;
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

    while (!userSatisfied)
    {
        std::cout << "\n=========== 新一轮轨迹调整开始 ===========\n";

        Dobot::CDescartesPoint firstPose{};
        firstPose.x = brushpointsoffset_ee_poses[0].x;
        firstPose.y = brushpointsoffset_ee_poses[0].y;
        firstPose.z = brushpointsoffset_ee_poses[0].z;
        firstPose.rx = brushpointsoffset_ee_poses[0].a;
        firstPose.ry = brushpointsoffset_ee_poses[0].b;
        firstPose.rz = brushpointsoffset_ee_poses[0].c;
        demo->moveRobotC(pointsafe, pointsafe);
        demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

        // 根據軌跡向量來補償
        Eigen::Matrix3d rotationMatrixs = eulerDegToRotationMatrix(firstPose.rx, firstPose.ry, firstPose.rz);
        Eigen::Vector3d brushDirs = rotationMatrixs.col(2);
        brushDirs.normalize();

        Dobot::CDescartesPoint pointstart{};
        pointstart.x = firstPose.x + -brushDirs.x() * 8;
        pointstart.y = firstPose.y + -brushDirs.y() * 8;
        pointstart.z = firstPose.z + -brushDirs.z() * 8;
        pointstart.rx = firstPose.rx;
        pointstart.ry = firstPose.ry;
        pointstart.rz = firstPose.rz;
        demo->moveRobotC(pointstart, pointstart);
        // 從 offsetpath.json 讀取 deltaOffset 初始值
        Eigen::Vector3d deltaOffset(0, 0, 0);

        // 檢查檔案是否存在，若存在則讀取
        // std::ifstream file(Brush_offset_path);
        // if (file.good())
        // {
        //     nlohmann::json j;
        //     file >> j;
        //     deltaOffset[0] = j["offsetpath_x"];
        //     deltaOffset[1] = j["offsetpath_y"];
        //     deltaOffset[2] = j["offsetpath_z"];
        // }
        // 若檔案不存在，則保持預設的 (0, 0, 0)
        // firstPose.x += deltaOffset[0];
        // firstPose.y += deltaOffset[1];
        // firstPose.z += deltaOffset[2];

        demo->moveRobotC(firstPose, firstPose);

        // std::cout << "机械臂到达起始点，请确认按Enter" << std::endl;
        // std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

        // 執行 fineTuneXYZ 進行調整
        fineTuneXYZ(demo, firstPose, deltaOffset);

        // 調整完成後，將更新後的 deltaOffset 存回 JSON
        // {
        //     nlohmann::json j;
        //     j["offsetpath_x"] = deltaOffset[0];
        //     j["offsetpath_y"] = deltaOffset[1];
        //     j["offsetpath_z"] = deltaOffset[2];

        //     std::ofstream out_file(Brush_offset_path);
        //     out_file << j.dump(4);
        // }

        for (auto &p : brushpointsoffset_ee_poses)
        {
            p.x += deltaOffset.x();
            p.y += deltaOffset.y();
            p.z += deltaOffset.z();
        }

        std::vector<Dobot::CDescartesPoint> descartesPoints;
        for (const auto &p : brushpointsoffset_ee_poses)
        {
            Dobot::CDescartesPoint cp{};
            cp.x = p.x;
            cp.y = p.y;
            cp.z = p.z;
            cp.rx = p.a;
            cp.ry = p.b;
            cp.rz = p.c;
            descartesPoints.push_back(cp);
        }

        Dobot::MovSParams params;
        params.tool = 0;
        params.user = 0;
        params.v = 80;
        params.a = 80;
        // params.speed = brushSpeed;
        params.freq = 0.2;
        demo->movsDemoC(descartesPoints, params);

        std::cout << "\n是否满意当前调整后的轨迹？(y/n): ";
        char choice;
        std::cin >> choice;
        std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

        demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);

        if (choice == 'y' || choice == 'Y')
        {
            userSatisfied = true;
            std::cout << "调整完成 ✅\n";
        }
        else
        {
            std::cout << "继续调整...\n";
        }
    }

    demo->moveRobotC(pointsafe, pointsafe);
    demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

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

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@添加力控@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    double targetforcevalue = pressureParameter;
    std::cout << "\n理想的壓力值是 :  " << targetforcevalue << std::endl;
    std::vector<Dobot::CDescartesPoint> descartesPointsforce;
    std::ofstream forcerepairedoutputfile(Force_FILE_PATH);
    std::ofstream forcerepaired(Movs_FORCE_LOG_PATHs);
    if (!forcerepairedoutputfile.is_open())
    {
        std::cerr << "无法保存含有力控的路径" << std::endl;
        return -1;
    }

    if (!forcerepaired.is_open())
    {
        std::cerr << "无法保存含有力控的路径" << std::endl;
        return -1;
    }

    // @@@@@@@@@@@@@@@@@@@对6维力清零@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    float forcefirst[6];
    int sampleCount = 0;
    const int maxSamples = 30;
    bool success = false;
    while (sampleCount < maxSamples)
    {
        int result = obj->GetCurrentForceData(forcefirst);
        if (result == 28 && forcefirst[2] > -3.0 && forcefirst[0] < 1.0 && forcefirst[2] < -1.0)
        {
            success = true;
            break;
        }
        sampleCount++;
        if (sampleCount < maxSamples)
        {
            std::cerr << "采样 " << sampleCount << " 次失败，继续采样...\n";
        }
        else
        {
            std::cerr << "采样 " << sampleCount << " 次均失败\n";
        }
    }

    printf("力控X: %.2f 力控Y: %.2f 力控Z: %.2f \n", forcefirst[0], forcefirst[1], forcefirst[2]);

    std::vector<Dobot::CDescartesPoint> descartesPoints;
    for (const auto &p : brushpointsoffset_ee_poses)
    {
        Dobot::CDescartesPoint cp{};
        cp.x = p.x;
        cp.y = p.y;
        cp.z = p.z;
        cp.rx = p.a;
        cp.ry = p.b;
        cp.rz = p.c;
        descartesPoints.push_back(cp);
    }

     std::cout << "开始结合力控进行运动" << std::endl;
    int firstcount = 0;

    for (size_t i = 0; i < descartesPoints.size(); ++i)
    {
        auto &offset = descartesPoints[i];
        bool converged = false;
        bool firststep = false;

        while (!converged)
        {
            Eigen::Matrix3d rotationMatrix = eulerDegToRotationMatrix(offset.rx, offset.ry, offset.rz);
            Eigen::Vector3d brushDir = rotationMatrix.col(2);
            brushDir.normalize();
            // 沿用上次的调整(理想状态下会加速)
            if (firststep)
            {
                Eigen::Vector3d delta = firstcount * 0.6 * brushDir;
                offset.x += delta.x();
                offset.y += delta.y();
                offset.z += delta.z();
                firststep = false;
            }

            demo->moveRobotC(offset, offset);
            double gxsa, gysa, gzsa, grxsa, grysa, grzsa;
            double targetxsa, targetysa, targetzsa;
            targetxsa = offset.x;
            targetysa = offset.y;
            targetzsa = offset.z;
            while (true)
            {
                if (demo->getCurrentPose(0, 0, gxsa, gysa, gzsa, grxsa, grysa, grzsa))
                {
                    double diffxsa = fabs(gxsa - targetxsa);
                    double diffysa = fabs(gysa - targetysa);
                    double diffzsa = fabs(gzsa - targetzsa);

                    if (diffxsa < 0.01 && diffysa < 0.01 && diffzsa < 0.01)
                    {
                        break;
                    }
                }
            }

            float force[6];
            while (obj->GetCurrentForceData(force) != 28 && force[2] != 0)
            {
                std::cerr << "获取力控数据失败\n";
                // return -1;
            }
            force[0] -= forcefirst[0];
            force[1] -= forcefirst[1];
            force[2] -= forcefirst[2];

            Eigen::Vector3d measured(force[0], force[1], force[2]);
            double proj = measured.dot(brushDir);
            double err = proj - targetforcevalue;
            if(targetforcevalue == 0){
                err = 0.0;
            }

            Eigen::Vector3d targetDir(0, 0, -1);
            double dotProduct = brushDir.dot(targetDir);
            double brushDirNorm = brushDir.norm();
            double targetDirNorm = targetDir.norm();
            double angleRad = std::acos(dotProduct / (brushDirNorm * targetDirNorm));
            double angleDeg = angleRad * 180.0 / M_PI;

            if (angleDeg > 6.0)
            {
                if (std::abs(err) <= 0.02)
                {
                    converged = true;
                    std::cout << "  proj 1= " << proj << std::endl;
                    forcerepaired << proj << std::endl;
                    break;
                }
                else if (err > 0.02)
                {
                    Eigen::Vector3d delta = -0.6 * brushDir;
                    offset.x += delta.x();
                    offset.y += delta.y();
                    offset.z += delta.z();
                    firstcount -= 1;
                }
                else if (err < -0.02)
                {
                    Eigen::Vector3d delta = 0.6 * brushDir;
                    offset.x += delta.x();
                    offset.y += delta.y();
                    offset.z += delta.z();
                    firstcount += 1;
                }
            }
            else
            {
                if (std::abs(err) <= 0.02)
                {
                    converged = true;
                    std::cout << "  proj 2= " << proj << std::endl;
                    forcerepaired << proj << std::endl;
                    break;
                }
                else if (err > 0.02)
                {
                    Eigen::Vector3d delta = -0.6 * brushDir;
                    offset.x += delta.x();
                    offset.y += delta.y();
                    offset.z += delta.z();
                    firstcount -= 1;
                }
                else if (err < -0.02)
                {
                    Eigen::Vector3d delta = 0.6 * brushDir;
                    offset.x += delta.x();
                    offset.y += delta.y();
                    offset.z += delta.z();
                    firstcount += 1;
                }
            }

            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // ##########################保存修正后的轨迹##########################
        forcerepairedoutputfile << offset.x << " "
                                << offset.y << " "
                                << offset.z << " "
                                << offset.rx << " "
                                << offset.ry << " "
                                << offset.rz << std::endl;

        descartesPointsforce.push_back(offset);
    }

    forcerepaired.close();
    forcerepairedoutputfile.close();
    std::cout << "调整后的力控轨迹保存完毕" << std::endl;
    demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
    demo->moveRobotC(pointsafe, pointsafe);
    demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

    Dobot::CDescartesPoint firstPosesk{};
    firstPosesk.x = descartesPointsforce[0].x;
    firstPosesk.y = descartesPointsforce[0].y;
    firstPosesk.z = descartesPointsforce[0].z;
    firstPosesk.rx = descartesPointsforce[0].rx;
    firstPosesk.ry = descartesPointsforce[0].ry;
    firstPosesk.rz = descartesPointsforce[0].rz;
    Eigen::Matrix3d rotationMatrixssk = eulerDegToRotationMatrix(firstPosesk.rx, firstPosesk.ry, firstPosesk.rz);
    Eigen::Vector3d brushDirssk = rotationMatrixssk.col(2);
    brushDirssk.normalize();

    Dobot::CDescartesPoint pointstartsk{};
    pointstartsk.x = firstPosesk.x + -brushDirssk.x() * 8;
    pointstartsk.y = firstPosesk.y + -brushDirssk.y() * 8;
    pointstartsk.z = firstPosesk.z + -brushDirssk.z() * 8;
    pointstartsk.rx = firstPosesk.rx;
    pointstartsk.ry = firstPosesk.ry;
    pointstartsk.rz = firstPosesk.rz;
    demo->moveRobotC(pointstartsk, pointstartsk);
    demo->moveRobotC(firstPosesk, firstPosesk);

    std::cout << "运行完整力控轨迹（每行一次movsDemoC）" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    // [MOD-FORCE-LOG-BEGIN] movsDemoC段重新力控清零（基准）
    // ---------- movsDemoC 段：重新力控清零（基准），仅在该段运动内采样力数据 ----------
    float forceMovsZero[6] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    {
        int sampleMovs = 0;
        bool movsZeroOk = false;
        while (sampleMovs < maxSamples)
        {
            int fr = obj->GetCurrentForceData(forceMovsZero);
            if (fr == 28 && forceMovsZero[2] > -3.0f && forceMovsZero[0] < 1.0f && forceMovsZero[2] < -1.0f)
            {
                movsZeroOk = true;
                break;
            }
            sampleMovs++;
            if (sampleMovs < maxSamples)
                std::cerr << "movs段力控清零 采样 " << sampleMovs << " 次失败，继续...\n";
        }
        if (!movsZeroOk)
            std::cerr << "movs段力控清零未达标，后续记录将相对最后一次读数作差\n";
        printf("movs段力控清零基准: X:%.2f Y:%.2f Z:%.2f\n", forceMovsZero[0], forceMovsZero[1], forceMovsZero[2]);
    }

    std::vector<MovsForceSample> movsForceAll;
    // [MOD-FORCE-LOG-END]

    Dobot::MovSParams params1;
    params1.tool = 0;
    params1.user = 0;
    params1.v = 80;
    params1.a = 80;
    params1.freq = 0.2;

    std::ifstream indexFile(indexFilePath);
    if (!indexFile.is_open())
    {
        std::cerr << "无法打开索引文件: " << indexFilePath << std::endl;
        return -1;
    }

    // 首先读取所有行到vector中，以便知道总行数
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
            if (idx >= 0 && idx < (int)descartesPointsforce.size())
            {
                selectedPoints.push_back(descartesPointsforce[idx]);
            }
            else
            {
                std::cerr << "\n第 " << currentLine << " 行索引 " << idx << " 超出范围 (0-"
                          << descartesPointsforce.size() - 1 << ")" << std::endl;
            }
        }

        if (!selectedPoints.empty())
        {
            // 安全到达
            selectedPoints[0].z += 10;
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

            selectedPoints[0].z -= 10;
            demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
            // std::this_thread::sleep_for(std::chrono::seconds(1));

            // [MOD-FORCE-LOG-BEGIN] 仅在 movsDemoC 执行期间采样力数据（后台线程），其他运动不读
            {
                std::atomic<bool> samplingActive{true};
                std::vector<MovsForceSample> movsChunk;
                const int capLine = currentLine;
                std::thread forceSampler([&]() {
                    int seq = 0;
                    while (samplingActive.load(std::memory_order_acquire))
                    {
                        float raw[6];
                        if (obj->GetCurrentForceData(raw) == 28)
                        {
                            MovsForceSample row{};
                            row.lineNo = capLine;
                            row.seq = seq++;
                            row.fx = raw[0] - forceMovsZero[0];
                            row.fy = raw[1] - forceMovsZero[1];
                            row.fz = raw[2] - forceMovsZero[2];
                            row.mx = raw[3] - forceMovsZero[3];
                            row.my = raw[4] - forceMovsZero[4];
                            row.mz = raw[5] - forceMovsZero[5];
                            movsChunk.push_back(row);
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                });
                demo->movsDemoC(selectedPoints, params1);
                samplingActive.store(false, std::memory_order_release);
                if (forceSampler.joinable())
                    forceSampler.join();
                movsForceAll.insert(movsForceAll.end(), movsChunk.begin(), movsChunk.end());
            }
            // [MOD-FORCE-LOG-END]
            // std::this_thread::sleep_for(std::chrono::seconds(2));

            // 根据是否是最后一行选择不同的动作
            if (!isLastLine)
            {
                // 非最后一行：执行原来的向后移动
                std::cout << "next" << std::endl;
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
            }

            // Dobot::CDescartesPoint rotatetooljoints{};
            // rotatetooljoints.x = 0;
            // rotatetooljoints.y = 0;
            // rotatetooljoints.z = 12;
            // rotatetooljoints.rx = 0;
            // rotatetooljoints.ry = 0;
            // rotatetooljoints.rz = 0;
            // demo->RelMovJDemo(rotatetooljoints, 0, 5, 60, 80, 100);
            // rotatetooljoints.z = 0;
            // rotatetooljoints.ry = -10;
            // demo->RelMovJDemo(rotatetooljoints, 0, 5, 60, 80, 100);
            // demo->RelMovJDemo(rotatetooljoints, 0, 5, 60, 80, 100);
            // demo->RelMovJDemo(rotatetooljoints, 0, 5, 60, 80, 100);
            // demo->RelMovJDemo(rotatetooljoints, 0, 5, 60, 80, 100);
            // rotatetooljoints.ry = 0;
            // rotatetooljoints.x = -12;
            // demo->RelMovJDemo(rotatetooljoints, 0, 5, 60, 80, 100);
            // std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    // [MOD-FORCE-LOG-BEGIN] movs段力数据落盘
    // {
    //     std::ofstream fm(Movs_FORCE_LOG_PATH);
    //     if (fm.is_open())
    //     {
    //         fm << "# lineNo seq fx fy fz mx my mz (相对 movs 段清零基准)\n";
    //         for (const auto &r : movsForceAll)
    //         {
    //             fm << r.lineNo << " " << r.seq << " "
    //                << r.fx << " " << r.fy << " " << r.fz << " "
    //                << r.mx << " " << r.my << " " << r.mz << "\n";
    //         }
    //         fm.close();
    //         std::cout << "movs 段力数据已保存: " << Movs_FORCE_LOG_PATH
    //                   << " 共 " << movsForceAll.size() << " 条采样\n";
    //     }
    //     else
    //         std::cerr << "无法写入 movs 段力数据: " << Movs_FORCE_LOG_PATH << std::endl;
    // }
    // [MOD-FORCE-LOG-END]

    // 基于当前点往上抬
    Dobot::CDescartesPoint rotatetooljointup{};
    rotatetooljointup.x = 0;
    rotatetooljointup.y = 0;
    rotatetooljointup.z = -60;
    rotatetooljointup.rx = 0;
    rotatetooljointup.ry = 0;
    rotatetooljointup.rz = 0;

    demo->RelMovJDemo(rotatetooljointup, 0, 0, 20, 50, 100);

    indexFile.close();

    // 退出
    demo->moveRobotC(pointsafe, pointsafe);

    std::cout << "正在退出程序請稍後：-）" << std::endl;
    obj->StopCapture();
    demo->~DobotTcpDemo();

    delete demo;

    obj = nullptr;
    demo = nullptr;

    return 0;
}
