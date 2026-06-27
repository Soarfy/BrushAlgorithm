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
#include <functional>
#include "DobotTcpDemo.h"
#include "TcpConfigHelper.h"
#include <windows.h>
#include "kw-lib-all.h"
#include <string>
#include "nlohmann/json.hpp"
#include "ForceTrajectoryIO.h"
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

/* ======================= 拖拽微调函数 ======================= */
void dragTuneXYZ(DobotTcpDemo *demo, Dobot::CDescartesPoint &curPose,
                 Eigen::Vector3d &totalOffset)
{
    const double savedRx = curPose.rx;
    const double savedRy = curPose.ry;
    const double savedRz = curPose.rz;

    // 轨迹点为法兰(tool0)位姿，必须用法兰位移(不能用tool5刷尖位移)
    double bx = 0, by = 0, bz = 0, brx = 0, bry = 0, brz = 0;
    while (!demo->getCurrentPose(0, 0, bx, by, bz, brx, bry, brz))
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 进入拖拽前等待到位稳定
    demo->startDrag();
    std::cout << "\n===== 拖拽微调模式 =====\n"
              << "机械臂已进入拖拽模式：请手动拖动机械臂，使刷尖到达目标位置。\n"
              << "(拖拽时姿态会变化，确认后恢复为原rx/ry/rz，仅采用法兰xyz位移)\n"
              << "完成后按 Enter 确认...\n";
    while (true)
    {
        if (_kbhit())
        {
            if (_getch() == 13)
                break;
        }
        Sleep(10);
    }

    double ax = 0, ay = 0, az = 0, arx = 0, ary = 0, arz = 0;
    while (!demo->getCurrentPose(0, 0, ax, ay, az, arx, ary, arz))
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    demo->stopDrag();
    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 等待退出拖拽模式稳定

    double dx = ax - bx;
    double dy = ay - by;
    double dz = az - bz;
    totalOffset += Eigen::Vector3d(dx, dy, dz);

    curPose.x += dx;
    curPose.y += dy;
    curPose.z += dz;
    curPose.rx = savedRx;
    curPose.ry = savedRy;
    curPose.rz = savedRz;
    demo->moveRobotC(curPose, curPose);

    std::cout << "拖拽位移(法兰, base系)[mm]: " << dx << ", " << dy << ", " << dz << std::endl;
    std::cout << "累计偏移[mm]: " << totalOffset.transpose() << std::endl;
}

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

    return vB;
}

/* ======================= 资源清理守卫 ======================= */
// 析构时统一释放设备资源(停止采集 + 释放机械臂句柄)，覆盖所有 return 退出路径
struct ScopeExit
{
    std::function<void()> fn;
    ~ScopeExit()
    {
        if (fn)
            fn();
    }
};

/* ======================= 运行模式 ======================= */
// 1=基础轨迹配置模式  2=随机模式(保留全部交互)  3=轨迹复用模式
struct ModePlan
{
    int mode = 2;
    bool gotoCalib = false;
    bool generateNew = false;
    bool saveStdJson = false;
};

static ModePlan makePlan(int mode)
{
    ModePlan p;
    p.mode = mode;
    if (mode == 1)
    {
        p.gotoCalib = true;
        p.generateNew = true;
        p.saveStdJson = true;
    }
    else if (mode == 3)
    {
        p.gotoCalib = true;
        p.generateNew = false;
        p.saveStdJson = false;
    }
    return p;
}

static bool saveStandardTrajectoryJson(const std::string &path, const std::vector<PointData> &pts)
{
    json j = json::array();
    for (const auto &p : pts)
    {
        j.push_back({{"x", p.x}, {"y", p.y}, {"z", p.z}, {"a", p.a}, {"b", p.b}, {"c", p.c}});
    }
    std::ofstream f(path);
    if (!f.is_open())
        return false;
    f << j.dump(4);
    f.close();
    return true;
}

static bool loadStandardTrajectoryJson(const std::string &path, std::vector<PointData> &pts)
{
    std::ifstream f(path);
    if (!f.is_open())
        return false;
    json j;
    f >> j;
    f.close();
    pts.clear();
    for (const auto &e : j)
    {
        PointData p{};
        p.x = e.at("x").get<double>();
        p.y = e.at("y").get<double>();
        p.z = e.at("z").get<double>();
        p.a = e.at("a").get<double>();
        p.b = e.at("b").get<double>();
        p.c = e.at("c").get<double>();
        pts.push_back(p);
    }
    return true;
}

int main()
{
    // @@@@@@@@@@@@@@@@@@@@@@@@區別代碼@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    const std::string Force_FILE_PATH = "../defaultconfig/rightinside/insideright.txt";
    // const std::string Brush_offset = "../defaultconfig/rightinside/brushoffsets.json";

    const std::string Movs_FORCE_LOG_PATHs = "../defaultconfig/rightinside/movs_force_during_movs.txt";
    std::ofstream forcerepaired(Movs_FORCE_LOG_PATHs);
    if (!forcerepaired.is_open())
    {
        std::cerr << "无法保存含有力控的路径" << std::endl;
        return -1;
    }

    const std::string Brush_offset = "../defaultconfig/brushoffsets.json";
    const std::string Brush_offset_path = "../defaultconfig/rightinside/brushoffsets_path.json";
    const std::string Brush_Config = "../defaultconfig/config.json";
    int value = 4;

    std::string command1 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\GenerateAnyPath.py " +
                           std::to_string(value) + "\"";

    std::string command2 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\GenerateAnyPathRepeat.py " +
                           std::to_string(value) + "\"";

    std::string command3 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\TCPRotation10OthersDorobot.py " +
                           std::to_string(value) + "\"";

    const std::string oldsegment = "../defaultconfig/rightinside/all_segments.txt";
    std::string indexFilePath = "../defaultconfig/rightinside/all_segments.txt";
    std::string indexFilePath2 = "../defaultconfig/rightinside/support_points.txt";
    std::ofstream poseFile("../defaultconfig/rightinside/current_pose_from_getpose.txt");
    const std::string eepath = "../defaultconfig/rightinside/ee_poses.txt";

    Dobot::CDescartesPoint rotatetooljoint{};
    rotatetooljoint.x = 0.0;
    rotatetooljoint.y = 0;
    rotatetooljoint.z = 0;
    rotatetooljoint.rx = -10;
    rotatetooljoint.ry = 45;
    rotatetooljoint.rz = 0;

    Dobot::CDescartesPoint rotatetooljointjump{};
    rotatetooljointjump.x = 0;
    rotatetooljointjump.y = 0;
    rotatetooljointjump.z = -50;
    rotatetooljointjump.rx = 0;
    rotatetooljointjump.ry = 0;
    rotatetooljointjump.rz = 0;

    double modifiedup = 0;
    double modifiedupx = -6.9142260000002125 + 14.163013999999862;
    double modifiedupy = -2.0350259999999025 - 24.190834000000166;
    double modifiedupz = 101.3919;

    double modifyoffsetX = 0;
    double modifyoffsetY = 0;
    double modifyoffsetZ = 0;

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

    // 资源守卫：任何 return 路径都会停止力采集并释放 demo，避免泄漏/重复释放
    ScopeExit deviceGuard{[&]()
                          {
                              if (obj)
                                  obj->StopCapture();
                              if (demo)
                              {
                                  delete demo;
                                  demo = nullptr;
                              }
                          }};

    // 任意点回到起始点
    demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
    Dobot::CDescartesPoint startfirst{};
    startfirst.x = 272.1420;
    startfirst.y = -311.4110;
    startfirst.z = 636.5964;
    startfirst.rx = -179.7730;
    startfirst.ry = -1.3510;
    startfirst.rz = -145.9050;
    demo->moveRobotC(startfirst, startfirst);

    std::cout << "\n=========== 运行模式选择 ===========\n"
              << "1 = 基础轨迹配置模式 (到位不标定 / 不微调牙刷 / 生成新标准轨迹并存json / 微调+力控)\n"
              << "2 = 随机模式 (保留全部交互提示, 原始流程)\n"
              << "3 = 轨迹复用模式 (到位不标定 / 复用模式1的json标准轨迹 / 微调+力控)\n"
              << "请选择(1/2/3): ";
    int mode = 2;
    std::cin >> mode;
    if (std::cin.fail())
    {
        std::cin.clear();
        std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
    }
    if (mode != 1 && mode != 2 && mode != 3)
    {
        std::cout << "无效模式输入, 默认使用 2 (随机模式)\n";
        mode = 2;
    }
    ModePlan plan = makePlan(mode);
    std::cout << "已选择运行模式: " << mode << std::endl;

    std::vector<PointData> brushpointsoffset_ee_poses;

    Dobot::CDescartesPoint pointsafe{};
    pointsafe.x = 264.8929;
    pointsafe.y = -285.1852;
    pointsafe.z = 491.0669;
    pointsafe.rx = -179.7725;
    pointsafe.ry = -1.3507;
    pointsafe.rz = -145.9055;

    if (mode == 2)
    {
    std::cout << "是否使用原有轨迹直接进行力控调整？(y/n): ";
    char useExistingTrajChoice;
    std::cin >> useExistingTrajChoice;
    const bool useExistingTrajectoryForForce =
        (useExistingTrajChoice == 'y' || useExistingTrajChoice == 'Y');

    if (useExistingTrajectoryForForce)
    {
        std::cout << "使用原有轨迹，跳过刷头/轨迹调整，直接进入力控流程..." << std::endl;

        std::ifstream inputFile(Brush_offset);
        if (!inputFile.is_open())
        {
            std::cerr << "无法打开文件进行读取：" << Brush_offset << std::endl;
            return -1;
        }
        json loadedJson;
        inputFile >> loadedJson;
        inputFile.close();

        double offsetXs = loadedJson.value("brushxoffsets", 0.0);
        double offsetYs = loadedJson.value("brushyoffsets", 0.0);
        double offsetZs = loadedJson.value("brushzoffsets", 0.0);

        TcpXyz brushTcp = loadDefaultBrushTcp();


        double tcpx = brushTcp.x;


        double tcpy = brushTcp.y;


        double tcpz = brushTcp.z;

        std::string tcpvalue = "{" + std::to_string(tcpx) + "," +
                               std::to_string(tcpy) + "," +
                               std::to_string(tcpz) + ",0,0,0}";
        demo->setToolDemo(5, tcpvalue);

        backupForceTrajectoryFile(Force_FILE_PATH);
        if (!loadForceTrajectoryFile(Force_FILE_PATH, brushpointsoffset_ee_poses))
        {
            std::cerr << "无法打开或解析原有轨迹：" << Force_FILE_PATH << std::endl;
            return -1;
        }
        std::cout << "已加载原有轨迹 " << brushpointsoffset_ee_poses.size() << " 点" << std::endl;

        demo->moveRobotC(pointsafe, pointsafe);
        std::cout << "初始位：先上抬再旋转..." << std::endl;
        demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
        demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);
    }
    else
    {
    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@回到初始态@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    demo->moveRobotC(pointsafe, pointsafe);

    Dobot::CDescartesPoint pointa{};
    pointa.x = 264.8929 + modifiedupx;
    pointa.y = -285.1852 + modifiedupy;
    pointa.z = 391.0669 + modifiedup + modifiedupz;
    pointa.rx = -179.7725;
    pointa.ry = -1.3507;
    pointa.rz = -145.9055;
    demo->moveRobotC(pointa, pointa);

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@微調牙刷@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    const double REF_X = 264.8929 + modifiedupx;
    const double REF_Y = -285.1852 + modifiedupy;
    const double REF_Z = 391.0669 + modifiedup + modifiedupz;
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

        TcpXyz brushTcp = loadDefaultBrushTcp();


        double tcpx = brushTcp.x;


        double tcpy = brushTcp.y;


        double tcpz = brushTcp.z;
        double tcprx = 0.0;
        double tcpry = 0.0;
        double tcprz = 0.0;
        // tcpx = -7.13;
        // tcpy = -124.073;
        // tcpz = 186.693;
        // tcprx = 0.0;
        // tcpry = 0.0;
        // tcprz = 0.0;

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

            const double offsetX = loadedJson.value("brushxoffset", 0.0);
            const double offsetY = loadedJson.value("brushyoffset", 0.0);
            const double offsetZ = loadedJson.value("brushzoffset", 0.0);

            const double offsetXs = loadedJson.value("brushxoffsets", 0.0);
            const double offsetYs = loadedJson.value("brushyoffsets", 0.0);
            const double offsetZs = loadedJson.value("brushzoffsets", 0.0);

            if (loadedJson.contains("brushxoffsetmodify") &&
                loadedJson.contains("brushyoffsetmodify") &&
                loadedJson.contains("brushzoffsetmodify"))
            {
                modifyoffsetX = loadedJson["brushxoffsetmodify"];
                modifyoffsetY = loadedJson["brushyoffsetmodify"];
                modifyoffsetZ = loadedJson["brushzoffsetmodify"];
            }

            Dobot::CDescartesPoint pointaLoaded{};
            pointaLoaded.x = 264.8929 + modifiedupx + offsetX;
            pointaLoaded.y = -285.1852 + modifiedupy + offsetY;
            pointaLoaded.z = 391.0669 + modifiedupz + modifiedup + offsetZ;
            pointaLoaded.rx = -179.7725;
            pointaLoaded.ry = -1.3507;
            pointaLoaded.rz = -145.9055;
            demo->moveRobotC(pointaLoaded, pointaLoaded);
            std::cout << "机械臂到达起始点" << std::endl;
            std::cout << "跳過牙刷微調，使用已保存偏移。" << std::endl;

            const TcpXyz brushTcp = loadDefaultBrushTcp();


            const double tcpx = brushTcp.x;


            const double tcpy = brushTcp.y;


            const double tcpz = brushTcp.z;

            const std::string tcpvalue = "{" + std::to_string(tcpx) + "," +
                                           std::to_string(tcpy) + "," +
                                           std::to_string(tcpz) + ",0,0,0}";
            demo->setToolDemo(5, tcpvalue);
            std::cout << "已加载 TCP: " << tcpvalue << std::endl;
        }
        else
        {
            std::cerr << "无法打开 " << Brush_offset << "，使用默认 TCP。" << std::endl;
            demo->setToolDemo(5, makeTcpValueString(loadDefaultBrushTcp().x, loadDefaultBrushTcp().y, loadDefaultBrushTcp().z));
            std::cout << "跳過牙刷微調。" << std::endl;
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

    // std::ifstream ee_poses_infile(eepath);
    // std::vector<PointData> brushpointsoffset_ee_poses;
    // if (!ee_poses_infile.is_open())
    // {
    //     std::cerr << "无法打开 ee_poses.txt" << std::endl;
    //     return -1;
    // }
    // double dx, dy, dz, rx, ry, rz;
    // while (ee_poses_infile >> dx >> dy >> dz >> rx >> ry >> rz)
    // {
    //     brushpointsoffset_ee_poses.push_back({dx, dy, dz, rx, ry, rz});
    // }
    // ee_poses_infile.close();
    // if (brushpointsoffset_ee_poses.empty())
    // {
    //     std::cerr << "轨迹为空！" << std::endl;
    //     return -1;
    // }

    // 询问用户是否沿用原来的轨迹
    std::cout << "是否沿用原来的轨迹？(y/n): ";
    char user_choice;
    std::cin >> user_choice;

    std::string trajectoryLoadPath = eepath;
    if (user_choice == 'y' || user_choice == 'Y')
    {
        trajectoryLoadPath = Force_FILE_PATH;
        backupForceTrajectoryFile(Force_FILE_PATH);
    }
    else if (user_choice == 'n' || user_choice == 'N')
    {
        trajectoryLoadPath = eepath;
    }
    else
    {
        std::cerr << "无效输入，默认使用新生成轨迹" << std::endl;
        trajectoryLoadPath = eepath;
    }

    if (!loadForceTrajectoryFile(trajectoryLoadPath, brushpointsoffset_ee_poses))
    {
        std::cerr << "无法打开或解析轨迹文件: " << trajectoryLoadPath << std::endl;
        return -1;
    }
    std::cout << "已加载轨迹 " << brushpointsoffset_ee_poses.size() << " 点" << std::endl;

    if (brushpointsoffset_ee_poses.empty())
    {
        std::cerr << "轨迹为空！" << std::endl;
        return -1;
    }

    // while (!userSatisfied)
    // {
    //     std::cout << "\n=========== 新一轮轨迹调整开始 ===========\n";

    //     Dobot::CDescartesPoint firstPose{};
    //     firstPose.x = brushpointsoffset_ee_poses[0].x;
    //     firstPose.y = brushpointsoffset_ee_poses[0].y;
    //     firstPose.z = brushpointsoffset_ee_poses[0].z;
    //     firstPose.rx = brushpointsoffset_ee_poses[0].a;
    //     firstPose.ry = brushpointsoffset_ee_poses[0].b;
    //     firstPose.rz = brushpointsoffset_ee_poses[0].c;
    //     demo->moveRobotC(pointsafe, pointsafe);
    //     demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

    //     // 根據軌跡向量來補償
    //     Eigen::Matrix3d rotationMatrixs = eulerDegToRotationMatrix(firstPose.rx, firstPose.ry, firstPose.rz);
    //     Eigen::Vector3d brushDirs = rotationMatrixs.col(2);
    //     brushDirs.normalize();

    //     Dobot::CDescartesPoint pointstart{};
    //     pointstart.x = firstPose.x + -brushDirs.x() * 8;
    //     pointstart.y = firstPose.y + -brushDirs.y() * 8;
    //     pointstart.z = firstPose.z + -brushDirs.z() * 8;
    //     pointstart.rx = firstPose.rx;
    //     pointstart.ry = firstPose.ry;
    //     pointstart.rz = firstPose.rz;
    //     demo->moveRobotC(pointstart, pointstart);
    //     // 從 offsetpath.json 讀取 deltaOffset 初始值
    //     Eigen::Vector3d deltaOffset(0, 0, 0);

    //     // 檢查檔案是否存在，若存在則讀取
    //     // std::ifstream file(Brush_offset_path);
    //     // if (file.good())
    //     // {
    //     //     nlohmann::json j;
    //     //     file >> j;
    //     //     deltaOffset[0] = j["offsetpath_x"];
    //     //     deltaOffset[1] = j["offsetpath_y"];
    //     //     deltaOffset[2] = j["offsetpath_z"];
    //     // }
    //     // 若檔案不存在，則保持預設的 (0, 0, 0)
    //     // firstPose.x += deltaOffset[0];
    //     // firstPose.y += deltaOffset[1];
    //     // firstPose.z += deltaOffset[2];

    //     demo->moveRobotC(firstPose, firstPose);

    //     // std::cout << "机械臂到达起始点，请确认按Enter" << std::endl;
    //     // std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    //     // 執行 fineTuneXYZ 進行調整
    //     fineTuneXYZ(demo, firstPose, deltaOffset);

    //     // 調整完成後，將更新後的 deltaOffset 存回 JSON
    //     // {
    //     //     nlohmann::json j;
    //     //     j["offsetpath_x"] = deltaOffset[0];
    //     //     j["offsetpath_y"] = deltaOffset[1];
    //     //     j["offsetpath_z"] = deltaOffset[2];

    //     //     std::ofstream out_file(Brush_offset_path);
    //     //     out_file << j.dump(4);
    //     // }

    //     for (auto &p : brushpointsoffset_ee_poses)
    //     {
    //         p.x += deltaOffset.x();
    //         p.y += deltaOffset.y();
    //         p.z += deltaOffset.z();
    //     }

    //     std::vector<Dobot::CDescartesPoint> descartesPoints;
    //     for (const auto &p : brushpointsoffset_ee_poses)
    //     {
    //         Dobot::CDescartesPoint cp{};
    //         cp.x = p.x;
    //         cp.y = p.y;
    //         cp.z = p.z;
    //         cp.rx = p.a;
    //         cp.ry = p.b;
    //         cp.rz = p.c;
    //         descartesPoints.push_back(cp);
    //     }

    //     Dobot::MovSParams params;
    //     params.tool = 0;
    //     params.user = 0;
    //     params.v = 80;
    //     params.a = 80;
    //     // params.speed = brushSpeed;
    //     params.freq = 0.2;
    //     demo->movsDemoC(descartesPoints, params);

    //     std::cout << "\n是否满意当前调整后的轨迹？(y/n): ";
    //     char choice;
    //     std::cin >> choice;
    //     std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    //     demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);

    //     if (choice == 'y' || choice == 'Y')
    //     {
    //         userSatisfied = true;
    //         std::cout << "调整完成 ✅\n";
    //     }
    //     else
    //     {
    //         std::cout << "继续调整...\n";
    //     }
    // }

    bool isFirstRun = true;
    Eigen::Vector3d totalDeltaOffset(0, 0, 0); // 使用独立的累积偏移变量

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
        std::cout << "初始位：先上抬再旋转，前往轨迹起点..." << std::endl;
        demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
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

        Eigen::Vector3d deltaOffset(0, 0, 0); // 本次循环的偏移量
        demo->moveRobotC(firstPose, firstPose);

        // 判断是否是第一次运行
        if (isFirstRun && (user_choice == 'y' || user_choice == 'Y'))
        {
            // 第一次运行，自动模拟按两次'n'键的效果
            std::cout << "\n第一次运行，自动进行初始调整（按两次n键）...\n";

            // 获取当前位姿的局部坐标系方向
            Eigen::Matrix3d rotationMatrixs_local = eulerDegToRotationMatrix(firstPose.rx, firstPose.ry, firstPose.rz);
            Eigen::Vector3d brushDirsx_local = rotationMatrixs_local.col(0);
            brushDirsx_local.normalize();

            // 模拟两次'n'键：每次沿-X方向移动
            for (int i = 0; i < 2; i++)
            {
                double dx = -brushDirsx_local.x();
                double dy = -brushDirsx_local.y();
                double dz = -brushDirsx_local.z();

                deltaOffset += Eigen::Vector3d(dx, dy, dz); // 累加到本次循环的偏移
                firstPose.x += dx;
                firstPose.y += dy;
                firstPose.z += dz;
                demo->moveRobotC(firstPose, firstPose);

                std::cout << "自动调整 " << (i + 1) << "/2 - 本次偏移: [" << dx << ", " << dy << ", " << dz << "]"
                          << " 累计本次偏移 [mm]: " << deltaOffset.transpose() << std::endl;
                Sleep(500); // 等待500ms，让操作更可见
            }

            isFirstRun = false; // 标记已经不是第一次运行了
        }
        else
        {
            // 非第一次运行，正常调用交互式微调
            fineTuneXYZ(demo, firstPose, deltaOffset);
        }

        // 将本次循环的偏移累加到总偏移量
        totalDeltaOffset += deltaOffset;

        std::cout << "本次循环偏移量: " << deltaOffset.transpose() << std::endl;
        std::cout << "历史总偏移量: " << totalDeltaOffset.transpose() << std::endl;

        // 更新所有轨迹点
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

        {
            Dobot::CDescartesPoint lifted = descartesPoints.back();
            lifted.z += 100; // 基坐标系 Z 上抬 100mm
            demo->moveRobotC(lifted, lifted);
        }

        std::cout << "\n是否满意当前调整后的轨迹？(y/n): ";
        char choice;
        std::cin >> choice;
        std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

        if (choice == 'y' || choice == 'Y')
        {
            userSatisfied = true;
            std::cout << "调整完成 ✅\n";
            std::cout << "最终总偏移量: " << totalDeltaOffset.transpose() << std::endl;
        }
        else
        {
            std::cout << "继续调整...\n";
            // 注意：如果不是第一次运行，下次循环会继续使用交互式微调
        }
    }

    demo->moveRobotC(pointsafe, pointsafe);
    std::cout << "初始位：先上抬再旋转，前往轨迹起点..." << std::endl;
    demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
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

    } // useExistingTrajectoryForForce
    } // if (mode == 2)
    else
    {
        // ================= 模式1 / 模式3 精简自动流程 =================
        const std::string Std_Traj_Json = "../defaultconfig/rightinside/standard_trajectory.json";
        const std::string Std_Traj_Json_NoComp = "../defaultconfig/rightinside/standard_trajectory_nocomp.json";

        // ---- 加载已有牙刷偏移并设置TCP(不做TCP标定/不做牙刷微调) ----
        {
            std::ifstream inputFile(Brush_offset);
            if (!inputFile.is_open())
            {
                std::cerr << "无法打开牙刷偏移文件: " << Brush_offset << std::endl;
                return -1;
            }
            json loadedJson;
            inputFile >> loadedJson;
            inputFile.close();

            double offsetXs = loadedJson.value("brushxoffsets", 0.0);
            double offsetYs = loadedJson.value("brushyoffsets", 0.0);
            double offsetZs = loadedJson.value("brushzoffsets", 0.0);

            TcpXyz brushTcp = loadDefaultBrushTcp();


            double tcpx = brushTcp.x;


            double tcpy = brushTcp.y;


            double tcpz = brushTcp.z;
            std::string tcpvalue = "{" + std::to_string(tcpx) + "," +
                                   std::to_string(tcpy) + "," +
                                   std::to_string(tcpz) + ",0,0,0}";
            demo->setToolDemo(5, tcpvalue);
            std::cout << "[模式" << mode << "] 已加载牙刷偏移并设置TCP: " << tcpvalue << std::endl;
        }

        std::cout << "[模式" << mode << "] 前往工作位置(不做TCP标定), 注意安全..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        demo->moveRobotC(pointsafe, pointsafe);

        Dobot::CDescartesPoint pointa{};
        pointa.x = 264.8929 + modifiedupx;
        pointa.y = -285.1852 + modifiedupy;
        pointa.z = 391.0669 + modifiedup + modifiedupz;
        pointa.rx = -179.7725;
        pointa.ry = -1.3507;
        pointa.rz = -145.9055;
        demo->moveRobotC(pointa, pointa);

        if (plan.generateNew)
        {
            // ---- 模式1: 生成新标准轨迹 ----
            std::cout << "[模式1] 生成新轨迹..." << std::endl;
            int rc = std::system(command1.c_str());
            if (rc != 0)
            {
                std::cout << "[模式1] 新轨迹生成失败！" << std::endl;
                return -1;
            }
            std::cout << "[模式1] 新轨迹生成成功！" << std::endl;

            // ---- 模式1: 牙刷初始旋转姿态调整(自动, 去Enter, 加延时+提示) ----
            std::cout << "[模式1] 牙刷初始旋转姿态调整(自动), 即将上抬并旋转, 注意安全..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
            demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);
            std::cout << "[模式1] 旋转完成, 即将下降..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            double gxdown, gydown, gzdown, grxdown, grydown, grzdown;
            while (!demo->getCurrentPose(0, 0, gxdown, gydown, gzdown, grxdown, grydown, grzdown))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(60));
            }
            Dobot::CDescartesPoint firstPoseback{};
            firstPoseback.x = gxdown;
            firstPoseback.y = gydown;
            firstPoseback.z = gzdown - 50;
            firstPoseback.rx = grxdown;
            firstPoseback.ry = grydown;
            firstPoseback.rz = grzdown;
            demo->moveRobotC(firstPoseback, firstPoseback);
            std::cout << "[模式1] 牙刷初始姿态已调整, 即将记录位姿..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            double gx, gy, gz, grx, gry, grz;
            while (true)
            {
                if (demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz) &&
                    !std::isnan(gx) && !std::isnan(gy) && !std::isnan(gz) &&
                    !std::isnan(grx) && !std::isnan(gry) && !std::isnan(grz))
                {
                    if (poseFile.is_open())
                    {
                        poseFile << gx << " " << gy << " " << gz << " "
                                 << grx << " " << gry << " " << grz << std::endl;
                        poseFile.close();
                        std::cout << "[模式1] 当前刷头位置已保存\n";
                    }
                    else
                    {
                        std::cerr << "[模式1] 当前刷头位置保存失败\n";
                    }
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(60));
            }

            std::cout << "[模式1] 将轨迹转换到机械臂末端..." << std::endl;
            std::system(command3.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            firstPoseback.z += 20;
            demo->moveRobotC(firstPoseback, firstPoseback);

            backupForceTrajectoryFile(Force_FILE_PATH);
            if (!loadForceTrajectoryFile(eepath, brushpointsoffset_ee_poses))
            {
                std::cerr << "无法加载新生成轨迹: " << eepath << std::endl;
                return -1;
            }
        }
        else
        {
            // ---- 模式3: 复用模式1保存的 json 标准轨迹 ----
            std::cout << "[模式3] 选择复用的标准轨迹: 1=有补偿(微调后)  2=无补偿(原始未微调)\n请选择(1/2): ";
            int stdTrajSel = 1;
            std::cin >> stdTrajSel;
            if (std::cin.fail())
            {
                std::cin.clear();
                std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
            }
            const std::string chosenStdTraj = (stdTrajSel == 2) ? Std_Traj_Json_NoComp : Std_Traj_Json;
            std::cout << "[模式3] 复用标准轨迹(json): " << chosenStdTraj << std::endl;
            if (!loadStandardTrajectoryJson(chosenStdTraj, brushpointsoffset_ee_poses))
            {
                std::cerr << "无法加载标准轨迹json(请先用模式1生成): " << chosenStdTraj << std::endl;
                return -1;
            }
            backupForceTrajectoryFile(Force_FILE_PATH);
            indexFilePath = oldsegment;
        }

        std::cout << "已加载轨迹 " << brushpointsoffset_ee_poses.size() << " 点" << std::endl;
        if (brushpointsoffset_ee_poses.empty())
        {
            std::cerr << "轨迹为空！" << std::endl;
            return -1;
        }

        // ---- 模式1: 保存"无补偿"(原始未微调)标准轨迹为 json ----
        if (plan.saveStdJson)
        {
            if (saveStandardTrajectoryJson(Std_Traj_Json_NoComp, brushpointsoffset_ee_poses))
                std::cout << "[模式1] 无补偿标准轨迹(原始未微调)已保存为json: " << Std_Traj_Json_NoComp
                          << " (" << brushpointsoffset_ee_poses.size() << " 点)" << std::endl;
            else
                std::cerr << "[模式1] 无补偿标准轨迹json保存失败: " << Std_Traj_Json_NoComp << std::endl;
        }

        // ---- 轨迹微调循环(模式1/3都做) ----
        double offsetX = 0, offsetY = 0, offsetZ = 0;
        double offsetXs = 0, offsetYs = 0, offsetZs = 0;
        {
            std::ifstream inputFile(Brush_offset);
            if (inputFile.is_open())
            {
                json loadedJson;
                inputFile >> loadedJson;
                inputFile.close();
                offsetX = loadedJson.value("brushxoffset", 0.0);
                offsetY = loadedJson.value("brushyoffset", 0.0);
                offsetZ = loadedJson.value("brushzoffset", 0.0);
                offsetXs = loadedJson.value("brushxoffsets", 0.0);
                offsetYs = loadedJson.value("brushyoffsets", 0.0);
                offsetZs = loadedJson.value("brushzoffsets", 0.0);
            }
        }
        double modifyoffsetX = 0, modifyoffsetY = 0, modifyoffsetZ = 0;

        bool userSatisfied = false;
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
            std::cout << "初始位：先上抬再旋转，前往轨迹起点..." << std::endl;
            demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
            demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

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
            Eigen::Vector3d deltaOffset(0, 0, 0);
            demo->moveRobotC(firstPose, firstPose);
            char tuneSel = 'k';
            std::cout << "\n选择微调方式: k=键盘微调  d=拖拽微调，请输入后回车: ";
            std::cin >> tuneSel;
            std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
            if (tuneSel == 'd' || tuneSel == 'D')
                dragTuneXYZ(demo, firstPose, deltaOffset);
            else
                fineTuneXYZ(demo, firstPose, deltaOffset);
            for (auto &p : brushpointsoffset_ee_poses)
            {
                p.x += deltaOffset.x();
                p.y += deltaOffset.y();
                p.z += deltaOffset.z();
            }

            modifyoffsetX += deltaOffset.x();
            modifyoffsetY += deltaOffset.y();
            modifyoffsetZ += deltaOffset.z();
            offsetX += deltaOffset.x();
            offsetY += deltaOffset.y();
            offsetZ += deltaOffset.z();

            Vector3 vecA = {deltaOffset.x(), deltaOffset.y(), deltaOffset.z()};
            double gxc, gyc, gzc, grxc, gryc, grzc;
            while (!demo->getCurrentPose(0, 0, gxc, gyc, gzc, grxc, gryc, grzc))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            Vector3 vecB = transformVectorAToB(vecA, grxc, gryc, grzc);
            offsetXs += vecB.x;
            offsetYs += vecB.y;
            offsetZs += vecB.z;

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
            params.freq = 0.2;
            demo->movsDemoC(descartesPoints, params);

            {
                Dobot::CDescartesPoint lifted = descartesPoints.back();
                lifted.z += 100; // 基坐标系 Z 上抬 100mm
                demo->moveRobotC(lifted, lifted);
            }

            std::cout << "\n是否满意当前调整后的轨迹？(y/n): ";
            char choice;
            std::cin >> choice;
            std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

            if (choice == 'y' || choice == 'Y')
            {
                userSatisfied = true;
                std::cout << "调整完成 ✅\n";

                json offsetJson;
                offsetJson["brushxoffsets"] = offsetXs;
                offsetJson["brushyoffsets"] = offsetYs;
                offsetJson["brushzoffsets"] = offsetZs;
                offsetJson["brushxoffset"] = offsetX;
                offsetJson["brushyoffset"] = offsetY;
                offsetJson["brushzoffset"] = offsetZ;
                offsetJson["brushxoffsetmodify"] = modifyoffsetX;
                offsetJson["brushyoffsetmodify"] = modifyoffsetY;
                offsetJson["brushzoffsetmodify"] = modifyoffsetZ;
                std::ofstream outputFile(Brush_offset);
                if (outputFile.is_open())
                {
                    outputFile << offsetJson.dump(4);
                    outputFile.close();
                    std::cout << "偏移量已保存到: " << Brush_offset << std::endl;
                }
            }
            else
            {
                std::cout << "继续调整...\n";
            }
        }

        // ---- 模式1: 保存标准轨迹为 json (供模式3复用/其它代码参考) ----
        if (plan.saveStdJson)
        {
            if (saveStandardTrajectoryJson(Std_Traj_Json, brushpointsoffset_ee_poses))
            {
                std::cout << "[模式1] 标准轨迹已保存为json: " << Std_Traj_Json
                          << " (" << brushpointsoffset_ee_poses.size() << " 点)" << std::endl;
            }
            else
            {
                std::cerr << "[模式1] 标准轨迹json保存失败: " << Std_Traj_Json << std::endl;
            }
        }

        demo->moveRobotC(pointsafe, pointsafe);
        std::cout << "微调结束回到安全点..." << std::endl;
        demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
        demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } // mode 1/3

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@添加力控@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    double targetforcevalue = pressureParameter;
    std::cout << "\n理想的壓力值是 :  " << targetforcevalue << std::endl;
    std::vector<Dobot::CDescartesPoint> descartesPointsforce;
    const std::string forceTrajectoryTempPath = forceTrajectoryTempPathFor(Force_FILE_PATH);
    std::ofstream forcerepairedoutputfile(forceTrajectoryTempPath);
    if (!forcerepairedoutputfile.is_open())
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
        offset.z = descartesPoints[0].z;
        bool converged = false;
        int forceTuneIter = 0;
        const int kMaxForceTuneIter = 200;

        while (!converged)
        {
            if (++forceTuneIter > kMaxForceTuneIter)
            {
                std::cerr << "点 " << i << " 力控迭代超限，保留当前位姿\n";
                converged = true;
                break;
            }
            Eigen::Matrix3d rotationMatrix = eulerDegToRotationMatrix(offset.rx, offset.ry, offset.rz);
            Eigen::Vector3d brushDir = rotationMatrix.col(2);
            brushDir.normalize();

            demo->moveRobotC(offset, offset);
            double gxsa, gysa, gzsa, grxsa, grysa, grzsa;
            double targetxsa, targetysa, targetzsa;
            targetxsa = offset.x;
            targetysa = offset.y;
            targetzsa = offset.z;
            int poseWaitCount = 0;
            const int kMaxPoseWait = 7500;
            while (poseWaitCount < kMaxPoseWait)
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
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                ++poseWaitCount;
            }
            if (poseWaitCount >= kMaxPoseWait)
            {
                std::cerr << "点 " << i << " 到位等待超时，继续力控采样\n";
            }

            float force[6]{};
            int forceReadRetry = 0;
            const int kMaxForceReadRetry = 100;
            while (forceReadRetry < kMaxForceReadRetry)
            {
                if (obj->GetCurrentForceData(force) == 28 || force[2] == 0)
                    break;
                std::cerr << "获取力控数据失败\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                ++forceReadRetry;
            }
            if (forceReadRetry >= kMaxForceReadRetry)
            {
                std::cerr << "点 " << i << " 力控读数超时，保留当前位姿\n";
                converged = true;
                break;
            }
            force[0] -= forcefirst[0];
            force[1] -= forcefirst[1];
            force[2] -= forcefirst[2];

            // std::this_thread::sleep_for(std::chrono::seconds(1));
            // std::cout << firstcount << std::endl;
            // std::cout << "next" << std::endl;
            // std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

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
                    forcerepaired << proj << std::endl;
                    std::cout << "  proj 1= " << proj << std::endl;
                    break;
                }
                else if (err > 0.02)
                {
                    Eigen::Vector3d delta = -0.6 * brushDir;
                    offset.x += delta.x();
                    offset.y += delta.y();
                    // offset.z += delta.z();
                    firstcount -= 1;
                }
                else if (err < -0.02)
                {
                    Eigen::Vector3d delta = 0.6 * brushDir;
                    offset.x += delta.x();
                    offset.y += delta.y();
                    // offset.z += delta.z();
                    firstcount += 1;
                }
            }
            else
            {
                if (std::abs(err) <= 0.02)
                {
                    converged = true;
                    forcerepaired << proj << std::endl;
                    std::cout << "  proj 2= " << proj << std::endl;
                    break;
                }
                else if (err > 0.02)
                {
                    Eigen::Vector3d delta = -0.6 * brushDir;
                    offset.x += delta.x();
                    offset.y += delta.y();
                    // offset.z += delta.z();
                    firstcount -= 1;
                }
                else if (err < -0.02)
                {
                    Eigen::Vector3d delta = 0.6 * brushDir;
                    offset.x += delta.x();
                    offset.y += delta.y();
                    // offset.z += delta.z();
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

    forcerepairedoutputfile.close();
    forcerepaired.close();

    if (descartesPointsforce.size() != brushpointsoffset_ee_poses.size())
    {
        std::error_code ec;
        std::filesystem::remove(forceTrajectoryTempPath, ec);
        std::cerr << "力控点数不完整 (" << descartesPointsforce.size() << "/"
                  << brushpointsoffset_ee_poses.size()
                  << ")，原轨迹未覆盖，备份: " << Force_FILE_PATH << ".bak\n";
        return -1;
    }
    if (!commitForceTrajectoryFile(forceTrajectoryTempPath, Force_FILE_PATH))
    {
        return -1;
    }
    std::cout << "调整后的力控轨迹保存完毕 (" << descartesPointsforce.size() << " 点)" << std::endl;

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

    if (!descartesPointsforce.empty())
    {
        Dobot::CDescartesPoint lifted = descartesPointsforce.back();
        lifted.z += 100; // 基坐标系 Z 上抬 100mm
        demo->moveRobotC(lifted, lifted);
    }

    demo->moveRobotC(pointsafe, pointsafe);
    std::cout << "初始位：先上抬再旋转，前往轨迹起点..." << std::endl;
    demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
    demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

    demo->moveRobotC(pointstartsk, pointstartsk);
    demo->moveRobotC(firstPosesk, firstPosesk);

    std::cout << "运行完整力控轨迹（每行一次movsDemoC）" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    Dobot::MovSParams params1;
    params1.tool = 0;
    params1.user = 0;
    params1.v = 80;
    params1.a = 80;
    params1.freq = 0.2;

    // std::ifstream indexFile(indexFilePath);
    // if (!indexFile.is_open())
    // {
    //     std::cerr << "无法打开索引文件: " << indexFilePath << std::endl;
    //     return -1;
    // }

    // std::string line;
    // int lineNumber = 0;
    // while (std::getline(indexFile, line))
    // {
    //     lineNumber++;
    //     std::istringstream iss(line);
    //     int idx;
    //     std::vector<Dobot::CDescartesPoint> selectedPoints;

    //     while (iss >> idx)
    //     {
    //         if (idx >= 0 && idx < (int)descartesPointsforce.size())
    //         {
    //             selectedPoints.push_back(descartesPointsforce[idx]);
    //         }
    //         else
    //         {
    //             std::cerr << "\n第 " << lineNumber << " 行索引 " << idx << " 超出范围 (0-"
    //                       << descartesPointsforce.size() - 1 << ")" << std::endl;
    //         }
    //     }

    //     if (!selectedPoints.empty())
    //     {
    //         // 安全到达
    //         selectedPoints[0].z += 10;
    //         demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
    //         std::this_thread::sleep_for(std::chrono::seconds(1));

    //         Dobot::CDescartesPoint rotatetooljointjumps{};
    //         rotatetooljointjumps.x = 0;
    //         rotatetooljointjumps.y = 0;
    //         rotatetooljointjumps.z = -10;
    //         rotatetooljointjumps.rx = 0;
    //         rotatetooljointjumps.ry = 0;
    //         rotatetooljointjumps.rz = 0;
    //         demo->RelMovJDemo(rotatetooljointjumps, 0, 5, 20, 50, 100);

    //         selectedPoints[0].z -= 10;
    //         demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
    //         std::this_thread::sleep_for(std::chrono::seconds(1));

    //         demo->movsDemoC(selectedPoints, params1);
    //         std::this_thread::sleep_for(std::chrono::seconds(2));

    //         Dobot::CDescartesPoint rotatetooljointjumpss{};
    //         rotatetooljointjumpss.x = -15;
    //         rotatetooljointjumpss.y = 0;
    //         rotatetooljointjumpss.z = 0;
    //         rotatetooljointjumpss.rx = 0;
    //         rotatetooljointjumpss.ry = 0;
    //         rotatetooljointjumpss.rz = 0;
    //         demo->RelMovJDemo(rotatetooljointjumpss, 0, 5, 20, 50, 100);

    //         // 扶刷
    //         // Dobot::CDescartesPoint rotatetooljoints{};
    //         // rotatetooljoints.x = 0;
    //         // rotatetooljoints.y = 0;
    //         // rotatetooljoints.z = 12;
    //         // rotatetooljoints.rx = 0;
    //         // rotatetooljoints.ry = 0;
    //         // rotatetooljoints.rz = 0;
    //         // demo->RelMovJDemo(rotatetooljoints, 0, 3, 60, 80, 100);
    //         // rotatetooljoints.z = 0;
    //         // rotatetooljoints.ry = -10;
    //         // demo->RelMovJDemo(rotatetooljoints, 0, 3, 60, 80, 100);
    //         // demo->RelMovJDemo(rotatetooljoints, 0, 3, 60, 80, 100);
    //         // demo->RelMovJDemo(rotatetooljoints, 0, 3, 60, 80, 100);
    //         // demo->RelMovJDemo(rotatetooljoints, 0, 3, 60, 80, 100);
    //         // rotatetooljoints.ry = 0;
    //         // rotatetooljoints.x = -12;
    //         // demo->RelMovJDemo(rotatetooljoints, 0, 3, 60, 80, 100);
    //         // std::this_thread::sleep_for(std::chrono::seconds(2));
    //     }

    //     // 最后沿着切向走
    //     Dobot::CDescartesPoint rotatetooljointleave{};
    //     rotatetooljointleave.x = 0;
    //     rotatetooljointleave.y = 10;
    //     rotatetooljointleave.z = 0;
    //     rotatetooljointleave.rx = 0;
    //     rotatetooljointleave.ry = 0;
    //     rotatetooljointleave.rz = 0;
    //     demo->RelMovJDemo(rotatetooljointleave, 0, 5, 20, 50, 100);
    // }

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
            rotatetooljointjumps.x = -30;
            rotatetooljointjumps.y = 0;
            rotatetooljointjumps.z = -30;
            rotatetooljointjumps.rx = 0;
            rotatetooljointjumps.ry = 0;
            rotatetooljointjumps.rz = 0;
            demo->RelMovJDemo(rotatetooljointjumps, 0, 5, 20, 50, 100);

            selectedPoints[0].z -= 10;
            demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
            // std::this_thread::sleep_for(std::chrono::seconds(1));

            demo->movsDemoC(selectedPoints, params1);
            // std::this_thread::sleep_for(std::chrono::seconds(2));

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

    // 走完完整轨迹后：沿机械臂基坐标系上抬80mm
    {
        double gx, gy, gz, grx, gry, grz;
        while (!demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        Dobot::CDescartesPoint lifted{};
        lifted.x = gx;
        lifted.y = gy;
        lifted.z = gz + 80; // 基坐标系 Z 上抬 80mm
        lifted.rx = grx;
        lifted.ry = gry;
        lifted.rz = grz;
        demo->moveRobotC(lifted, lifted);
    }

    indexFile.close();

    // 退出
    demo->moveRobotC(pointsafe, pointsafe);
    demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

    // // 专门浮刷
    std::ifstream indexFiles(indexFilePath2);
    if (!indexFiles.is_open())
    {
        std::cerr << "无法打开点位索引文件: " << indexFilePath2 << std::endl;
        return -1;
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
        return 0;
    }

    // 2. 初始动作：移动到第0个点（初始扶刷准备位）
    // std::cout << "初始定位至起点 (ID: 0)..." << std::endl;
    // demo->moveRobotC(descartesPointsforce[0], descartesPointsforce[0]);
    // std::this_thread::sleep_for(std::chrono::seconds(1));

    // 3. 循环处理每一个 ID
    for (size_t i = 0; i < targetIds.size(); ++i)
    {
        int currentTargetId = targetIds[i];

        // 索引合法性检查
        if (currentTargetId < 0 || currentTargetId >= (int)descartesPointsforce.size())
        {
            std::cerr << "跳过非法索引: " << currentTargetId << std::endl;
            continue;
        }

        std::cout << "--- 正在处理第 " << i + 1 << " 个任务，目标点 ID: " << currentTargetId << " ---" << std::endl;

        descartesPointsforce[currentTargetId].z += 50;
        demo->moveRobotC(descartesPointsforce[currentTargetId], descartesPointsforce[currentTargetId]);
        // std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待稳定

        Dobot::CDescartesPoint rotatetooljointjumps{};
        rotatetooljointjumps.x = -30;
        rotatetooljointjumps.y = 0;
        rotatetooljointjumps.z = -30;
        rotatetooljointjumps.rx = 0;
        rotatetooljointjumps.ry = 0;
        rotatetooljointjumps.rz = 0;
        demo->RelMovJDemo(rotatetooljointjumps, 0, 5, 20, 50, 100);

        descartesPointsforce[currentTargetId].z -= 50;
        demo->moveRobotC(descartesPointsforce[currentTargetId], descartesPointsforce[currentTargetId]);
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        // B. 执行您要求的特殊动作 (向 X 负方向移动 15mm)
        Dobot::CDescartesPoint rotatetooljointjumpss{};
        rotatetooljointjumpss.x = 35;
        rotatetooljointjumpss.y = 0;
        rotatetooljointjumpss.z = 0;
        rotatetooljointjumpss.rx = 0;
        rotatetooljointjumpss.ry = 0;
        rotatetooljointjumpss.rz = 0;

        std::cout << "到达点 " << currentTargetId << "，执行回退补偿动作..." << std::endl;
        demo->RelMovJDemo(rotatetooljointjumpss, 0, 5, 20, 50, 100);

        // C. 动作间隙停顿
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        // 如果读到最后一行，自动结束循环
        if (i == targetIds.size() - 1)
        {
            double gx, gy, gz, grx, gry, grz;
            while (!demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            Dobot::CDescartesPoint lifted{};
            lifted.x = gx;
            lifted.y = gy;
            lifted.z = gz + 100; // 基坐标系 Z 上抬 100mm
            lifted.rx = grx;
            lifted.ry = gry;
            lifted.rz = grz;
            demo->moveRobotC(lifted, lifted);
            std::cout << "已完成最后一行索引，流程结束。" << std::endl;
        }
    }

    // 退出
    demo->moveRobotC(pointsafe, pointsafe);

    std::cout << "正在退出程序請稍後：-）" << std::endl;
    // 设备资源(StopCapture + delete demo)统一由 deviceGuard 在退出时释放
    return 0;
}
