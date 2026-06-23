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
    const std::string Force_FILE_PATH = "../defaultconfig/rightside/sideright.txt";
    // const std::string Brush_offset = "../defaultconfig/rightside/brushoffsets.json";

    const std::string Movs_FORCE_LOG_PATHs = "../defaultconfig/rightside/movs_force_during_movs.txt";
    std::ofstream forcerepaired(Movs_FORCE_LOG_PATHs);
    if (!forcerepaired.is_open())
    {
        std::cerr << "无法保存含有力控的路径" << std::endl;
        return -1;
    }
    const std::string Brush_offset = "../defaultconfig/brushoffsets.json";
    const std::string Brush_offset_path = "../defaultconfig/rightside/brushoffsets_path.json";
    const std::string Brush_Config = "../defaultconfig/config.json";
    int value = 2;

    std::string command1 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\GenerateAnyPath.py " +
                           std::to_string(value) + "\"";

    std::string command2 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\GenerateAnyPathRepeat.py " +
                           std::to_string(value) + "\"";

    std::string command3 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\TCPRotation10OthersDorobot.py " +
                           std::to_string(value) + "\"";

    const std::string oldsegment = "../defaultconfig/rightside/all_segments.txt";
    std::string indexFilePath = "../defaultconfig/rightside/all_segments.txt";
    std::string indexFilePath2 = "../defaultconfig/rightside/support_points.txt";
    std::ofstream poseFile("../defaultconfig/rightside/current_pose_from_getpose.txt");
    const std::string eepath = "../defaultconfig/rightside/ee_poses.txt";

    Dobot::CDescartesPoint rotatetooljoint{};
    rotatetooljoint.x = 0.0;
    rotatetooljoint.y = 0;
    rotatetooljoint.z = 0;
    rotatetooljoint.rx = 0;
    rotatetooljoint.ry = -45;
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

    // double gxdownfirst, gydownfirst, gzdownfirst, grxdownfirst, grydownfirst, grzdownfirst;

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

        double tcpx = -9.748236;
        double tcpy = -186.312977;
        double tcpz = 223.252632;

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

        // 检查 offsetJson 是否包含 "brushxoffsetmodify" 键
        if (offsetJson.contains("brushxoffsetmodify") &&
            offsetJson.contains("brushyoffsetmodify") &&
            offsetJson.contains("brushzoffsetmodify"))
        {
            modifyoffsetX = offsetJson["brushxoffsetmodify"];
            modifyoffsetY = offsetJson["brushyoffsetmodify"];
            modifyoffsetZ = offsetJson["brushzoffsetmodify"];
        }
        else
        {
            std::cout << "警告：offsetJson 中缺少修改偏移量键，跳过加载。" << std::endl;
            // 或者设置默认值
            modifyoffsetX = 0.0;
            modifyoffsetY = 0.0;
            modifyoffsetZ = 0.0;
        }

        offsetJson["brushxoffsets"] = vecB.x;
        offsetJson["brushyoffsets"] = vecB.y;
        offsetJson["brushzoffsets"] = vecB.z;

        std::ofstream file(Brush_offset);
        file << offsetJson.dump(4);
        file.close();

        std::cout << "牙刷微調完成，偏移量已保存。" << std::endl;

        double tcpx = -9.748236;
        double tcpy = -186.312977;
        double tcpz = 223.252632;
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

            double offsetXs = loadedJson.value("brushxoffsets", 0.0);
            double offsetYs = loadedJson.value("brushyoffsets", 0.0);
            double offsetZs = loadedJson.value("brushzoffsets", 0.0);

            if (loadedJson.contains("brushxoffsetmodify") &&
                loadedJson.contains("brushyoffsetmodify") &&
                loadedJson.contains("brushzoffsetmodify"))
            {
                modifyoffsetX = loadedJson["brushxoffsetmodify"];
                modifyoffsetY = loadedJson["brushyoffsetmodify"];
                modifyoffsetZ = loadedJson["brushzoffsetmodify"];
            }

            Dobot::CDescartesPoint pointa{};
            pointa.x = 264.8929 + modifiedupx + offsetX;
            pointa.y = -285.1852 + modifiedupy + offsetY;
            pointa.z = 391.0669 + modifiedupz + modifiedup + offsetZ;
            pointa.rx = -179.7725;
            pointa.ry = -1.3507;
            pointa.rz = -145.9055;
            demo->moveRobotC(pointa, pointa);
            std::cout << "机械臂到达起始点" << std::endl;
            std::cout << "跳過牙刷微調。" << std::endl;

            double tcpx = -9.748236;
            double tcpy = -186.312977;
            double tcpz = 223.252632;
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

    double gxdown2s, gydown2s, gzdown2s, grxdown2s, grydown2s, grzdown2s;
    while (!demo->getCurrentPose(0, 5, gxdown2s, gydown2s, gzdown2s, grxdown2s, grydown2s, grzdown2s))
    {
        std::cout << "获取姿态中。。。。。" << std::endl;
    }

    std::cout << "侧向tcp位置调整后" << gxdown2s << " " << gydown2s << " " << gzdown2s << " "
              << grxdown2s << " " << grydown2s << " " << grzdown2s << std::endl;

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@生成的轨迹转移到机械臂末端@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    std::cout << "将轨迹转移到机械臂末端" << std::endl;
    int python_result11 = std::system(command3.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    firstPoseback.z += 20;
    demo->moveRobotC(firstPoseback, firstPoseback);

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

    // 添加一个标志位来判断是否是第一次运行
    bool isFirstRun = false;
    Eigen::Vector3d totalDeltaOffset(0, 0, 0); // 使用独立的累积偏移变量

    // 在 while 循环之前，先读取一次 Brush_offset 文件
    std::ifstream inputFile(Brush_offset);
    if (!inputFile.is_open())
    {
        std::cerr << "无法打开文件进行读取：" << Brush_offset << std::endl;
        return -1; // 或者处理错误
    }

    json loadedJson;
    inputFile >> loadedJson;
    inputFile.close();

    // 获取初始偏移量
    double offsetX = loadedJson["brushxoffset"];
    double offsetY = loadedJson["brushyoffset"];
    double offsetZ = loadedJson["brushzoffset"];

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
        if (isFirstRun && (user_choice == 'n' || user_choice == 'N'))
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
                double dx = brushDirsx_local.x();
                double dy = brushDirsx_local.y();
                double dz = brushDirsx_local.z();

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

        // 将偏移应用到tcp
        // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        modifyoffsetX += deltaOffset.x();
        modifyoffsetY += deltaOffset.y();
        modifyoffsetZ += deltaOffset.z();

        // 将每次调整的坐标应用到tcp上去
        // 转移到机械臂末端的误差
        Vector3 vecA = {deltaOffset.x(), deltaOffset.y(), deltaOffset.z()};
        double gxc, gyc, gzc, grxc, gryc, grzc;
        while (!demo->getCurrentPose(0, 0, gxc, gyc, gzc, grxc, gryc, grzc))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << grxc << "," << gryc << "," << grzc << std::endl;
        Vector3 vecB = transformVectorAToB(vecA, grxc, gryc, grzc);
        std::cout << "向量在坐标系 B 下的值为：" << std::endl;

        // 只修改变量，不立即写入文件
        // offsetXs += vecB.x;
        // offsetYs += vecB.y;
        // offsetZs += vecB.z;

        // offsetX += deltaOffset.x();
        // offsetY += deltaOffset.y();
        // offsetZ += deltaOffset.z();

        // std::cout << "临时偏移量更新为：" << std::endl;
        // std::cout << "Xc: " << offsetXs << "\nYc: " << offsetYs << "\nZc: " << offsetZs << std::endl;

        // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
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

        std::cout << "\n是否满意当前调整后的轨迹？(y/n): ";
        char choice;
        std::cin >> choice;
        std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

        demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);

        if (choice == 'y' || choice == 'Y')
        {
            userSatisfied = true;
            std::cout << "调整完成 ✅\n";
            std::cout << "最终总偏移量: " << totalDeltaOffset.transpose() << std::endl;

            json offsetJson;

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
                std::cout << "偏移量已成功保存到文件：" << Brush_offset << std::endl;
            }
            else
            {
                std::cerr << "无法打开文件进行写入：" << Brush_offset << std::endl;
            }
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
        const std::string Std_Traj_Json = "../defaultconfig/rightside/standard_trajectory.json";
        const std::string Std_Traj_Json_NoComp = "../defaultconfig/rightside/standard_trajectory_nocomp.json";

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

            double tcpx = -9.748236;
            double tcpy = -186.312977;
            double tcpz = 223.252632;
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

            std::cout << "\n是否满意当前调整后的轨迹？(y/n): ";
            char choice;
            std::cin >> choice;
            std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

            demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);

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
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            float force[6]{};
            while (obj->GetCurrentForceData(force) != 28 && force[2] != 0)
            {
                std::cerr << "获取力控数据失败\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
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
                    std::cout << "  proj 1= " << proj << std::endl;
                    forcerepaired << proj << std::endl;
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
                    std::cout << "  proj 2= " << proj << std::endl;
                    forcerepaired << proj << std::endl;
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
    demo->moveRobotC(pointsafe, pointsafe);
    std::cout << "初始位：先上抬再旋转，前往轨迹起点..." << std::endl;
    demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
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
            // selectedPoints[0].z += 10;
            demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
            // std::this_thread::sleep_for(std::chrono::seconds(1));

            // Dobot::CDescartesPoint rotatetooljointjumps{};
            // rotatetooljointjumps.x = 30;
            // rotatetooljointjumps.y = 0;
            // rotatetooljointjumps.z = -30;
            // rotatetooljointjumps.rx = 0;
            // rotatetooljointjumps.ry = 0;
            // rotatetooljointjumps.rz = 0;
            // demo->RelMovJDemo(rotatetooljointjumps, 0, 5, 20, 50, 100);

            // selectedPoints[0].z -= 10;
            demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
            // std::this_thread::sleep_for(std::chrono::seconds(1));

            demo->movsDemoC(selectedPoints, params1);
            // std::this_thread::sleep_for(std::chrono::seconds(2));

            // 根据是否是最后一行选择不同的动作
            if (!isLastLine)
            {
                // 非最后一行：执行原来的向后移动
                // Dobot::CDescartesPoint rotatetooljointjumpss{};
                // rotatetooljointjumpss.x = -15;
                // rotatetooljointjumpss.y = 0;
                // rotatetooljointjumpss.z = 0;
                // rotatetooljointjumpss.rx = 0;
                // rotatetooljointjumpss.ry = 0;
                // rotatetooljointjumpss.rz = 0;
                // demo->RelMovJDemo(rotatetooljointjumpss, 0, 5, 20, 50, 100);
                std::cout << "hello " << std::endl;
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
        }
    }

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
        rotatetooljointjumps.x = 10;
        rotatetooljointjumps.y = 0;
        rotatetooljointjumps.z = -38;
        rotatetooljointjumps.rx = 0;
        rotatetooljointjumps.ry = 0;
        rotatetooljointjumps.rz = 0;
        demo->RelMovJDemo(rotatetooljointjumps, 0, 5, 20, 50, 100);

        descartesPointsforce[currentTargetId].z -= 50;
        demo->moveRobotC(descartesPointsforce[currentTargetId], descartesPointsforce[currentTargetId]);
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        // B. 执行您要求的特殊动作 (向 X 负方向移动 15mm)
        Dobot::CDescartesPoint rotatetooljointjumpss{};
        rotatetooljointjumpss.x = -35;
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
            std::cout << "已完成最后一行索引，流程结束。" << std::endl;
        }
    }

    demo->RelMovJDemo(rotatetooljointup, 0, 0, 20, 50, 100);

    // 退出
    demo->moveRobotC(pointsafe, pointsafe);

    std::cout << "正在退出程序請稍後：-）" << std::endl;
    // 设备资源(StopCapture + delete demo)统一由 deviceGuard 在退出时释放
    return 0;
}
