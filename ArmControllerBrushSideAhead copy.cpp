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

int main()
{
    // @@@@@@@@@@@@@@@@@@@@@@@@區別代碼@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    const std::string Force_FILE_PATH = "../defaultconfig/sideahead/sideahead.txt";
    // const std::string Brush_offset = "../defaultconfig/rightside/brushoffsets.json";

    const std::string Movs_FORCE_LOG_PATHs = "../defaultconfig/sideahead/movs_force_during_movs.txt";
    std::ofstream forcerepaired(Movs_FORCE_LOG_PATHs);
    if (!forcerepaired.is_open())
    {
        std::cerr << "无法保存含有力控的路径" << std::endl;
        return -1;
    }
    const std::string Brush_offset = "../defaultconfig/brushoffsets.json";
    const std::string Brush_offset_path = "../defaultconfig/sideahead/brushoffsets_path.json";
    const std::string Brush_Config = "../defaultconfig/config.json";
    int value = 7;

    std::string command1 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\GenerateAnyPath.py " +
                           std::to_string(value) + "\"";

    std::string command2 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\GenerateAnyPathRepeat.py " +
                           std::to_string(value) + "\"";

    std::string command3 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\TCPRotation10OthersDorobot.py " +
                           std::to_string(value) + "\"";

    const std::string oldsegment = "../defaultconfig/sideahead/all_segments.txt";
    std::string indexFilePath = "../defaultconfig/sideahead/all_segments.txt";
    std::string indexFilePath2 = "../defaultconfig/sideahead/support_points.txt";
    std::string indexFilePath3 = "../defaultconfig/rightside/ee_poses.txt";

    std::ofstream poseFile("../defaultconfig/sideahead/current_pose_from_getpose.txt");
    const std::string eepath = "../defaultconfig/sideahead/ee_poses.txt";

    Dobot::CDescartesPoint rotatetooljoint{};
    rotatetooljoint.x = 0;
    rotatetooljoint.y = 0;
    rotatetooljoint.z = 0;
    rotatetooljoint.rx = 0;
    rotatetooljoint.ry = -48;
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
    Dobot::CDescartesPoint rotatetooljointjump6{};

    double xA = 0, yA = 0, zA = 0, rxA = 0, ryA = 0, rzA = 0;
    double xB = 0, yB = 0, zB = 0, rxB = 0, ryB = 0, rzB = 0; // B绕Z旋转90度
    double dxm = 0, dym = 0, dzm = 0, drxm = 0, drym = 0, drzm = 0;

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
        // pressureParameter = 0.15;
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

    auto moveToSafeViaWaypoints = [&]() {
        demo->moveRobotC(pointpartone, pointpartone);
        demo->moveRobotC(pointtwo, pointtwo);
        demo->moveRobotC(pointsafe, pointsafe);
    };

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

        double tcpx = -9.748236 - vecB.x;
        double tcpy = -186.312977 - vecB.y;
        double tcpz = 223.252632 - vecB.z;
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

            double tcpx = -9.748236 - offsetXs;
            double tcpy = -186.312977 - offsetYs;
            double tcpz = 223.252632 - offsetZs;
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

    std::ifstream ee_poses_infile;
    std::vector<PointData> brushpointsoffset_ee_poses;

    // 询问用户是否沿用原来的轨迹
    std::cout << "是否沿用原来的轨迹？(y/n): ";
    char user_choice;
    std::cin >> user_choice;

    if (user_choice == 'y' || user_choice == 'Y')
    {
        // 用户选择沿用原来的轨迹，使用 Force_FILE_PATH
        ee_poses_infile.open(Force_FILE_PATH);
        if (!ee_poses_infile.is_open())
        {
            std::cerr << "无法打开文件: " << Force_FILE_PATH << std::endl;
            return -1;
        }
    }
    else if (user_choice == 'n' || user_choice == 'N')
    {
        // 用户选择不沿用，使用原来的 eepath
        ee_poses_infile.open(eepath);
        if (!ee_poses_infile.is_open())
        {
            std::cerr << "无法打开文件: " << eepath << std::endl;
            return -1;
        }
    }
    else
    {
        std::cerr << "无效输入，默认使用原有轨迹" << std::endl;
        ee_poses_infile.open(eepath);
        if (!ee_poses_infile.is_open())
        {
            std::cerr << "无法打开文件: " << eepath << std::endl;
            return -1;
        }
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

    // ================= 读取前牙引导轨迹(indexFilePath3) =================
    std::ifstream picked_path_infile(indexFilePath3);
    std::vector<Dobot::CDescartesPoint> pickedPathPoints;
    if (!picked_path_infile.is_open())
    {
        std::cerr << "无法打开前牙引导轨迹文件: " << indexFilePath3 << std::endl;
        return -1;
    }
    while (picked_path_infile >> dx >> dy >> dz >> rx >> ry >> rz)
    {
        Dobot::CDescartesPoint cp{};
        cp.x = dx;
        cp.y = dy;
        cp.z = dz + 30;
        cp.rx = rx;
        cp.ry = ry;
        cp.rz = rz;
        pickedPathPoints.push_back(cp);
    }

    picked_path_infile.close();
    if (pickedPathPoints.empty())
    {
        std::cerr << "前牙引导轨迹为空: " << indexFilePath3 << std::endl;
        return -1;
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
        // std::this_thread::sleep_for(std::chrono::milliseconds(300));
        std::cout << "[" << stageTag << "] 执行前牙引导轨迹..." << std::endl;
        demo->movsDemoC(pickedPathPoints, pickedParams);
        // std::this_thread::sleep_for(std::chrono::seconds(1));
    };

    bool isFirstRun = true;
    Eigen::Vector3d totalDeltaOffset(0, 0, 0); // 使用独立的累积偏移变量

    while (!userSatisfied)
    {

        Dobot::CDescartesPoint firstPose{};
        firstPose.x = brushpointsoffset_ee_poses[0].x;
        firstPose.y = brushpointsoffset_ee_poses[0].y;
        firstPose.z = brushpointsoffset_ee_poses[0].z;
        firstPose.rx = brushpointsoffset_ee_poses[0].a;
        firstPose.ry = brushpointsoffset_ee_poses[0].b;
        firstPose.rz = brushpointsoffset_ee_poses[0].c;
        demo->moveRobotC(pointsafe, pointsafe);
        demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

        Eigen::Matrix3d rotationMatrixs = eulerDegToRotationMatrix(firstPose.rx, firstPose.ry, firstPose.rz);
        Eigen::Vector3d brushDirs = rotationMatrixs.col(2);
        brushDirs.normalize();

        Dobot::CDescartesPoint pointstart{};
        pointstart.x = firstPose.x + -brushDirs.x() * 38;
        pointstart.y = firstPose.y + -brushDirs.y() * 38;
        pointstart.z = firstPose.z + -brushDirs.z() * 38;
        pointstart.rx = firstPose.rx;
        pointstart.ry = firstPose.ry;
        pointstart.rz = firstPose.rz;

        std::cout << "\n=========== 新一轮轨迹调整开始 ===========\n";
        runPickedPathToFrontTeeth("轨迹微调前");

        // 相对精准运动
        xA = pickedPathPoints.back().x;
        yA = pickedPathPoints.back().y;
        zA = pickedPathPoints.back().z;
        rxA = pickedPathPoints.back().rx;
        ryA = pickedPathPoints.back().ry;
        rzA = pickedPathPoints.back().rz;

        xB = pointstart.x;
        yB = pointstart.y;
        zB = pointstart.z;
        rxB = pointstart.rx;
        ryB = pointstart.ry;
        rzB = pointstart.rz;

        computeRelativeTransform(xA, yA, zA, rxA, ryA, rzA,
                                 xB, yB, zB, rxB, ryB, rzB,
                                 dxm, dym, dzm, drxm, drym, drzm);

        rotatetooljointjump6.x = dxm;
        rotatetooljointjump6.y = dym;
        rotatetooljointjump6.z = dzm;
        rotatetooljointjump6.rx = drxm;
        rotatetooljointjump6.ry = drym;
        rotatetooljointjump6.rz = drzm;
        demo->RelMovJDemo(rotatetooljointjump6, 0, 0, 20, 50, 100);

        // 根據軌跡向量來補償
        demo->moveRobotC(pointstart, pointstart);

        Eigen::Vector3d deltaOffset(0, 0, 0);
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

        Dobot::CDescartesPoint rotatetooljointjump2{};
        rotatetooljointjump2.x = 0;
        rotatetooljointjump2.y = 0;
        rotatetooljointjump2.z = -18;
        rotatetooljointjump2.rx = 0;
        rotatetooljointjump2.ry = 0;
        rotatetooljointjump2.rz = 0;

        demo->RelMovJDemo(rotatetooljointjump2, 0, 5, 20, 50, 100);
        

        if (choice == 'y' || choice == 'Y')
        {
            userSatisfied = true;
            std::cout << "调整完成 ✅\n";
            std::cout << "最终总偏移量: " << totalDeltaOffset.transpose() << std::endl;
        }
        else
        {
            moveToSafeViaWaypoints();
            std::cout << "继续调整...\n";
            // 注意：如果不是第一次运行，下次循环会继续使用交互式微调
        }
    }

    moveToSafeViaWaypoints();
    demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);
    runPickedPathToFrontTeeth("力控前");

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
    pointstarts.x = firstPoses.x + -brushDirss.x() * 38;
    pointstarts.y = firstPoses.y + -brushDirss.y() * 38;
    pointstarts.z = firstPoses.z + -brushDirss.z() * 38;
    pointstarts.rx = firstPoses.rx;
    pointstarts.ry = firstPoses.ry;
    pointstarts.rz = firstPoses.rz;

    xA = pickedPathPoints.back().x;
    yA = pickedPathPoints.back().y;
    zA = pickedPathPoints.back().z;
    rxA = pickedPathPoints.back().rx;
    ryA = pickedPathPoints.back().ry;
    rzA = pickedPathPoints.back().rz;

    xB = pointstarts.x;
    yB = pointstarts.y;
    zB = pointstarts.z;
    rxB = pointstarts.rx;
    ryB = pointstarts.ry;
    rzB = pointstarts.rz;

    computeRelativeTransform(xA, yA, zA, rxA, ryA, rzA,
                             xB, yB, zB, rxB, ryB, rzB,
                             dxm, dym, dzm, drxm, drym, drzm);

    // std::cout << "相对平移 (dx, dy, dz): " << dx << ", " << dy << ", " << dz << " 米" << std::endl;
    // std::cout << "相对旋转 (drx, dry, drz): " << drx << ", " << dry << ", " << drz << " 度" << std::endl;

    rotatetooljointjump6.x = dxm;
    rotatetooljointjump6.y = dym;
    rotatetooljointjump6.z = dzm;
    rotatetooljointjump6.rx = drxm;
    rotatetooljointjump6.ry = drym;
    rotatetooljointjump6.rz = drzm;
    demo->RelMovJDemo(rotatetooljointjump6, 0, 0, 20, 50, 100);
    // std::this_thread::sleep_for(std::chrono::seconds(1));

    demo->moveRobotC(pointstarts, pointstarts);
    // std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "微调结束回到安全点，请确认按Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@添加力控@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    double targetforcevalue = pressureParameter;
    std::cout << "\n理想的壓力值是 :  " << targetforcevalue << std::endl;
    std::vector<Dobot::CDescartesPoint> descartesPointsforce;
    std::ofstream forcerepairedoutputfile(Force_FILE_PATH);
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
        // z不参与
        offset.z = descartesPoints[0].z;
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
                // offset.z += delta.z();
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

    std::cout << "调整后的力控轨迹保存完毕" << std::endl;
    Dobot::CDescartesPoint rotatetooljointjump32{};
    rotatetooljointjump32.x = 0;
    rotatetooljointjump32.y = 0;
    rotatetooljointjump32.z = -18;
    rotatetooljointjump32.rx = 0;
    rotatetooljointjump32.ry = 0;
    rotatetooljointjump32.rz = 0;
    demo->RelMovJDemo(rotatetooljointjump32, 0, 5, 20, 50, 100);
    moveToSafeViaWaypoints();
    demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

    runPickedPathToFrontTeeth("往复刷牙前");
    std::cout << "往复刷牙前 Start :-)" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

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
    pointstartsk.x = firstPosesk.x + -brushDirssk.x() * 38;
    pointstartsk.y = firstPosesk.y + -brushDirssk.y() * 38;
    pointstartsk.z = firstPosesk.z + -brushDirssk.z() * 38;
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

    // std::cout << "相对平移 (dx, dy, dz): " << dx << ", " << dy << ", " << dz << " 米" << std::endl;
    // std::cout << "相对旋转 (drx, dry, drz): " << drx << ", " << dry << ", " << drz << " 度" << std::endl;

    rotatetooljointjump6.x = dxm;
    rotatetooljointjump6.y = dym;
    rotatetooljointjump6.z = dzm;
    rotatetooljointjump6.rx = drxm;
    rotatetooljointjump6.ry = drym;
    rotatetooljointjump6.rz = drzm;
    demo->RelMovJDemo(rotatetooljointjump6, 0, 0, 20, 50, 100);
    // std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "运行完整力控轨迹（每行一次movsDemoC）1" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    demo->moveRobotC(pointstartsk, pointstartsk);
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // demo->moveRobotC(firstPosesk, firstPosesk);

    // std::cout << "运行完整力控轨迹（每行一次movsDemoC）" << std::endl;
    // std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
    // std::cout << "运行完整力控轨迹（每行一次movsDemoC）" << std::endl;
    // std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

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
            std::cout << "开始运行轨迹" << std::endl;
            std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
            // 安全到达
            selectedPoints[0].z += 50;
            demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
            // std::this_thread::sleep_for(std::chrono::seconds(1));

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
                // std::this_thread::sleep_for(std::chrono::seconds(1));
                
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
                // std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }

    // 基于当前点往上抬
    Dobot::CDescartesPoint rotatetooljointup{};
    rotatetooljointup.x = 0;
    rotatetooljointup.y = 0;
    rotatetooljointup.z = -18;
    rotatetooljointup.rx = 0;
    rotatetooljointup.ry = 0;
    rotatetooljointup.rz = 0;

    demo->RelMovJDemo(rotatetooljointup, 0, 0, 20, 50, 100);
    indexFile.close();
    moveToSafeViaWaypoints();

    // 退出
    demo->moveRobotC(pointsafe, pointsafe);
    demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);


    std::cout << "开始浮刷" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
    runPickedPathToFrontTeeth("扶刷前");

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

    // std::cout << "相对平移 (dx, dy, dz): " << dx << ", " << dy << ", " << dz << " 米" << std::endl;
    // std::cout << "相对旋转 (drx, dry, drz): " << drx << ", " << dry << ", " << drz << " 度" << std::endl;

    rotatetooljointjump6.x = dxm;
    rotatetooljointjump6.y = dym;
    rotatetooljointjump6.z = dzm;
    rotatetooljointjump6.rx = drxm;
    rotatetooljointjump6.ry = drym;
    rotatetooljointjump6.rz = drzm;
    demo->RelMovJDemo(rotatetooljointjump6, 0, 0, 20, 50, 100);
    // std::this_thread::sleep_for(std::chrono::seconds(1));

    demo->moveRobotC(pointstartsk, pointstartsk);
    // std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "运行完整力控轨迹（每行一次movsDemoC）" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

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
        rotatetooljointjumps.x = 28;
        rotatetooljointjumps.y = 0;
        rotatetooljointjumps.z = -20;
        rotatetooljointjumps.rx = 0;
        rotatetooljointjumps.ry = 0;
        rotatetooljointjumps.rz = 0;
        demo->RelMovJDemo(rotatetooljointjumps, 0, 5, 20, 50, 100);

        descartesPointsforce[currentTargetId].z -= 50;
        demo->moveRobotC(descartesPointsforce[currentTargetId], descartesPointsforce[currentTargetId]);
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        // B. 执行您要求的特殊动作 (向 X 负方向移动 15mm)
        Dobot::CDescartesPoint rotatetooljointjumpss{};
        rotatetooljointjumpss.x = -35.0;
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

    Dobot::CDescartesPoint rotatetooljointjump22{};
    rotatetooljointjump22.x = 0;
    rotatetooljointjump22.y = 0;
    rotatetooljointjump22.z = -18;
    rotatetooljointjump22.rx = 0;
    rotatetooljointjump22.ry = 0;
    rotatetooljointjump22.rz = 0;

    demo->RelMovJDemo(rotatetooljointjump22, 0, 0, 20, 50, 100);
    moveToSafeViaWaypoints();

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
