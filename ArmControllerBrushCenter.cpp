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

/* ======================= 键盘微调函数 ======================= */
/* ======================= 拖拽微调函数 ======================= */
void dragTuneXYZ(DobotTcpDemo *demo, Dobot::CDescartesPoint &curPose,
                 Eigen::Vector3d &totalOffset)
{
    // 拖拽前记录刷尖(tool5)在base系的位置
    double bx = 0, by = 0, bz = 0, brx = 0, bry = 0, brz = 0;
    while (!demo->getCurrentPose(0, 5, bx, by, bz, brx, bry, brz))
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 进入拖拽前等待到位稳定
    demo->startDrag();
    std::cout << "\n===== 拖拽微调模式(手动) =====\n"
              << "机械臂已进入拖拽模式：请手动拖动机械臂，使刷尖到达目标位姿。\n"
              << "(手动微调: 拖拽后将直接采用此刻的完整位姿 xyz + rx/ry/rz 作为新的固定姿态)\n"
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

    // 拖拽后记录刷尖(tool5)在base系的位置(仅用于记录累计位移)
    double ax = 0, ay = 0, az = 0, arx = 0, ary = 0, arz = 0;
    while (!demo->getCurrentPose(0, 5, ax, ay, az, arx, ary, arz))
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 拖拽后记录法兰(tool0)在base系的完整位姿(xyz + rx/ry/rz)
    double fx = 0, fy = 0, fz = 0, frx = 0, fry = 0, frz = 0;
    while (!demo->getCurrentPose(0, 0, fx, fy, fz, frx, fry, frz))
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    demo->stopDrag();
    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // 等待退出拖拽模式稳定

    // base系下刷尖位移，仅用于记录累计偏移
    double dx = ax - bx;
    double dy = ay - by;
    double dz = az - bz;
    totalOffset += Eigen::Vector3d(dx, dy, dz);

    // 手动微调: 直接采用拖拽后的完整法兰位姿(含 rx/ry/rz)作为新的固定姿态
    curPose.x = fx;
    curPose.y = fy;
    curPose.z = fz;
    curPose.rx = frx;
    curPose.ry = fry;
    curPose.rz = frz;
    demo->moveRobotC(curPose, curPose);

    std::cout << "拖拽后法兰新固定位姿: x=" << fx << " y=" << fy << " z=" << fz
              << " rx=" << frx << " ry=" << fry << " rz=" << frz << std::endl;
    std::cout << "拖拽位移(刷尖, base系)[mm]: " << dx << ", " << dy << ", " << dz << std::endl;
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

    Dobot::CDescartesPoint rotatetooljoint{};
    rotatetooljoint.x = 0;
    rotatetooljoint.y = 0;
    rotatetooljoint.z = 0;
    rotatetooljoint.rx = 0;
    rotatetooljoint.ry = 0;
    rotatetooljoint.rz = 0;

    std::cout << "\n===== XYZ 微调模式 =====\n"
              << "W/S : +Y / -Y\n"
              << "A/D : -X / +X\n"
              << "Q/E : +Z / -Z\n"
              << "[ U ] : ry + 1.0 度\n"
              << "[ J ] : ry - 1.0 度\n"
              << "Enter : 结束微调\n";

    while (true)
    {
        if (_kbhit())
        {

            std::cout << "\n===== XYZ 微调模式 =====\n"
                      << "W/S : +Y / -Y\n"
                      << "A/D : -X / +X\n"
                      << "Q/E : +Z / -Z\n"
                      << "[ U ] : ry + 1.0 度\n"
                      << "[ J ] : ry - 1.0 度\n"
                      << "Enter : 结束微调\n";

            char key = _getch();

            double dx = 0, dy = 0, dz = 0;

            if (key == 'w')
            {
                std::cout << "w" << std::endl;
                dx += -0.017294;
                dy += -0.016502;
                dz += -0.999714;
                curPose.x += dx;
                curPose.y += dy;
                curPose.z += dz;

                demo->moveRobotC(curPose, curPose);
            }

            else if (key == 's')
            {
                std::cout << "s" << std::endl;
                dx += 0.017294;
                dy += 0.016502;
                dz += 0.999714;
                curPose.x += dx;
                curPose.y += dy;
                curPose.z += dz;

                demo->moveRobotC(curPose, curPose);
            }
            else if (key == 'a')
            {
                std::cout << "a" << std::endl;
                dx += 0.827884;
                dy += 0.560404;
                dz += -0.023572;
                curPose.x += dx;
                curPose.y += dy;
                curPose.z += dz;

                demo->moveRobotC(curPose, curPose);
            }
            else if (key == 'd')
            {
                std::cout << "d" << std::endl;
                dx += -0.827884;
                dy += -0.560404;
                dz += 0.023572;
                curPose.x += dx;
                curPose.y += dy;
                curPose.z += dz;

                demo->moveRobotC(curPose, curPose);
            }
            else if (key == 'q')
            {
                std::cout << "q" << std::endl;
                dx += 0.560633;
                dy += -0.828055;
                dz += 0.003970;
                curPose.x += dx;
                curPose.y += dy;
                curPose.z += dz;

                demo->moveRobotC(curPose, curPose);
            }
            else if (key == 'e')
            {
                std::cout << "e" << std::endl;
                dx += -0.560633;
                dy += 0.828055;
                dz += -0.003970;
                curPose.x += dx;
                curPose.y += dy;
                curPose.z += dz;

                demo->moveRobotC(curPose, curPose);
            }
            if (key == 'u')
            {
                std::cout << "u" << std::endl;
                rotatetooljoint.rx = 1.0;
                demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);
                double gx, gy, gz, grx, gry, grz;

                while (!demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                Eigen::Matrix3d rotationMatrixss = eulerDegToRotationMatrix(grx, gry, grz);
                Eigen::Vector3d brushDirss = rotationMatrixss.col(2);
                brushDirss.normalize();
                brushDirsz = brushDirss;
                curPose.x = gx;
                curPose.y = gy;
                curPose.z = gz;
                curPose.rx = grx;
                curPose.ry = gry;
                curPose.rz = grz;
            }
            else if (key == 'j')
            {
                std::cout << "j" << std::endl;
                rotatetooljoint.rx = -1.0;
                demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);
                double gx, gy, gz, grx, gry, grz;

                while (!demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                Eigen::Matrix3d rotationMatrixss = eulerDegToRotationMatrix(grx, gry, grz);
                Eigen::Vector3d brushDirss = rotationMatrixss.col(2);
                brushDirss.normalize();
                brushDirsz = brushDirss;
                curPose.x = gx;
                curPose.y = gy;
                curPose.z = gz;
                curPose.rx = grx;
                curPose.ry = gry;
                curPose.rz = grz;
            }
            else if (key == 'z')
            {
                std::cout << "z" << std::endl;
                dx += brushDirsz.x();
                dy += brushDirsz.y();
                dz += brushDirsz.z();
                curPose.x += dx;
                curPose.y += dy;
                curPose.z += dz;

                demo->moveRobotC(curPose, curPose);
            }
            else if (key == 'x')
            {
                std::cout << "x" << std::endl;
                dx += -brushDirsz.x();
                dy += -brushDirsz.y();
                dz += -brushDirsz.z();
                curPose.x += dx;
                curPose.y += dy;
                curPose.z += dz;

                demo->moveRobotC(curPose, curPose);
            }
            else if (key == 'c')
            {
                dx += brushDirsy.x();
                dy += brushDirsy.y();
                dz += brushDirsy.z();
                curPose.x += dx;
                curPose.y += dy;
                curPose.z += dz;

                demo->moveRobotC(curPose, curPose);
            }
            else if (key == 'v')
            {
                dx += -brushDirsy.x();
                dy += -brushDirsy.y();
                dz += -brushDirsy.z();
                curPose.x += dx;
                curPose.y += dy;
                curPose.z += dz;

                demo->moveRobotC(curPose, curPose);
            }
            else if (key == 'b')
            {
                dx += brushDirsx.x();
                dy += brushDirsx.y();
                dz += brushDirsx.z();
                curPose.x += dx;
                curPose.y += dy;
                curPose.z += dz;

                demo->moveRobotC(curPose, curPose);
            }
            else if (key == 'n')
            {
                dx += -brushDirsx.x();
                dy += -brushDirsx.y();
                dz += -brushDirsx.z();
                curPose.x += dx;
                curPose.y += dy;
                curPose.z += dz;

                demo->moveRobotC(curPose, curPose);
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

            std::cout << "累计偏移 [mm]: " << totalOffset.transpose() << std::endl;
        }

        Sleep(10);
    }
}

// 鍵盤手動旋轉
void fineTuneRY(DobotTcpDemo *demo, double &rotationoffset)
{
    std::cout << "\n===== RY (Pitch) 旋转微调模式 =====\n"
              << "[ U ] : ry + 1.0 度\n"
              << "[ J ] : ry - 1.0 度\n"
              << "Enter : 结束微调\n";

    Dobot::CDescartesPoint rotatetooljoint{};
    rotatetooljoint.x = 0;
    rotatetooljoint.y = 0;
    rotatetooljoint.z = 0;
    rotatetooljoint.rx = 0;
    rotatetooljoint.ry = 0;
    rotatetooljoint.rz = 0;

    while (true)
    {
        if (_kbhit())
        {
            std::cout << "\n===== RY (Pitch) 旋转微调模式 =====\n"
                      << "[ U ] : ry + 1.0 度\n"
                      << "[ J ] : ry - 1.0 度\n"
                      << "Enter : 结束微调\n";
            char key = _getch();

            if (key == 'u' || key == 'U')
            {
                rotatetooljoint.rx = 1.0;
                rotationoffset += 1.0;
            }
            else if (key == 'j' || key == 'J')
            {
                rotatetooljoint.rx = -1.0;
                rotationoffset -= 1.0;
            }
            else if (key == 13) // Enter 键
            {
                std::cout << "RY 微调结束。\n";
                break;
            }
            else
            {
                continue;
            }
            demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);
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

// 簡單做法是修改初始姿態，然後再改刷牙方式即可
int main()
{
    // @@@@@@@@@@@@@@@@@@@@@@@@區別代碼@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    const std::string Force_FILE_PATH = "../defaultconfig/center/center.txt";

    const std::string Movs_FORCE_LOG_PATHs = "../defaultconfig/center/movs_force_during_movs.txt";
    std::ofstream forcerepaired(Movs_FORCE_LOG_PATHs);
    if (!forcerepaired.is_open())
    {
        std::cerr << "无法保存含有力控的路径" << std::endl;
        return -1;
    }
    // const std::string Brush_offset = "../defaultconfig/center/brushoffsets.json";
    const std::string Brush_offset = "../defaultconfig/brushoffsets.json";
    const std::string Brush_offset_path = "../defaultconfig/center/brushoffsets_path.json";
    const std::string Brush_Config = "../defaultconfig/config.json";
    int value = 6;

    std::string command1 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\GenerateAnyPathCenter.py "
                           "\"";

    std::string command2 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\GenerateAnyPathRepeat.py " +
                           std::to_string(value) + "\"";

    std::string command3 = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate"
                           "&& python D:\\UsmileProject\\hand_eye_calibration\\TCPRotation10OthersDorobotCenter.py " +
                           std::to_string(value) + "\"";

    const std::string oldsegment = "../defaultconfig/center/all_segments.txt";
    std::string indexFilePath = "../defaultconfig/center/all_segments.txt";
    std::ofstream poseFile("../defaultconfig/center/current_pose_from_getpose.txt");
    const std::string eepath = "../defaultconfig/center/ee_poses.txt";

    Dobot::CDescartesPoint rotatetooljoint{};
    rotatetooljoint.x = 0.01;
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
    double modifiedupx = -6.9142260000002125 + 14.163013999999862;
    double modifiedupy = -2.0350259999999025 - 24.190834000000166;
    double modifiedupz = 101.3919;

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

    // 任意点回到起始点
    demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);
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

        double tcpx = -9.352824;
        double tcpy = -186.998296;
        double tcpz = 224.724733;

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

        double tcpx = -9.352824;
        double tcpy = -186.998296;
        double tcpz = 224.724733;
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

            double offsetXs = loadedJson.value("brushxoffsets", 0.0);
            double offsetYs = loadedJson.value("brushyoffsets", 0.0);
            double offsetZs = loadedJson.value("brushzoffsets", 0.0);

            double tcpx = -9.352824;
            double tcpy = -186.998296;
            double tcpz = 224.724733;
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
    // std::this_thread::sleep_for(std::chrono::seconds());

    // @@@@@@@@@@@@@@@@@@@@@@@@@@调整牙刷起始姿态@@@@@@@@@@@@@@@@@@

    std::cout << "牙刷初始姿态调整，请确认Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
    std::cout << "初始位：先上抬再旋转..." << std::endl;
    demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
    demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);
    std::cout << "牙刷初始姿态已经调整好，请确认Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    // ========== 从控制器读取真实位姿（GetPose） ==========
    double gx, gy, gz, grx, gry, grz;
    while (true)
    {
        if (demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
        {
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

        Dobot::CDescartesPoint firstPose1{};
        firstPose1.x = brushpointsoffset_ee_poses[0].x;
        firstPose1.y = brushpointsoffset_ee_poses[0].y;
        firstPose1.z = brushpointsoffset_ee_poses[0].z;
        firstPose1.rx = brushpointsoffset_ee_poses[0].a;
        firstPose1.ry = brushpointsoffset_ee_poses[0].b;
        firstPose1.rz = brushpointsoffset_ee_poses[0].c;

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

        // Eigen::Vector3d deltaOffset(0, 0, 0);
        // fineTuneXYZ(demo, firstPose, deltaOffset);

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
        // // 若檔案不存在，則保持預設的 (0, 0, 0)
        // firstPose.x += deltaOffset[0];
        // firstPose.y += deltaOffset[1];
        // firstPose.z += deltaOffset[2];

        demo->moveRobotC(firstPose, firstPose);

        // 旋轉手調
        // double rotationoffset = 0;
        // fineTuneRY(demo, rotationoffset);
        std::cout << "机械臂到达起始点，请确认按Enter" << std::endl;
        std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

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
        double diffx = firstPose.x - firstPose1.x;
        double diffy = firstPose.y - firstPose1.y;
        double diffz = firstPose.z - firstPose1.z;
        double diffrx = firstPose.rx;
        double diffry = firstPose.ry;
        double diffrz = firstPose.rz;

        for (auto &p : brushpointsoffset_ee_poses)
        {
            p.x += diffx;
            p.y += diffy;
            p.z += diffz;
            p.a = diffrx;
            p.b = diffry;
            p.c = diffrz;
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
    std::cout << "初始位：先上抬再旋转，前往轨迹起点..." << std::endl;
    demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
    demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

    // 調整完xyz來調整rx,ry,rz

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
        // 说明: center 轨迹具有特殊性, 其标准轨迹为"模式1下微调好且未加力控"的轨迹, 存为 json
        const std::string Std_Traj_Json = "../defaultconfig/center/standard_trajectory.json";
        const std::string Std_Traj_Json_NoComp = "../defaultconfig/center/standard_trajectory_nocomp.json";

        // ---- 加载已有牙刷偏移并设置TCP(不做TCP标定/不做牙刷微调) ----
        // 与 ArmControllerBrushUpRight 一致: TCP写死, 先设TCP再走安全点, pointa不叠加牙刷偏移
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

            double tcpx = -9.352824;
            double tcpy = -186.998296;
            double tcpz = 224.724733;
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

            // ---- 模式1: 牙刷初始姿态调整(自动, 去Enter, 加延时+提示) ----
            std::cout << "[模式1] 牙刷初始姿态调整(自动), 即将上抬并旋转, 注意安全..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
            demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);
            std::cout << "[模式1] 牙刷初始姿态已调整, 即将记录位姿..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            // ---- 记录当前位姿(GetPose) ----
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

            Dobot::CDescartesPoint firstPose1{};
            firstPose1.x = brushpointsoffset_ee_poses[0].x;
            firstPose1.y = brushpointsoffset_ee_poses[0].y;
            firstPose1.z = brushpointsoffset_ee_poses[0].z;
            firstPose1.rx = brushpointsoffset_ee_poses[0].a;
            firstPose1.ry = brushpointsoffset_ee_poses[0].b;
            firstPose1.rz = brushpointsoffset_ee_poses[0].c;

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

            demo->moveRobotC(firstPose, firstPose);

            std::cout << "机械臂到达起始点，请确认按Enter" << std::endl;
            std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

            Eigen::Vector3d deltaOffset(0, 0, 0);
            char tuneSel = 'k';
            std::cout << "\n选择微调方式: k=键盘微调  d=拖拽微调，请输入后回车: ";
            std::cin >> tuneSel;
            std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
            if (tuneSel == 'd' || tuneSel == 'D')
                dragTuneXYZ(demo, firstPose, deltaOffset);
            else
                fineTuneXYZ(demo, firstPose, deltaOffset);

            double diffx = firstPose.x - firstPose1.x;
            double diffy = firstPose.y - firstPose1.y;
            double diffz = firstPose.z - firstPose1.z;
            double diffrx = firstPose.rx;
            double diffry = firstPose.ry;
            double diffrz = firstPose.rz;

            for (auto &p : brushpointsoffset_ee_poses)
            {
                p.x += diffx;
                p.y += diffy;
                p.z += diffz;
                p.a = diffrx;
                p.b = diffry;
                p.c = diffrz;
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
            params.freq = 0.2;
            demo->movsDemoC(descartesPoints, params);

            std::cout << "\n是否满意当前调整后的轨迹？(y/n): ";
            char choice;
            std::cin >> choice;
            std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

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

        // ---- 模式1: 保存"调整好且未加力控"的标准轨迹为 json (供模式3复用) ----
        if (plan.saveStdJson)
        {
            if (saveStandardTrajectoryJson(Std_Traj_Json, brushpointsoffset_ee_poses))
            {
                std::cout << "[模式1] 标准轨迹(未加力控)已保存为json: " << Std_Traj_Json
                          << " (" << brushpointsoffset_ee_poses.size() << " 点)" << std::endl;
            }
            else
            {
                std::cerr << "[模式1] 标准轨迹json保存失败: " << Std_Traj_Json << std::endl;
            }
        }

        // ---- 微调结束回到安全点 ----
        demo->moveRobotC(pointsafe, pointsafe);
        std::cout << "初始位：先上抬再旋转..." << std::endl;
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
        bool converged = false;
        bool firststep = false;
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
                    forcerepaired << proj << std::endl;
                    std::cout << "  proj 2= " << proj << std::endl;
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

    std::string line;
    int lineNumber = 0;
    while (std::getline(indexFile, line))
    {
        lineNumber++;
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
                std::cerr << "\n第 " << lineNumber << " 行索引 " << idx << " 超出范围 (0-"
                          << descartesPointsforce.size() - 1 << ")" << std::endl;
            }
        }

        // if (!selectedPoints.empty())
        // {
        //     demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
        //     std::this_thread::sleep_for(std::chrono::seconds(1));
        //     // std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //     demo->movsDemoC(selectedPoints, params1);
        //     std::this_thread::sleep_for(std::chrono::seconds(2));
        // }

        if (!selectedPoints.empty())
        {
            // 轮循所有点，依次移动到每个点
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

            // // 最后执行一次movsDemoC
            // demo->movsDemoC(selectedPoints, params1);
            // std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

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
