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

Eigen::Vector3d getManualOffset2(DobotTcpDemo *demo, double /*refX*/, double /*refY*/, double /*refZ*/)
{
    double sx = 0, sy = 0, sz = 0, srx = 0, sry = 0, srz = 0;
    while (!demo->getCurrentPose(0, 0, sx, sy, sz, srx, sry, srz))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\n===== 刷頭微調 (仅记录相对当前位置的键盘调整量) =====\n"
              << "W/S : +Y / -Y\n"
              << "A/D : -X / +X\n"
              << "Q/E : +Z / -Z\n"
              << "直接 Enter : 不做微调\n";

    while (true)
    {
        if (_kbhit())
        {
            std::cout << "\n===== 刷頭微調 =====\n"
                      << "W/S : +Y / -Y | A/D : -X / +X | Q/E : +Z / -Z | Enter : 结束\n";

            char key = _getch();

            if (key == 'w')
            {
                Dobot::CDescartesPoint rotatetooljoint{};
                rotatetooljoint.x = 0;
                rotatetooljoint.y = 0;
                rotatetooljoint.z = -1;
                rotatetooljoint.rx = 0;
                rotatetooljoint.ry = 0;
                rotatetooljoint.rz = 0;

                demo->RelMovJDemo(rotatetooljoint, 0, 0, 20, 50, 100);
            }
            else if (key == 's')
            {
                Dobot::CDescartesPoint rotatetooljoint{};
                rotatetooljoint.x = 0;
                rotatetooljoint.y = 0;
                rotatetooljoint.z = 1;
                rotatetooljoint.rx = 0;
                rotatetooljoint.ry = 0;
                rotatetooljoint.rz = 0;

                demo->RelMovJDemo(rotatetooljoint, 0, 0, 20, 50, 100);
            }
            else if (key == 'a')
            {
                Dobot::CDescartesPoint rotatetooljoint{};
                rotatetooljoint.x = -1;
                rotatetooljoint.y = 0;
                rotatetooljoint.z = 0;
                rotatetooljoint.rx = 0;
                rotatetooljoint.ry = 0;
                rotatetooljoint.rz = 0;

                demo->RelMovJDemo(rotatetooljoint, 0, 0, 20, 50, 100);
            }
            else if (key == 'd')
            {
                Dobot::CDescartesPoint rotatetooljoint{};
                rotatetooljoint.x = 1;
                rotatetooljoint.y = 0;
                rotatetooljoint.z = 0;
                rotatetooljoint.rx = 0;
                rotatetooljoint.ry = 0;
                rotatetooljoint.rz = 0;

                demo->RelMovJDemo(rotatetooljoint, 0, 0, 20, 50, 100);
            }
            else if (key == 'q')
            {
                Dobot::CDescartesPoint rotatetooljoint{};
                rotatetooljoint.x = 0;
                rotatetooljoint.y = -1;
                rotatetooljoint.z = 0;
                rotatetooljoint.rx = 0;
                rotatetooljoint.ry = 0;
                rotatetooljoint.rz = 0;

                demo->RelMovJDemo(rotatetooljoint, 0, 0, 20, 50, 100);
            }
            else if (key == 'e')
            {
                Dobot::CDescartesPoint rotatetooljoint{};
                rotatetooljoint.x = 0;
                rotatetooljoint.y = 1;
                rotatetooljoint.z = 0;
                rotatetooljoint.rx = 0;
                rotatetooljoint.ry = 0;
                rotatetooljoint.rz = 0;

                demo->RelMovJDemo(rotatetooljoint, 0, 0, 20, 50, 100);
            }

            else if (key == 13)
            {
                std::cout << "\n微调结束\n";
                break;
            }
            else
                continue;
        }
        Sleep(10);
    }

    double gx = 0, gy = 0, gz = 0, grx = 0, gry = 0, grz = 0;
    while (!demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return Eigen::Vector3d(gx - sx, gy - sy, gz - sz);
}

Eigen::Vector3d recordSideRotationOffset(DobotTcpDemo *demo,
                                         const Dobot::CDescartesPoint &calibPoint,
                                         Dobot::CDescartesPoint rotation,
                                         const char *label,
                                         Dobot::CDescartesPoint *outFlangePoseAfterTune = nullptr,
                                         Eigen::Vector3d *outTipBaseMm = nullptr)
{
    std::cout << "\n===== " << label << " 姿态旋转偏移标定 =====\n"
              << "先上抬 60mm，再旋转，再下降 60mm，请微调 XYZ 后按 Enter 结束\n";

    demo->moveRobotC(calibPoint, calibPoint);

    Dobot::CDescartesPoint rotJump{};
    rotJump.x = 0;
    rotJump.y = 0;
    rotJump.z = -60;
    rotJump.rx = 0;
    rotJump.ry = 0;
    rotJump.rz = 0;
    demo->RelMovJDemo(rotJump, 0, 5, 20, 50, 100);
    demo->RelMovJDemo(rotation, 0, 5, 20, 50, 100);

    double gxdown, gydown, gzdown, grxdown, grydown, grzdown;
    while (!demo->getCurrentPose(0, 0, gxdown, gydown, gzdown, grxdown, grydown, grzdown))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    Dobot::CDescartesPoint poseDown{};
    poseDown.x = gxdown;
    poseDown.y = gydown;
    poseDown.z = gzdown - 60;
    poseDown.rx = grxdown;
    poseDown.ry = grydown;
    poseDown.rz = grzdown;
    demo->moveRobotC(poseDown, poseDown);

    Eigen::Vector3d offset = getManualOffset2(demo, calibPoint.x, calibPoint.y, calibPoint.z);
    if (offset.isZero(1e-6))
    {
        std::cout << label << " 键盘微调量 [mm]: 0 (未调整)\n";
    }
    else
    {
        std::cout << label << " 键盘微调量 [mm]: " << offset.transpose() << std::endl;
    }

    if (outFlangePoseAfterTune != nullptr)
    {
        double gx = 0, gy = 0, gz = 0, grx = 0, gry = 0, grz = 0;
        while (!demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        outFlangePoseAfterTune->x = gx;
        outFlangePoseAfterTune->y = gy;
        outFlangePoseAfterTune->z = gz;
        outFlangePoseAfterTune->rx = grx;
        outFlangePoseAfterTune->ry = gry;
        outFlangePoseAfterTune->rz = grz;
        std::cout << label << " TCP标定记录-法兰 tool0 (mm/°): "
                  << gx << ", " << gy << ", " << gz << ", "
                  << grx << ", " << gry << ", " << grz << std::endl;

        if (outTipBaseMm != nullptr)
        {
            double tx = 0, ty = 0, tz = 0, trx = 0, try_ = 0, trz = 0;
            while (!demo->getCurrentPose(0, 5, tx, ty, tz, trx, try_, trz))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            *outTipBaseMm = Eigen::Vector3d(tx, ty, tz);
        }
    }

    double gxup, gyup, gzup, grxup, gryup, grzup;
    while (!demo->getCurrentPose(0, 0, gxup, gyup, gzup, grxup, gryup, grzup))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    Dobot::CDescartesPoint poseUp{};
    poseUp.x = gxup;
    poseUp.y = gyup;
    poseUp.z = gzup + 120;
    poseUp.rx = grxup;
    poseUp.ry = gryup;
    poseUp.rz = grzup;
    demo->moveRobotC(poseUp, poseUp);

    Dobot::CDescartesPoint poseTopAdjusted{};
    poseTopAdjusted.x = poseUp.x;
    poseTopAdjusted.y = poseUp.y;
    poseTopAdjusted.z = poseUp.z;
    poseTopAdjusted.rx = calibPoint.rx;
    poseTopAdjusted.ry = calibPoint.ry;
    poseTopAdjusted.rz = calibPoint.rz;
    demo->moveRobotC(poseTopAdjusted, poseTopAdjusted);

    Dobot::CDescartesPoint calibReset = calibPoint;
    demo->moveRobotC(calibReset, calibReset);
    return offset;
}

void saveRotationOffsetEntry(json &rotationJson, const std::string &key, const Eigen::Vector3d &offset)
{
    rotationJson[key]["x"] = offset.x();
    rotationJson[key]["y"] = offset.y();
    rotationJson[key]["z"] = offset.z();
}

constexpr double kTcpBaselineX = -9.748236;
constexpr double kTcpBaselineY = -186.312977;
constexpr double kTcpBaselineZ = 223.252632;

// Dobot GetPose: 位置 mm，Rx/Ry/Rz 为角度 °，Z-Y-X（与 transformVectorAToB 一致）
Eigen::Matrix3d dobotPoseToRotationMatrixDeg(double rx_deg, double ry_deg, double rz_deg)
{
    const double ax = degToRad(rx_deg);
    const double ay = degToRad(ry_deg);
    const double az = degToRad(rz_deg);
    const double cx = std::cos(ax), sx = std::sin(ax);
    const double cy = std::cos(ay), sy = std::sin(ay);
    const double cz = std::cos(az), sz = std::sin(az);

    Eigen::Matrix3d R;
    R << cy * cz, cz * sx * sy - cx * sz, sx * sz + cx * cz * sy,
        cy * sz, cx * cz + sx * sy * sz, cx * sy * sz - cz * sx,
        -sy, cy * sx, cx * cy;
    return R;
}

struct FourPointTcpSample
{
    Dobot::CDescartesPoint flange{};
    Eigen::Vector3d tipBaseMm{}; // 同时刻 GetPose(0,5) 刷尖世界坐标，仅用于四点法求解
};

struct FourPointTcpResult
{
    Eigen::Vector3d tcpFlange{};
    Eigen::Vector3d contactMeanBaseMm{};
    Eigen::Vector3d tipSpreadMm{};
    Eigen::Vector3d contactSpreadMm{};
    Eigen::Vector3d tcpStdDevMm{};
    double maxTipSpreadMm = 0.0;
    double maxContactSpreadMm = 0.0;
    bool alignmentWarning = false;
    bool valid = false;
};

// 四点法 TCP：仅用微调后的法兰 tool0 姿态。
// p_i + R_i*t = P（四姿态刷尖应对准同一点 P），用 baseline TCP 引导迭代求 t（避免 Rx≈-180° 时 SVD 病态）。
bool computeFourPointTcp(const std::vector<FourPointTcpSample> &samples,
                         FourPointTcpResult &outResult,
                         std::string *outError = nullptr)
{
    if (samples.size() < 4)
    {
        if (outError)
            *outError = "四点 TCP 标定需要 4 个姿态，当前只有 " + std::to_string(samples.size()) + " 个";
        return false;
    }

    const Eigen::Vector3d tcpBootstrap(kTcpBaselineX, kTcpBaselineY, kTcpBaselineZ);

    Eigen::Vector3d contactPoint = Eigen::Vector3d::Zero();
    for (const auto &s : samples)
    {
        const Eigen::Matrix3d R =
            dobotPoseToRotationMatrixDeg(s.flange.rx, s.flange.ry, s.flange.rz);
        const Eigen::Vector3d pFlange(s.flange.x, s.flange.y, s.flange.z);
        contactPoint += pFlange + R * tcpBootstrap;
    }
    contactPoint /= static_cast<double>(samples.size());

    Eigen::Vector3d tcp = tcpBootstrap;
    for (int iter = 0; iter < 20; ++iter)
    {
        Eigen::Vector3d tcpSum = Eigen::Vector3d::Zero();
        for (const auto &s : samples)
        {
            const Eigen::Matrix3d R =
                dobotPoseToRotationMatrixDeg(s.flange.rx, s.flange.ry, s.flange.rz);
            const Eigen::Vector3d pFlange(s.flange.x, s.flange.y, s.flange.z);
            tcpSum += R.transpose() * (contactPoint - pFlange);
        }
        const Eigen::Vector3d tcpNew = tcpSum / static_cast<double>(samples.size());

        Eigen::Vector3d contactSum = Eigen::Vector3d::Zero();
        for (const auto &s : samples)
        {
            const Eigen::Matrix3d R =
                dobotPoseToRotationMatrixDeg(s.flange.rx, s.flange.ry, s.flange.rz);
            const Eigen::Vector3d pFlange(s.flange.x, s.flange.y, s.flange.z);
            contactSum += pFlange + R * tcpNew;
        }
        const Eigen::Vector3d contactNew = contactSum / static_cast<double>(samples.size());

        if ((tcpNew - tcp).norm() < 1e-6 && (contactNew - contactPoint).norm() < 1e-6)
        {
            tcp = tcpNew;
            contactPoint = contactNew;
            break;
        }
        tcp = tcpNew;
        contactPoint = contactNew;
    }

    std::vector<Eigen::Vector3d> tcpPerPose;
    tcpPerPose.reserve(samples.size());
    Eigen::Vector3d tipMin = Eigen::Vector3d::Zero();
    Eigen::Vector3d tipMax = Eigen::Vector3d::Zero();
    Eigen::Vector3d contactMin = Eigen::Vector3d::Zero();
    Eigen::Vector3d contactMax = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < samples.size(); ++i)
    {
        const auto &s = samples[i];
        const Eigen::Matrix3d R =
            dobotPoseToRotationMatrixDeg(s.flange.rx, s.flange.ry, s.flange.rz);
        const Eigen::Vector3d pFlange(s.flange.x, s.flange.y, s.flange.z);
        const Eigen::Vector3d tipWorld = s.tipBaseMm;
        const Eigen::Vector3d contact = pFlange + R * tcp;
        const Eigen::Vector3d tcpI = R.transpose() * (contactPoint - pFlange);
        tcpPerPose.push_back(tcpI);

        if (i == 0)
        {
            tipMin = tipWorld;
            tipMax = tipWorld;
            contactMin = contact;
            contactMax = contact;
        }
        else
        {
            tipMin = tipMin.cwiseMin(tipWorld);
            tipMax = tipMax.cwiseMax(tipWorld);
            contactMin = contactMin.cwiseMin(contact);
            contactMax = contactMax.cwiseMax(contact);
        }
    }

    Eigen::Vector3d tcpVar = Eigen::Vector3d::Zero();
    for (const auto &t : tcpPerPose)
    {
        const Eigen::Vector3d d = t - tcp;
        tcpVar += d.cwiseProduct(d);
    }
    if (tcpPerPose.size() > 1)
    {
        tcpVar /= static_cast<double>(tcpPerPose.size() - 1);
    }
    const Eigen::Vector3d tcpStd = tcpVar.cwiseSqrt();

    outResult.tcpFlange = tcp;
    outResult.contactMeanBaseMm = contactPoint;
    outResult.tipSpreadMm = tipMax - tipMin;
    outResult.maxTipSpreadMm = outResult.tipSpreadMm.maxCoeff();
    outResult.contactSpreadMm = contactMax - contactMin;
    outResult.maxContactSpreadMm = outResult.contactSpreadMm.maxCoeff();
    outResult.tcpStdDevMm = tcpStd;
    outResult.alignmentWarning =
        outResult.maxTipSpreadMm > 5.0 || outResult.maxContactSpreadMm > 5.0;

    if (!tcp.allFinite())
    {
        if (outError)
            *outError = "四点 TCP 求解出现 NaN/Inf";
        return false;
    }

    if (tcp.cwiseAbs().maxCoeff() > 800.0)
    {
        if (outError)
            *outError = "TCP 结果超出合理范围(>|800|mm)，请检查姿态或旋转矩阵约定";
        return false;
    }

    if (tcpStd.maxCoeff() > 15.0)
    {
        if (outError)
            *outError = "各姿态反算 TCP 标准差过大 (max " + std::to_string(tcpStd.maxCoeff()) +
                        " mm)，请检查四点姿态";
        return false;
    }

    outResult.valid = true;
    return true;
}

void printFourPointTcpReport(const std::vector<FourPointTcpSample> &samples,
                             const std::vector<std::string> &labels,
                             const FourPointTcpResult &result)
{
    std::cout << "\n========== 四点法 TCP 标定报告 ==========\n";
    std::cout << "标定记录: 微调后法兰 GetPose(user=0, tool=0)\n";
    std::cout << "四点算法: 由四法兰姿态迭代求 t，使 p_i+R_i*t 收敛到同一点 P（mm/° ZYX）\n";
    std::cout << "单位: 位置 mm，Rx/Ry/Rz 为角度 ° (Dobot 笛卡尔 GetPose)\n";

    for (size_t i = 0; i < samples.size(); ++i)
    {
        const char *name = (i < labels.size()) ? labels[i].c_str() : "姿态";
        const auto &f = samples[i].flange;
        const auto &tip = samples[i].tipBaseMm;
        const Eigen::Matrix3d R = dobotPoseToRotationMatrixDeg(f.rx, f.ry, f.rz);
        const Eigen::Vector3d pFlange(f.x, f.y, f.z);
        const Eigen::Vector3d contact = pFlange + R * result.tcpFlange;
        const Eigen::Vector3d tcpI = R.transpose() * (result.contactMeanBaseMm - pFlange);

        std::cout << "\n姿态 " << (i + 1) << " [" << name << "]\n";
        std::cout << "  法兰 tool0: " << f.x << ", " << f.y << ", " << f.z << ", "
                  << f.rx << ", " << f.ry << ", " << f.rz << "\n";
        std::cout << "  刷尖 tool5: " << tip.x() << ", " << tip.y() << ", " << tip.z()
                  << " (当前 TCP 正解，仅供参考)\n";
        std::cout << "  反算接触点: " << contact.transpose() << " mm\n";
        std::cout << "  反算 TCP:   " << tcpI.transpose() << " (法兰系, mm)\n";
    }

    Eigen::Vector3d tipMean = Eigen::Vector3d::Zero();
    for (const auto &s : samples)
    {
        tipMean += s.tipBaseMm;
    }
    if (!samples.empty())
    {
        tipMean /= static_cast<double>(samples.size());
    }

    std::cout << "\n--- 刷尖 tool5 一致性 (当前 TCP 下，微调后应接近) ---\n";
    std::cout << "  平均刷尖:  " << tipMean.transpose() << " mm\n";
    std::cout << "  刷尖极差: " << result.tipSpreadMm.transpose() << " mm (max "
              << result.maxTipSpreadMm << " mm)\n";

    std::cout << "\n--- 四点法反算接触点一致性 ---\n";
    std::cout << "  平均接触点: " << result.contactMeanBaseMm.transpose() << " mm\n";
    std::cout << "  接触点极差: " << result.contactSpreadMm.transpose() << " mm (max "
              << result.maxContactSpreadMm << " mm)\n";

    if (result.alignmentWarning)
    {
        std::cout << "\n[警告] 四姿态未完全对准同一点（常见于仅对部分侧面做了键盘微调）。\n";
        std::cout << "       旋转偏移已写入 rotationoffset.json；TCP 结果仍可参考，请谨慎应用。\n";
    }

    const Eigen::Vector3d &tcpFlange = result.tcpFlange;

    std::cout << "\n--- 四点平均 TCP (法兰坐标系, mm) ---\n";
    std::cout << "  X: " << tcpFlange.x() << "\n";
    std::cout << "  Y: " << tcpFlange.y() << "\n";
    std::cout << "  Z: " << tcpFlange.z() << "\n";
    std::cout << "  各姿态反算标准差: " << result.tcpStdDevMm.transpose() << " mm\n";

    std::cout << "\n--- 当前 baseline TCP ---\n";
    std::cout << "  {" << kTcpBaselineX << ", " << kTcpBaselineY << ", " << kTcpBaselineZ << ", 0, 0, 0}\n";

    std::cout << "\n--- 与 baseline 的差值 (计算值 - baseline) ---\n";
    std::cout << "  dX: " << (tcpFlange.x() - kTcpBaselineX)
              << "  dY: " << (tcpFlange.y() - kTcpBaselineY)
              << "  dZ: " << (tcpFlange.z() - kTcpBaselineZ) << "\n";

    std::cout << "\n--- 将写入 setTool(5) ---\n";
    std::cout << "  {" << tcpFlange.x() << ", " << tcpFlange.y() << ", "
              << tcpFlange.z() << ", 0, 0, 0}\n";
    std::cout << "========================================\n";
}

// 四点 TCP 标定成功后：直接替换 tool5 并写入 brushoffsets.json（不再询问 y/n）
bool applyAndSaveFourPointTcp(DobotTcpDemo *demo,
                              const std::vector<FourPointTcpSample> &samples,
                              const std::vector<std::string> &labels,
                              const FourPointTcpResult &result,
                              const std::string &brushOffsetPath)
{
    printFourPointTcpReport(samples, labels, result);

    if (!result.valid)
    {
        std::cout << "结果未通过有效性检查，不会写入 TCP。\n";
        return false;
    }

    if (result.alignmentWarning)
    {
        std::cout << "[警告] 刷尖/接触点极差偏大，仍将应用四点法 TCP 结果供精度验证。\n";
    }

    json offsetJson;
    std::ifstream inFile(brushOffsetPath);
    if (inFile.is_open())
    {
        inFile >> offsetJson;
        inFile.close();
    }

    const double offsetXs = kTcpBaselineX - result.tcpFlange.x();
    const double offsetYs = kTcpBaselineY - result.tcpFlange.y();
    const double offsetZs = kTcpBaselineZ - result.tcpFlange.z();

    offsetJson["brushxoffsets"] = offsetXs;
    offsetJson["brushyoffsets"] = offsetYs;
    offsetJson["brushzoffsets"] = offsetZs;
    offsetJson["tcp_four_point_x"] = result.tcpFlange.x();
    offsetJson["tcp_four_point_y"] = result.tcpFlange.y();
    offsetJson["tcp_four_point_z"] = result.tcpFlange.z();
    offsetJson["tcp_four_point_method"] = "4-point flange iterative (tool0, deg ZYX)";
    offsetJson["tcp_four_point_tip_spread_mm"] = result.maxTipSpreadMm;
    offsetJson["tcp_four_point_contact_spread_mm"] = result.maxContactSpreadMm;
    offsetJson["tcp_four_point_alignment_warning"] = result.alignmentWarning;
    offsetJson["tcp_four_point_std_mm"] = {result.tcpStdDevMm.x(), result.tcpStdDevMm.y(), result.tcpStdDevMm.z()};
    offsetJson["tcp_four_point_applied"] = true;

    std::ofstream outFile(brushOffsetPath);
    if (!outFile.is_open())
    {
        std::cerr << "无法写入: " << brushOffsetPath << std::endl;
        return false;
    }
    outFile << offsetJson.dump(4);
    outFile.close();

    const std::string tcpvalue = "{" + std::to_string(result.tcpFlange.x()) + "," +
                                 std::to_string(result.tcpFlange.y()) + "," +
                                 std::to_string(result.tcpFlange.z()) + ",0,0,0}";
    demo->setToolDemo(5, tcpvalue);

    std::cout << "四点法 TCP 已自动替换 tool5 并保存到 " << brushOffsetPath << std::endl;
    std::cout << "brushxoffsets=" << offsetXs << ", brushyoffsets=" << offsetYs
              << ", brushzoffsets=" << offsetZs << std::endl;
    std::cout << "setTool(5): " << tcpvalue << std::endl;
    return true;
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

/* ======================= 运行模式 ======================= */
// 1=基础轨迹配置模式  2=随机模式(保留全部交互)  3=轨迹复用模式
struct ModePlan
{
    int mode = 2;
    bool gotoCalib = false;   // 到达标定点(不做TCP标定)
    bool generateNew = false; // 生成新标准轨迹(command1) + 初始姿态(H) + command3
    bool saveStdJson = false; // 微调后保存标准轨迹为 json
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

// 将标准轨迹(末端位姿点)保存为 json，供其它代码/模式3复用
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

// 读取 json 标准轨迹到末端位姿点
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
    const std::string Force_FILE_PATH = "../defaultconfig/rightup/upright.txt";
    // [MOD-FORCE-LOG] 完整力控轨迹(movsDemoC)阶段的力数据日志路径
    const std::string Movs_FORCE_LOG_PATH = "../defaultconfig/rightup/movs_force_during_movss.txt";
    const std::string Movs_FORCE_LOG_PATHs = "../defaultconfig/rightup/movs_force_during_movs.txt";
    // const std::string Brush_offset = "../defaultconfig/rightup/brushoffsets.json";
    const std::string Brush_offset = "../defaultconfig/brushoffsets.json";
    const std::string Rotation_offset = "../defaultconfig/rotationoffset.json";
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
    rotatetooljoint.x = 0.001;
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
    // double modifiedupz = 101.3919;
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

        double tcpx = -9.748236 - offsetXs;
        double tcpy = -186.312977 - offsetYs;
        double tcpz = 223.252632 - offsetZs;

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
    std::cout << "是否需要到达标定位置？(y/n)" << std::endl;
    char userInputcalibrate;
    std::cin >> userInputcalibrate;

    Dobot::CDescartesPoint brushcalibratepoint{};
    brushcalibratepoint.x = 271.6384;
    brushcalibratepoint.y = -275.2379;
    brushcalibratepoint.z = 462.4796;
    brushcalibratepoint.rx = -179.7725;
    brushcalibratepoint.ry = -1.3507;
    brushcalibratepoint.rz = -145.9055;

    if (userInputcalibrate == 'y' || userInputcalibrate == 'Y')
    {
        demo->moveRobotC(brushcalibratepoint, brushcalibratepoint);
        std::cout << "是否到达标定位置？(y/n)" << std::endl;
        char userInputcalibrateok;
        std::cin >> userInputcalibrateok;
        if (userInputcalibrateok == 'y' || userInputcalibrateok == 'Y')
        {
            std::cout << "到达标定位置,请开始下一步" << std::endl;

            std::cout << "是否进行侧面姿态旋转偏移标定？(y/n): ";
            char userInputRotationCalibrate;
            std::cin >> userInputRotationCalibrate;
            if (userInputRotationCalibrate == 'y' || userInputRotationCalibrate == 'Y')
            {
                std::string tcpBaseline = "{-9.748236,-186.312977,223.252632,0,0,0}";
                demo->setToolDemo(5, tcpBaseline);

                Dobot::CDescartesPoint uprightRot{};
                uprightRot.x = 0.001;
                uprightRot.y = 0;
                uprightRot.z = 0;
                uprightRot.rx = 0;
                uprightRot.ry = 0;
                uprightRot.rz = 0;

                Dobot::CDescartesPoint sideRightRot{};
                sideRightRot.x = 0.0;
                sideRightRot.y = 0;
                sideRightRot.z = 0;
                sideRightRot.rx = 0;
                sideRightRot.ry = -45;
                sideRightRot.rz = 0;

                Dobot::CDescartesPoint sideLeftRot{};
                sideLeftRot.x = 0.1;
                sideLeftRot.y = 0;
                sideLeftRot.z = 0;
                sideLeftRot.rx = 0;
                sideLeftRot.ry = 45;
                sideLeftRot.rz = 0;

                Dobot::CDescartesPoint sideAheadRot{};
                sideAheadRot.x = 0;
                sideAheadRot.y = 0;
                sideAheadRot.z = 0;
                sideAheadRot.rx = 0;
                sideAheadRot.ry = -48;
                sideAheadRot.rz = 0;

                json rotationJson;
                std::ifstream rotIn(Rotation_offset);
                if (rotIn.is_open())
                {
                    rotIn >> rotationJson;
                    rotIn.close();
                }

                std::vector<FourPointTcpSample> tcpSamples;
                std::vector<std::string> tcpCalibLabels;
                tcpSamples.reserve(4);
                tcpCalibLabels.reserve(4);

                FourPointTcpSample sample{};

                saveRotationOffsetEntry(
                    rotationJson,
                    "uprightrotateoffset",
                    recordSideRotationOffset(demo, brushcalibratepoint, uprightRot, "正立(UpRight)",
                                             &sample.flange, &sample.tipBaseMm));
                tcpSamples.push_back(sample);
                tcpCalibLabels.push_back("正立(UpRight)");

                saveRotationOffsetEntry(
                    rotationJson,
                    "siderightrotateoffset",
                    recordSideRotationOffset(demo, brushcalibratepoint, sideRightRot, "右侧面(SideRight)",
                                             &sample.flange, &sample.tipBaseMm));
                tcpSamples.push_back(sample);
                tcpCalibLabels.push_back("右侧面(SideRight)");

                saveRotationOffsetEntry(
                    rotationJson,
                    "sideleftrotateoffset",
                    recordSideRotationOffset(demo, brushcalibratepoint, sideLeftRot, "左侧面(SideLeft)",
                                             &sample.flange, &sample.tipBaseMm));
                tcpSamples.push_back(sample);
                tcpCalibLabels.push_back("左侧面(SideLeft)");

                saveRotationOffsetEntry(
                    rotationJson,
                    "sideaheadrotateoffset",
                    recordSideRotationOffset(demo, brushcalibratepoint, sideAheadRot, "前侧面(SideAhead)",
                                             &sample.flange, &sample.tipBaseMm));
                tcpSamples.push_back(sample);
                tcpCalibLabels.push_back("前侧面(SideAhead)");

                std::ofstream rotOut(Rotation_offset);
                if (rotOut.is_open())
                {
                    rotOut << rotationJson.dump(4);
                    rotOut.close();
                    std::cout << "侧面旋转偏移已保存: " << Rotation_offset << std::endl;
                }
                else
                {
                    std::cerr << "无法写入: " << Rotation_offset << std::endl;
                }

                FourPointTcpResult tcpResult{};
                std::string tcpError;
                if (computeFourPointTcp(tcpSamples, tcpResult, &tcpError))
                {
                    applyAndSaveFourPointTcp(demo, tcpSamples, tcpCalibLabels, tcpResult, Brush_offset);
                }
                else
                {
                    std::cerr << "四点 TCP 计算失败: " << tcpError << std::endl;
                    if (tcpResult.tcpFlange.allFinite() && tcpResult.tcpFlange.cwiseAbs().maxCoeff() > 0)
                    {
                        tcpResult.valid = false;
                        printFourPointTcpReport(tcpSamples, tcpCalibLabels, tcpResult);
                    }
                }

                demo->moveRobotC(brushcalibratepoint, brushcalibratepoint);
            }
        }
    }

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
        Eigen::Vector3d totalOffset = getManualOffset2(demo, REF_X, REF_Y, REF_Z);
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

        double tcpx = -9.748236 + vecB.x;
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

        std::cout << "新的tcp为：" << tcpvalue << std::endl;

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
            pointa.x = 264.8929 + modifiedupx + offsetX;
            pointa.y = -285.18520 + modifiedupy + offsetY;
            pointa.z = 391.0669 + modifiedupz + modifiedup + offsetZ;
            pointa.rx = -179.7725;
            pointa.ry = -1.3507;
            pointa.rz = -145.9055;
            demo->moveRobotC(pointa, pointa);

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

    double offsetXs = loadedJson.value("brushxoffsets", 0.0);
    double offsetYs = loadedJson.value("brushyoffsets", 0.0);
    double offsetZs = loadedJson.value("brushzoffsets", 0.0);

    // 重复调整轨迹偏差（对调整好的轨迹做微小补偿）
    double modifyoffsetX = 0;
    double modifyoffsetY = 0;
    double modifyoffsetZ = 0;

    // bool userSatisfied = false;
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

        // 将每次调整的坐标应用到tcp上去
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

        std::cout << "临时偏移量更新为：" << std::endl;
        std::cout << "Xc: " << offsetXs << "\nYc: " << offsetYs << "\nZc: " << offsetZs << std::endl;

        //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@2
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

            // 用户满意后，统一保存偏移量到文件
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
        const std::string Std_Traj_Json = "../defaultconfig/rightup/standard_trajectory.json";

        // ---- C 到达标定点(不做TCP标定) ----
        if (plan.gotoCalib)
        {
            Dobot::CDescartesPoint brushcalibratepoint{};
            brushcalibratepoint.x = 271.6384;
            brushcalibratepoint.y = -275.2379;
            brushcalibratepoint.z = 462.4796;
            brushcalibratepoint.rx = -179.7725;
            brushcalibratepoint.ry = -1.3507;
            brushcalibratepoint.rz = -145.9055;
            std::cout << "[模式" << mode << "] 前往标定位置(不做TCP标定), 注意安全..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            demo->moveRobotC(brushcalibratepoint, brushcalibratepoint);
        }

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

            double tcpx = -9.748236 - offsetXs;
            double tcpy = -186.312977 - offsetYs;
            double tcpz = 223.252632 - offsetZs;
            std::string tcpvalue = "{" + std::to_string(tcpx) + "," +
                                   std::to_string(tcpy) + "," +
                                   std::to_string(tcpz) + ",0,0,0}";
            demo->setToolDemo(5, tcpvalue);
            std::cout << "[模式" << mode << "] 已加载牙刷偏移并设置TCP: " << tcpvalue << std::endl;
        }

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
            // ---- 模式1: G 生成新标准轨迹 ----
            std::cout << "[模式1] 生成新轨迹..." << std::endl;
            int rc = std::system(command1.c_str());
            if (rc != 0)
            {
                std::cout << "[模式1] 新轨迹生成失败！" << std::endl;
                return -1;
            }
            std::cout << "[模式1] 新轨迹生成成功！" << std::endl;

            // ---- 模式1: H 牙刷初始旋转姿态调整(自动, 去Enter, 加延时+提示) ----
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

            // 记录真实位姿(供 command3 转换轨迹使用)
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

            // ---- 模式1: H2 轨迹转换到机械臂末端 ----
            std::cout << "[模式1] 将轨迹转换到机械臂末端..." << std::endl;
            std::system(command3.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            firstPoseback.z += 20;
            demo->moveRobotC(firstPoseback, firstPoseback);

            // ---- 模式1: 使用新生成的末端轨迹 ----
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
            std::cout << "[模式3] 复用标准轨迹(json): " << Std_Traj_Json << std::endl;
            if (!loadStandardTrajectoryJson(Std_Traj_Json, brushpointsoffset_ee_poses))
            {
                std::cerr << "无法加载标准轨迹json(请先用模式1生成): " << Std_Traj_Json << std::endl;
                return -1;
            }
            backupForceTrajectoryFile(Force_FILE_PATH);
            indexFilePath = oldsegment; // 复用分段索引
        }

        std::cout << "已加载轨迹 " << brushpointsoffset_ee_poses.size() << " 点" << std::endl;
        if (brushpointsoffset_ee_poses.empty())
        {
            std::cerr << "轨迹为空！" << std::endl;
            return -1;
        }

        // ---- K 轨迹微调循环(模式1/3都做) ----
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
            if (targetforcevalue == 0)
            {
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
                std::thread forceSampler([&]()
                                         {
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
                    } });
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
