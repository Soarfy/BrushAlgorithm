#pragma once

#include "DobotTcpDemo.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr double kBrushTcpRetreatMm = 30.0;
constexpr double kBrushBaseLiftMm = 80.0;

inline double brushDegToRad(double degrees)
{
    return degrees * M_PI / 180.0;
}

inline Eigen::Matrix3d brushEulerDegToRotationMatrix(double rx_deg, double ry_deg, double rz_deg)
{
    double rx = brushDegToRad(rx_deg);
    double ry = brushDegToRad(ry_deg);
    double rz = brushDegToRad(rz_deg);
    Eigen::AngleAxisd rollAngle(rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rz, Eigen::Vector3d::UnitZ());
    return (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
}

inline bool waitValidCurrentPose(DobotTcpDemo *demo, double &gx, double &gy, double &gz,
                                 double &grx, double &gry, double &grz, int maxWaitMs = 15000)
{
    int waited = 0;
    while (waited < maxWaitMs)
    {
        if (demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz) &&
            !std::isnan(gx) && !std::isnan(gy) && !std::isnan(gz) &&
            !std::isnan(grx) && !std::isnan(gry) && !std::isnan(grz))
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        waited += 60;
    }
    std::cerr << "获取当前位姿超时 (" << maxWaitMs << " ms)\n";
    return false;
}

inline void retreatTcpZThenLiftBaseZ(DobotTcpDemo *demo, const Dobot::CDescartesPoint &pt,
                                   double tcpRetreatMm = kBrushTcpRetreatMm,
                                   double baseLiftMm = kBrushBaseLiftMm)
{
    Eigen::Matrix3d rot = brushEulerDegToRotationMatrix(pt.rx, pt.ry, pt.rz);
    Eigen::Vector3d brushDir = rot.col(2);
    brushDir.normalize();

    Dobot::CDescartesPoint leavePt = pt;
    leavePt.x += -brushDir.x() * tcpRetreatMm;
    leavePt.y += -brushDir.y() * tcpRetreatMm;
    leavePt.z += -brushDir.z() * tcpRetreatMm;
    demo->moveRobotC(leavePt, leavePt);

    double gx, gy, gz, grx, gry, grz;
    if (waitValidCurrentPose(demo, gx, gy, gz, grx, gry, grz))
    {
        Dobot::CDescartesPoint lifted{};
        lifted.x = gx;
        lifted.y = gy;
        lifted.z = gz + baseLiftMm;
        lifted.rx = grx;
        lifted.ry = gry;
        lifted.rz = grz;
        demo->moveRobotC(lifted, lifted);
    }
}

inline void retreatFromCurrentPose(DobotTcpDemo *demo, double tcpRetreatMm = kBrushTcpRetreatMm,
                                   double baseLiftMm = kBrushBaseLiftMm)
{
    double gx, gy, gz, grx, gry, grz;
    if (!waitValidCurrentPose(demo, gx, gy, gz, grx, gry, grz))
        return;
    Dobot::CDescartesPoint pt{};
    pt.x = gx;
    pt.y = gy;
    pt.z = gz;
    pt.rx = grx;
    pt.ry = gry;
    pt.rz = grz;
    retreatTcpZThenLiftBaseZ(demo, pt, tcpRetreatMm, baseLiftMm);
}

inline void liftRotateApproachFirstPoint(DobotTcpDemo *demo,
                                         const Dobot::CDescartesPoint &pointsafe,
                                         Dobot::CDescartesPoint rotatetooljointjump,
                                         Dobot::CDescartesPoint rotatetooljoint,
                                         const Dobot::CDescartesPoint &firstPoint,
                                         double approachOffsetMm = 8.0)
{
    demo->moveRobotC(pointsafe, pointsafe);
    std::cout << "初始位：先上抬再旋转，前往轨迹起点..." << std::endl;
    demo->RelMovJDemo(rotatetooljointjump, 0, 5, 20, 50, 100);
    demo->RelMovJDemo(rotatetooljoint, 0, 5, 20, 50, 100);

    Eigen::Matrix3d rot = brushEulerDegToRotationMatrix(firstPoint.rx, firstPoint.ry, firstPoint.rz);
    Eigen::Vector3d brushDir = rot.col(2);
    brushDir.normalize();

    Dobot::CDescartesPoint approach = firstPoint;
    approach.x += -brushDir.x() * approachOffsetMm;
    approach.y += -brushDir.y() * approachOffsetMm;
    approach.z += -brushDir.z() * approachOffsetMm;

    demo->moveRobotC(approach, approach);
    demo->moveRobotC(firstPoint, firstPoint);
}
