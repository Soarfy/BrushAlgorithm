#include <cmath>
#include <conio.h>
#include <crtdbg.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <limits>
#include "DobotTcpDemo.h"
#include <windows.h>
#include <string>
#include "nlohmann/json.hpp"
#include "ForceTrajectoryIO.h"
using json = nlohmann::json;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct TrajectoryPoint
{
    double x, y, z, a, b, c;
};

enum class BrushRunMode
{
    UpLike,
    SideRightLike,
    SideNegativeXApproach,
    InsideLeftApproach,
    CenterPointwise,
    SideAheadLike,
};

struct RegionBrushConfig
{
    const char *name;
    const char *forcePath;
    const char *indexPath;
    BrushRunMode mode;
    double modifiedup;
    double modifiedupx;
    double modifiedupy;
    double modifiedupz;
    Dobot::CDescartesPoint rotatetooljoint;
    Dobot::CDescartesPoint rotatetooljointjump;
    bool usePreJumpSequence;
    double approachBackMm;
    bool useSegmentBetween;
    Dobot::CDescartesPoint segmentBetween;
    bool useLastLeave;
    Dobot::CDescartesPoint lastLeave;
    bool usePostUp;
    Dobot::CDescartesPoint postUp;
    const char *pickedPathFile;
    const char *pickedPathFallback;
    bool useExitWaypoints;
    bool useEntryWaypoints;
    bool exitRotateAfterSafe;
    double entryLiftMm;
    Dobot::CDescartesPoint guideExitPose;
    Dobot::CDescartesPoint pointpartone;
    Dobot::CDescartesPoint pointtwo;
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
    return (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
}

Dobot::CDescartesPoint makeSafePoint()
{
    Dobot::CDescartesPoint pointsafe{};
    pointsafe.x = 264.8929;
    pointsafe.y = -285.1852;
    pointsafe.z = 491.0669;
    pointsafe.rx = -179.7725;
    pointsafe.ry = -1.3507;
    pointsafe.rz = -145.9055;
    return pointsafe;
}

Dobot::CDescartesPoint makePointA(const RegionBrushConfig &cfg)
{
    Dobot::CDescartesPoint pointa{};
    pointa.x = 264.8929 + cfg.modifiedupx;
    pointa.y = -285.1852 + cfg.modifiedupy;
    pointa.z = 391.0669 + cfg.modifiedup + cfg.modifiedupz;
    pointa.rx = -179.7725;
    pointa.ry = -1.3507;
    pointa.rz = -145.9055;
    return pointa;
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

bool loadTrajectoryPoints(const std::string &filePath, std::vector<Dobot::CDescartesPoint> &out)
{
    std::vector<TrajectoryPoint> raw;
    if (!loadForceTrajectoryFile(filePath, raw))
        return false;

    out.clear();
    out.reserve(raw.size());
    for (const auto &p : raw)
    {
        Dobot::CDescartesPoint cp{};
        cp.x = p.x;
        cp.y = p.y;
        cp.z = p.z;
        cp.rx = p.a;
        cp.ry = p.b;
        cp.rz = p.c;
        out.push_back(cp);
    }
    return !out.empty();
}

bool loadIndexLines(const std::string &indexPath, std::vector<std::string> &lines)
{
    std::ifstream indexFile(indexPath);
    if (!indexFile.is_open())
    {
        std::cerr << "无法打开索引文件: " << indexPath << std::endl;
        return false;
    }
    std::string line;
    while (std::getline(indexFile, line))
    {
        if (!line.empty())
            lines.push_back(line);
    }
    return !lines.empty();
}

bool loadPickedPathPointsFromFile(const std::string &path, std::vector<Dobot::CDescartesPoint> &out)
{
    std::ifstream in(path);
    if (!in.is_open())
        return false;

    out.clear();
    double x, y, z, rx, ry, rz;
    while (in >> x >> y >> z >> rx >> ry >> rz)
    {
        Dobot::CDescartesPoint cp{};
        cp.x = x;
        cp.y = y;
        cp.z = z + 30;
        cp.rx = rx;
        cp.ry = ry;
        cp.rz = rz;
        out.push_back(cp);
    }
    return !out.empty();
}

bool loadPickedPathPoints(const char *primaryPath,
                          const char *fallbackPath,
                          std::vector<Dobot::CDescartesPoint> &out,
                          std::string &loadedFrom)
{
    if (primaryPath != nullptr && loadPickedPathPointsFromFile(primaryPath, out))
    {
        loadedFrom = primaryPath;
        return true;
    }
    if (fallbackPath != nullptr && loadPickedPathPointsFromFile(fallbackPath, out))
    {
        loadedFrom = fallbackPath;
        return true;
    }
    return false;
}

void logCurrentPose(DobotTcpDemo *demo, const char *tag)
{
    double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "[POSE] " << tag << std::endl;
    if (demo->getCurrentPose(0, 0, x, y, z, rx, ry, rz))
    {
        std::cout << "  base user=0 tool=0: xyz=(" << x << ", " << y << ", " << z
                  << ") rpy=(" << rx << ", " << ry << ", " << rz << ")" << std::endl;
    }
    else
    {
        std::cerr << "  base user=0 tool=0: GetPose FAILED" << std::endl;
    }
    if (demo->getCurrentPose(0, 5, x, y, z, rx, ry, rz))
    {
        std::cout << "  brush user=0 tool=5: xyz=(" << x << ", " << y << ", " << z
                  << ") rpy=(" << rx << ", " << ry << ", " << rz << ")" << std::endl;
    }
    else
    {
        std::cerr << "  brush user=0 tool=5: GetPose FAILED (tool5 可能未设置)" << std::endl;
    }
    std::cout << std::flush;
}

void relMovJWithLog(DobotTcpDemo *demo,
                    const char *tag,
                    Dobot::CDescartesPoint &pt,
                    int user,
                    int tool,
                    int a,
                    int v,
                    int cp)
{
    double zBefore = 0;
    double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
    if (demo->getCurrentPose(0, 0, x, y, zBefore, rx, ry, rz))
        std::cout << "[RelMovJTool] " << tag << " 发送指令 user=" << user << " tool=" << tool
                  << " d=(" << pt.x << ", " << pt.y << ", " << pt.z << ", "
                  << pt.rx << ", " << pt.ry << ", " << pt.rz << ")"
                  << " | 当前 base Z=" << zBefore << std::endl;
    else
        std::cout << "[RelMovJTool] " << tag << " 发送指令 (GetPose 失败，无法读当前 Z)" << std::endl;

    logCurrentPose(demo, (std::string(tag) + " BEFORE").c_str());
    demo->RelMovJDemo(pt, user, tool, a, v, cp);
    logCurrentPose(demo, (std::string(tag) + " AFTER").c_str());

    if (demo->getCurrentPose(0, 0, x, y, z, rx, ry, rz))
    {
        const double dz = z - zBefore;
        std::cout << "[DELTA] " << tag << " base Z 变化: " << dz << " mm"
                  << " (前=" << zBefore << " 后=" << z << ")" << std::endl;
        if (std::abs(dz) < 0.5 && (std::abs(pt.z) > 0.5 || std::abs(pt.x) > 0.5 || std::abs(pt.y) > 0.5))
            std::cerr << "[WARN] " << tag << " 相对运动指令已发但 base Z 几乎未变，请检查 tool/user 或 RelMovJTool 是否失败"
                      << std::endl;
    }
    std::cout << std::flush;
}

void liftRelativeMm(DobotTcpDemo *demo, double mm, const char *tag)
{
    Dobot::CDescartesPoint liftRel{};
    liftRel.x = 0;
    liftRel.y = 0;
    liftRel.z = -mm;
    liftRel.rx = 0;
    liftRel.ry = 0;
    liftRel.rz = 0;
    relMovJWithLog(demo, tag, liftRel, 0, 5, 20, 50, 100);
}

void moveToSafeViaWaypoints(DobotTcpDemo *demo,
                            const Dobot::CDescartesPoint &pointpartone,
                            const Dobot::CDescartesPoint &pointtwo,
                            const Dobot::CDescartesPoint &pointsafe);

void prepareRegionEntry(DobotTcpDemo *demo,
                        const RegionBrushConfig &cfg,
                        const Dobot::CDescartesPoint &pointsafe)
{
    std::cout << "\n[ENTRY] ===== " << cfg.name << " 入点开始 =====" << std::endl;
    std::cout << "[ENTRY] mode=" << static_cast<int>(cfg.mode)
              << " usePreJump=" << cfg.usePreJumpSequence
              << " entryLiftMm=" << cfg.entryLiftMm
              << " useEntryWaypoints=" << cfg.useEntryWaypoints << std::endl;
    logCurrentPose(demo, "ENTRY 起始");

    if (cfg.mode == BrushRunMode::SideAheadLike)
    {
        if (cfg.useEntryWaypoints)
        {
            std::cout << "[" << cfg.name << "] 经安全途经点 → pointsafe..." << std::endl;
            logCurrentPose(demo, "途经点 pointpartone 前");
            demo->moveRobotC(cfg.pointpartone, cfg.pointpartone);
            logCurrentPose(demo, "到达 pointpartone");
            demo->moveRobotC(cfg.pointtwo, cfg.pointtwo);
            logCurrentPose(demo, "到达 pointtwo");
            demo->moveRobotC(pointsafe, pointsafe);
            logCurrentPose(demo, "到达 pointsafe");
        }
        else
        {
            std::cout << "[" << cfg.name << "] 回安全点..." << std::endl;
            demo->moveRobotC(pointsafe, pointsafe);
            logCurrentPose(demo, "到达 pointsafe");
        }

        if (cfg.entryLiftMm > 0.0)
        {
            std::cout << "[" << cfg.name << "] 上抬 " << cfg.entryLiftMm << "mm..." << std::endl;
            liftRelativeMm(demo, cfg.entryLiftMm, "SideAhead entryLift");
        }
        else
        {
            std::cout << "[WARN] SideAhead entryLiftMm=0，跳过上抬" << std::endl;
        }

        std::cout << "[" << cfg.name << "] 旋转区域姿态..." << std::endl;
        Dobot::CDescartesPoint regionRot = cfg.rotatetooljoint;
        relMovJWithLog(demo, "SideAhead regionRot", regionRot, 0, 5, 20, 50, 100);
        logCurrentPose(demo, "ENTRY 完成");
        return;
    }

    if (cfg.usePreJumpSequence)
    {
        std::cout << "[" << cfg.name << "] 预上抬 rotatetooljointjump z="
                  << cfg.rotatetooljointjump.z << "mm..." << std::endl;
        Dobot::CDescartesPoint preJump = cfg.rotatetooljointjump;
        relMovJWithLog(demo, "preJump rotatetooljointjump", preJump, 0, 5, 20, 50, 100);
    }
    else
    {
        std::cout << "[WARN] " << cfg.name << " usePreJumpSequence=false，跳过预上抬" << std::endl;
    }

    if (cfg.useEntryWaypoints)
    {
        std::cout << "[" << cfg.name << "] 经安全途经点..." << std::endl;
        moveToSafeViaWaypoints(demo, cfg.pointpartone, cfg.pointtwo, pointsafe);
        logCurrentPose(demo, "途经点结束 pointsafe");
    }
    else
    {
        std::cout << "[" << cfg.name << "] 回安全点..." << std::endl;
        demo->moveRobotC(pointsafe, pointsafe);
        logCurrentPose(demo, "到达 pointsafe");
    }

    std::cout << "[" << cfg.name << "] 旋转区域姿态..." << std::endl;
    Dobot::CDescartesPoint regionRot = cfg.rotatetooljoint;
    relMovJWithLog(demo, "regionRot", regionRot, 0, 5, 20, 50, 100);
    logCurrentPose(demo, "ENTRY 完成");
}

void prepareRegionDeparture(DobotTcpDemo *demo,
                            const Dobot::CDescartesPoint &pointsafe,
                            Dobot::CDescartesPoint &regionRot,
                            double liftMm)
{
    demo->moveRobotC(pointsafe, pointsafe);
    liftRelativeMm(demo, liftMm, "prepareRegionDeparture lift");
    relMovJWithLog(demo, "prepareRegionDeparture rot", regionRot, 0, 5, 20, 50, 100);
}

Dobot::CDescartesPoint computeApproachStart(const Dobot::CDescartesPoint &firstPose, double backMm)
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

void moveToApproachPoint(DobotTcpDemo *demo, const Dobot::CDescartesPoint &firstPose, double backMm)
{
    const Dobot::CDescartesPoint pointstart = computeApproachStart(firstPose, backMm);
    std::cout << "[APPROACH] 退 " << backMm << "mm 到 pointstart: ("
              << pointstart.x << ", " << pointstart.y << ", " << pointstart.z << ")" << std::endl;
    logCurrentPose(demo, "APPROACH 前");
    demo->moveRobotC(pointstart, pointstart);
    logCurrentPose(demo, "到达 pointstart");
    demo->moveRobotC(firstPose, firstPose);
    logCurrentPose(demo, "到达轨迹首点");
}

void runPickedPathWithTransition(DobotTcpDemo *demo,
                                 const std::vector<Dobot::CDescartesPoint> &pickedPathPoints,
                                 const Dobot::CDescartesPoint &trajStart,
                                 const Dobot::CDescartesPoint &guideExitPose,
                                 const Dobot::MovSParams &params)
{
    Dobot::MovSParams pickedParams = params;
    pickedParams.tool = 0;
    pickedParams.user = 0;
    pickedParams.v = 80;
    pickedParams.a = 80;
    pickedParams.freq = 0.2;

    std::cout << "[SideAhead] 先到前牙引导轨迹起点..." << std::endl;
    demo->moveRobotC(pickedPathPoints.front(), pickedPathPoints.front());
    std::cout << "[SideAhead] 执行前牙引导轨迹 (" << pickedPathPoints.size() << " 点)..." << std::endl;
    demo->movsDemoC(pickedPathPoints, pickedParams);

    std::cout << "[SideAhead] 引导完成，MovJ 前往过渡位..." << std::endl;
    demo->moveRobotC(guideExitPose, guideExitPose);
    std::cout << "[SideAhead] 过渡位: ("
              << guideExitPose.x << ", " << guideExitPose.y << ", " << guideExitPose.z << ", "
              << guideExitPose.rx << ", " << guideExitPose.ry << ", " << guideExitPose.rz << ")"
              << std::endl;
    std::cout << "[SideAhead] 按 Enter 继续前往刷牙轨迹起点..." << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
    std::cout << "[SideAhead] 前往刷牙轨迹起点 (退 38mm)..." << std::endl;
    demo->moveRobotC(trajStart, trajStart);
}

void moveToSafeViaWaypoints(DobotTcpDemo *demo,
                            const Dobot::CDescartesPoint &pointpartone,
                            const Dobot::CDescartesPoint &pointtwo,
                            const Dobot::CDescartesPoint &pointsafe)
{
    demo->moveRobotC(pointpartone, pointpartone);
    demo->moveRobotC(pointtwo, pointtwo);
    demo->moveRobotC(pointsafe, pointsafe);
}

void executeRegionExit(DobotTcpDemo *demo,
                       const RegionBrushConfig &cfg,
                       const Dobot::CDescartesPoint &pointsafe)
{
    if (cfg.usePostUp)
    {
        std::cout << "[" << cfg.name << "] 轨迹结束，相对上抬 (z=" << cfg.postUp.z << "mm)..." << std::endl;
        Dobot::CDescartesPoint postUp = cfg.postUp;
        relMovJWithLog(demo, "exit postUp", postUp, 0, 0, 20, 50, 100);
    }

    if (cfg.useExitWaypoints)
    {
        std::cout << "[" << cfg.name << "] 经安全途经点退出 → pointsafe..." << std::endl;
        moveToSafeViaWaypoints(demo, cfg.pointpartone, cfg.pointtwo, pointsafe);
    }
    else
    {
        std::cout << "[" << cfg.name << "] 回安全点..." << std::endl;
        demo->moveRobotC(pointsafe, pointsafe);
    }

    if (cfg.mode == BrushRunMode::SideAheadLike)
    {
        std::cout << "[" << cfg.name << "] 退出完成，已回 pointsafe" << std::endl;
    }

    if (cfg.exitRotateAfterSafe)
    {
        std::cout << "[" << cfg.name << "] 安全点恢复区域姿态..." << std::endl;
        Dobot::CDescartesPoint rot = cfg.rotatetooljoint;
        relMovJWithLog(demo, "exit rotateAfterSafe", rot, 0, 5, 20, 50, 100);
    }
}

// 与各区域「运行完整力控轨迹（每行一次movsDemoC）」段内逻辑对齐
void runMovsSegment(DobotTcpDemo *demo,
                    const RegionBrushConfig &cfg,
                    std::vector<Dobot::CDescartesPoint> &selectedPoints,
                    const Dobot::MovSParams &params,
                    bool isLastLine)
{
    if (selectedPoints.empty())
        return;

    switch (cfg.mode)
    {
    case BrushRunMode::SideRightLike:
        // SideRight: 直接到段首点，无 z 抬高/斜切接近
        demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
        break;

    case BrushRunMode::UpLike:
    {
        // UpRight / UpLeft: z+10 → tool5 下压(0,0,-10) → 回到段首
        std::cout << "[SEGMENT][" << cfg.name << "] UpLike 段首 z+10 接近..." << std::endl;
        logCurrentPose(demo, "段首接近前");
        selectedPoints[0].z += 10;
        demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
        logCurrentPose(demo, "段首 z+10 后");
        {
            Dobot::CDescartesPoint jumpIn{0, 0, -10, 0, 0, 0};
            relMovJWithLog(demo, "UpLike segment jumpIn", jumpIn, 0, 5, 20, 50, 100);
        }
        selectedPoints[0].z -= 10;
        demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
        logCurrentPose(demo, "段首接近完成");
        break;
    }

    case BrushRunMode::SideAheadLike:
        // SideAhead: z+50 → (30,0,-30) → 回到段首
        selectedPoints[0].z += 50;
        demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
        {
            Dobot::CDescartesPoint jumpIn{30, 0, -30, 0, 0, 0};
            demo->RelMovJDemo(jumpIn, 0, 5, 20, 50, 100);
        }
        selectedPoints[0].z -= 50;
        demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
        break;

    case BrushRunMode::SideNegativeXApproach:
        // SideLeft / InSideRight: z+10 → (-30,0,-30) → 回到段首
        selectedPoints[0].z += 10;
        demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
        {
            Dobot::CDescartesPoint jumpIn{-30, 0, -30, 0, 0, 0};
            demo->RelMovJDemo(jumpIn, 0, 5, 20, 50, 100);
        }
        selectedPoints[0].z -= 10;
        demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
        break;

    case BrushRunMode::InsideLeftApproach:
        // InSideLeft: z+10 → (30,0,-30) → 回到段首
        selectedPoints[0].z += 10;
        demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
        {
            Dobot::CDescartesPoint jumpIn{30, 0, -30, 0, 0, 0};
            demo->RelMovJDemo(jumpIn, 0, 5, 20, 50, 100);
        }
        selectedPoints[0].z -= 10;
        demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
        break;

    default:
        demo->moveRobotC(selectedPoints[0], selectedPoints[0]);
        break;
    }

    demo->movsDemoC(selectedPoints, params);

    if (!isLastLine && cfg.useSegmentBetween)
    {
        Dobot::CDescartesPoint between = cfg.segmentBetween;
        demo->RelMovJDemo(between, 0, 5, 20, 50, 100);
    }
    else if (isLastLine && cfg.useLastLeave)
    {
        Dobot::CDescartesPoint leave = cfg.lastLeave;
        demo->RelMovJDemo(leave, 0, 5, 20, 50, 100);
    }
}

void runCenterSegment(DobotTcpDemo *demo, const std::vector<Dobot::CDescartesPoint> &selectedPoints)
{
    for (const auto &point : selectedPoints)
    {
        demo->moveRobotC(point, point);

        Dobot::CDescartesPoint wiggle{};
        wiggle.x = 0;
        wiggle.y = -5;
        wiggle.z = 0;
        wiggle.rx = 0;
        wiggle.ry = 0;
        wiggle.rz = 0;
        demo->RelMovJDemo(wiggle, 0, 5, 20, 50, 100);
        wiggle.y = 10;
        demo->RelMovJDemo(wiggle, 0, 5, 20, 50, 100);
    }
}

bool executeRegionTrajectory(DobotTcpDemo *demo,
                             const RegionBrushConfig &cfg,
                             const Dobot::CDescartesPoint &pointsafe,
                             const Dobot::MovSParams &params,
                             const std::string &brushOffsetPath)
{
    std::cout << "\n=== 开始区域: " << cfg.name << " ===" << std::endl;

    if (!applyBrushTcpFromJson(demo, brushOffsetPath))
        return false;
    logCurrentPose(demo, "setTool(5) 后");

    std::vector<Dobot::CDescartesPoint> trajectory;
    if (!loadTrajectoryPoints(cfg.forcePath, trajectory))
    {
        std::cerr << "无法加载力控轨迹: " << cfg.forcePath << std::endl;
        return false;
    }
    std::cout << "读取轨迹点数: " << trajectory.size() << std::endl;

    prepareRegionEntry(demo, cfg, pointsafe);

    if (cfg.mode == BrushRunMode::SideAheadLike)
    {
        if (cfg.pickedPathFile == nullptr)
        {
            std::cerr << "[SideAhead] 未配置引导轨迹文件，无法过渡" << std::endl;
            return false;
        }

        std::vector<Dobot::CDescartesPoint> pickedPathPoints;
        std::string loadedGuidePath;
        if (!loadPickedPathPoints(cfg.pickedPathFile, cfg.pickedPathFallback,
                                  pickedPathPoints, loadedGuidePath))
        {
            std::cerr << "[SideAhead] 无法加载引导轨迹，已尝试:" << std::endl;
            std::cerr << "  - " << cfg.pickedPathFile << std::endl;
            if (cfg.pickedPathFallback != nullptr)
                std::cerr << "  - " << cfg.pickedPathFallback << std::endl;
            std::cerr << "请确认 exe 工作目录下存在上述文件之一" << std::endl;
            return false;
        }
        std::cout << "[SideAhead] 已加载引导轨迹: " << loadedGuidePath
                  << " (" << pickedPathPoints.size() << " 点)" << std::endl;

        const Dobot::CDescartesPoint pointstarts =
            computeApproachStart(trajectory.front(), cfg.approachBackMm);
        runPickedPathWithTransition(demo, pickedPathPoints, pointstarts, cfg.guideExitPose, params);
    }
    else
    {
        std::cout << "[" << cfg.name << "] 前往轨迹起点 (退 " << cfg.approachBackMm << "mm)..." << std::endl;
        moveToApproachPoint(demo, trajectory.front(), cfg.approachBackMm);
    }

    std::cout << "到达首点，按 Enter 开始分段轨迹..." << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    std::vector<std::string> lines;
    if (!loadIndexLines(cfg.indexPath, lines))
        return false;

    const int totalLines = static_cast<int>(lines.size());
    int currentLine = 0;
    for (const auto &line : lines)
    {
        ++currentLine;
        const bool isLastLine = (currentLine == totalLines);

        std::istringstream iss(line);
        int idx;
        std::vector<Dobot::CDescartesPoint> selectedPoints;
        while (iss >> idx)
        {
            if (idx >= 0 && idx < static_cast<int>(trajectory.size()))
                selectedPoints.push_back(trajectory[idx]);
            else
                std::cerr << "索引 " << idx << " 超出范围\n";
        }

        if (selectedPoints.empty())
            continue;

        if (cfg.mode == BrushRunMode::CenterPointwise)
            runCenterSegment(demo, selectedPoints);
        else
            runMovsSegment(demo, cfg, selectedPoints, params, isLastLine);
    }

    executeRegionExit(demo, cfg, pointsafe);

    std::cout << "=== 区域完成: " << cfg.name << " ===\n" << std::endl;
    return true;
}

RegionBrushConfig makeRegionConfig(const char *name,
                                   const char *forcePath,
                                   const char *indexPath,
                                   BrushRunMode mode,
                                   const Dobot::CDescartesPoint &rotatetooljoint,
                                   double approachBackMm = 8.0)
{
    RegionBrushConfig cfg{};
    cfg.name = name;
    cfg.forcePath = forcePath;
    cfg.indexPath = indexPath;
    cfg.mode = mode;
    cfg.modifiedup = 0;
    cfg.modifiedupx = -6.9142260000002125 + 14.163013999999862;
    cfg.modifiedupy = -2.0350259999999025 - 24.190834000000166;
    cfg.modifiedupz = 101.3919;
    cfg.rotatetooljoint = rotatetooljoint;
    cfg.rotatetooljointjump = {0, 0, -50, 0, 0, 0};
    cfg.usePreJumpSequence = (mode != BrushRunMode::CenterPointwise);
    cfg.approachBackMm = approachBackMm;
    cfg.useSegmentBetween = false;
    cfg.segmentBetween = {-15, 0, 0, 0, 0, 0};
    cfg.useLastLeave = false;
    cfg.lastLeave = {0, 20, 0, 0, 0, 0};
    cfg.usePostUp = false;
    cfg.postUp = {0, 0, -60, 0, 0, 0};
    cfg.pickedPathFile = nullptr;
    cfg.pickedPathFallback = nullptr;
    cfg.useExitWaypoints = false;
    cfg.useEntryWaypoints = false;
    cfg.exitRotateAfterSafe = false;
    cfg.entryLiftMm = 0.0;
    cfg.guideExitPose = {58.8848, -393.7958, 528.6063, -179.6860, 43.6490, -160.6180};
    cfg.pointpartone = {606.9460, -64.8707, 426.0307, 175.3696, 43.8935, 109.5293};
    cfg.pointtwo = {482.4471, 32.6242, 402.5695, 179.9249, 1.0609, -145.5932};
    return cfg;
}

int main()
{
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    std::cout << "=== 运行全部力控轨迹（按各区域程序方式） ===" << std::endl;

    const std::string brushOffsetPath = "../defaultconfig/brushoffsets.json";
    const std::string configPath = "../defaultconfig/config.json";

    double brushSpeed = 80.0;
    try
    {
        std::ifstream file(configPath);
        if (file.is_open())
        {
            json j;
            file >> j;
            brushSpeed = j.value("brushSpeed", 80.0);
        }
    }
    catch (...)
    {
        std::cerr << "读取 config.json 失败，使用默认 brushSpeed=80\n";
    }

    DobotTcpDemo *demo = new DobotTcpDemo();
    const Dobot::CDescartesPoint pointsafe = makeSafePoint();

    Dobot::MovSParams params{};
    params.tool = 0;
    params.user = 0;
    params.v = 80;
    params.a = 80;
    params.speed = brushSpeed;
    params.freq = 0.2;

    RegionBrushConfig regions[] = {
        [] {
            // UpRight: 段间无 x-15，末段 y+20，结束 z-60
            auto cfg = makeRegionConfig("UpRight",
                                        "../defaultconfig/rightup/upright.txt",
                                        "../defaultconfig/rightup/all_segments.txt",
                                        BrushRunMode::UpLike,
                                        {0.001, 0, 0, 0, 0, 0});
            cfg.useLastLeave = true;
            cfg.usePostUp = true;
            return cfg;
        }(),
        [] {
            // UpLeft: 段间 x-15，末段 y+20，结束 z-60
            auto cfg = makeRegionConfig("UpLeft",
                                        "../defaultconfig/leftup/upleft.txt",
                                        "../defaultconfig/leftup/all_segments.txt",
                                        BrushRunMode::UpLike,
                                        {0.1, 0, 0, 0, 0, 0});
            cfg.useSegmentBetween = true;
            cfg.useLastLeave = true;
            cfg.usePostUp = true;
            return cfg;
        }(),
        [] {
            // SideRight: 退出 z-60 → pointsafe → rotatetooljoint
            auto cfg = makeRegionConfig("SideRight",
                                        "../defaultconfig/rightside/sideright.txt",
                                        "../defaultconfig/rightside/all_segments.txt",
                                        BrushRunMode::SideRightLike,
                                        {0.0, 0, 0, 0, -45, 0});
            cfg.useLastLeave = true;
            cfg.usePostUp = true;
            cfg.exitRotateAfterSafe = true;
            return cfg;
        }(),
        [] {
            // SideAhead: 引导→过渡位→38mm入点；退出 z-18 → 双途经点 → pointsafe
            auto cfg = makeRegionConfig("SideAhead",
                                        "../defaultconfig/sideahead/sideahead.txt",
                                        "../defaultconfig/sideahead/all_segments.txt",
                                        BrushRunMode::SideAheadLike,
                                        {0, 0, 0, 0, -48, 0},
                                        38.0);
            cfg.useSegmentBetween = true;
            cfg.useLastLeave = true;
            cfg.postUp = {0, 0, -18, 0, 0, 0};
            cfg.usePostUp = true;
            cfg.pickedPathFile = "../defaultconfig/rightside/ee_poses.txt";
            cfg.pickedPathFallback = "../defaultconfig/sideahead/ee_poses.txt";
            cfg.entryLiftMm = 38.0;
            cfg.guideExitPose = {58.8848, -393.7958, 528.6063, -179.6860, 43.6490, -160.6180};
            cfg.useEntryWaypoints = true;
            cfg.useExitWaypoints = true;
            return cfg;
        }(),
        [] {
            // SideLeft: 退出 z-60 → pointsafe → rotatetooljoint
            auto cfg = makeRegionConfig("SideLeft",
                                        "../defaultconfig/leftside/sideleft.txt",
                                        "../defaultconfig/leftside/all_segments.txt",
                                        BrushRunMode::SideNegativeXApproach,
                                        {0.1, 0, 0, 0, 45, 0});
            cfg.useSegmentBetween = true;
            cfg.useLastLeave = true;
            cfg.usePostUp = true;
            cfg.exitRotateAfterSafe = true;
            return cfg;
        }(),
        [] {
            auto cfg = makeRegionConfig("InSideRight",
                                        "../defaultconfig/rightinside/insideright.txt",
                                        "../defaultconfig/rightinside/all_segments.txt",
                                        BrushRunMode::SideNegativeXApproach,
                                        {0.0, 0, 0, -10, 45, 0});
            cfg.useSegmentBetween = true;
            cfg.useLastLeave = true;
            cfg.usePostUp = true;
            cfg.exitRotateAfterSafe = true;
            return cfg;
        }(),
        [] {
            auto cfg = makeRegionConfig("InSideLeft",
                                        "../defaultconfig/leftinside/insideleft.txt",
                                        "../defaultconfig/leftinside/all_segments.txt",
                                        BrushRunMode::InsideLeftApproach,
                                        {0.0, 0, 0, 0, -45, 0});
            cfg.useSegmentBetween = true;
            cfg.useLastLeave = true;
            cfg.usePostUp = true;
            cfg.exitRotateAfterSafe = true;
            return cfg;
        }(),
        [] {
            // Center: 逐点 move + y 摆动，不用 movsDemoC
            auto cfg = makeRegionConfig("Center",
                                        "../defaultconfig/center/center.txt",
                                        "../defaultconfig/center/all_segments.txt",
                                        BrushRunMode::CenterPointwise,
                                        {0.01, 0, 0, 0, 0, 0});
            cfg.usePreJumpSequence = false;
            return cfg;
        }(),
    };

    demo->moveRobotC(pointsafe, pointsafe);
    logCurrentPose(demo, "main 起始 pointsafe");
    std::cout << "已回安全点，按 Enter 开始全部区域..." << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    int successCount = 0;
    int failCount = 0;
    for (const auto &region : regions)
    {
        if (executeRegionTrajectory(demo, region, pointsafe, params, brushOffsetPath))
            ++successCount;
        else
            ++failCount;
        Sleep(500);
    }

    demo->moveRobotC(pointsafe, pointsafe);
    std::cout << "\n=== 全部完成 ===" << std::endl;
    std::cout << "成功: " << successCount << "  失败: " << failCount << std::endl;

    delete demo;
    demo = nullptr;
    return failCount > 0 ? -1 : 0;
}
