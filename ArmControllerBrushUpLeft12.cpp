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
void fineTuneXYZ(DobotTcpDemo* demo, Dobot::CDescartesPoint& curPose,
    Eigen::Vector3d& totalOffset)
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

            std::cout << "key" <<key <<std::endl;

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
            else if(key == 'z'){
                std::cout <<  " hello "<< brushDirs.x() <<std::endl;
                dx += brushDirs.x();
                dy += brushDirs.y();
                dz += brushDirs.z();
            }
            else if(key == 'x'){
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

Eigen::Vector3d getManualOffset(DobotTcpDemo *demo, double refX, double refY, double refZ)
{
    double gx, gy, gz, grx, gry, grz;
    while (!demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    Dobot::CDescartesPoint curPose{264.8929, -285.1852, 396.0669, -179.7725, -1.3507, -145.9055};
    Eigen::Vector3d totalOffset(0, 0, 0);

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
            std::cout << "\r当前牙刷累计轨迹偏移 [mm]: " << std::endl;
            demo->moveRobotC(curPose, curPose);

            // 实时打印当前相对基准点的偏移
            Eigen::Vector3d currentRelativeOffset(curPose.x - refX, curPose.y - refY, curPose.z - refZ);
            std::cout << "\r当前牙刷累计轨迹偏移 [mm]: " << currentRelativeOffset.transpose() << "    " << std::flush;
        }
        Sleep(10);
    }

    // 返回最终计算出的偏移量
    return Eigen::Vector3d(curPose.x - refX, curPose.y - refY, curPose.z - refZ);
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
        teethModelPath   = j.at("teethModelPath").get<std::string>();
        toothbrushPath   = j.at("toothbrushPath").get<std::string>();
        brushIterations  = j.at("brushIterations").get<int>();
        brushSpeed       = j.at("brushSpeed").get<double>();
        backAndForthCount= j.at("backAndForthCount").get<int>();
        pressureParameter= j.at("pressureParameter").get<double>();
        brushDuration    = j.at("brushDuration").get<int>();

        // 打印测试
        std::cout << "teethModelPath: " << teethModelPath << std::endl;
        std::cout << "toothbrushPath: " << toothbrushPath << std::endl;
        std::cout << "brushIterations: " << brushIterations << std::endl;
        std::cout << "brushSpeed: " << brushSpeed << std::endl;
        std::cout << "backAndForthCount: " << backAndForthCount << std::endl;
        std::cout << "pressureParameter: " << pressureParameter << std::endl;
        std::cout << "brushDuration: " << brushDuration << std::endl;
    }
    
    catch (std::exception& e)
    {
        std::cerr << "JSON 解析错误: " << e.what() << std::endl;
        return -1;
    }

    SetConsoleOutputCP(CP_UTF8);
    CameraCapture* camera = new CameraCapture("169.254.7.168");
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
    DobotTcpDemo* demo = new DobotTcpDemo();

    struct PointData
    {
        double x, y, z, a, b, c;
    };

    struct BrushVector
    {
        double x, y, z;
    };

    const std::string Force_FILE_PATH = "D:\\UsmileProject\\hand_eye_calibration\\upleft.txt";

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

    
    // 微調牙刷
    const double REF_X = 264.8929;
    const double REF_Y = -285.1852;
    const double REF_Z = 391.0669;

    // 詢問用戶是否進行微調
    std::cout << "是否進行牙刷微調？(y/n): ";
    char userInput;
    std::cin >> userInput;

    if (userInput == 'y' || userInput == 'Y')
    {
        // 1. 进入键盘微调模式，手动对准后获取 Offset
        Eigen::Vector3d totalOffset = getManualOffset(demo, REF_X, REF_Y, REF_Z);
        json offsetJson;
        offsetJson["brushxoffset"] = totalOffset.x();
        offsetJson["brushyoffset"] = totalOffset.y();
        offsetJson["brushzoffset"] = totalOffset.z();

        std::ofstream file("D:\\UsmileProject\\hand_eye_calibration\\brushoffsets.json");
        file << offsetJson.dump(4); // 使用4空格缩进
        file.close();

        std::cout << "牙刷微調完成，偏移量已保存。" << std::endl;
    }
    else
    {
        std::ifstream inputFile("D:\\UsmileProject\\hand_eye_calibration\\brushoffsets.json");
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
            pointa.x = 264.8929 + offsetX;
            pointa.y = -285.1852 + offsetY;
            pointa.z = 391.0669 + offsetZ;
            pointa.rx = -179.7725;
            pointa.ry = -1.3507;
            pointa.rz = -145.9055;
            demo->moveRobotC(pointa, pointa);
            std::cout << "机械臂到达起始点" << std::endl;
            std::cout << "跳過牙刷微調。" << std::endl;
        }
    }

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
    rotatetooljoint.ry = 0;
    rotatetooljoint.rz = 0;

    std::wcout << L"机械臂到达起始点，请确认按Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    std::cout << "请选择操作：" << std::endl;
    std::cout << "1. 使用已导入的轨迹" << std::endl;
    std::cout << "2. 生成新轨迹" << std::endl;
    std::cout << "请输入选择 (1 或 2): ";
    
    int choice;
    std::cin >> choice;

    std::string indexFilePath = "D:\\UsmileProject\\hand_eye_calibration\\all_segments.txt";
    if (choice == 2) {
        int python_result2 =
            std::system("cmd /c "
                        "\"D:\\UsmileProject\\hand_eye_calibration\\."
                        "venv312\\Scripts\\activate && python "
                        "D:\\UsmileProject\\hand_eye_calibration\\GenerateAnyPath.py 1\"");
        
        if (python_result2 == 0) {
            std::cout << "新轨迹生成成功！" << std::endl;
        } else {
            std::cout << "新轨迹生成失败！" << std::endl;
        }
    }
    else if (choice == 1) {
        std::cout << "正在使用已导入的轨迹..." << std::endl;
        int python_result1 =
            std::system("cmd /c "
                        "\"D:\\UsmileProject\\hand_eye_calibration\\."
                        "venv312\\Scripts\\activate && python "
                        "D:\\UsmileProject\\hand_eye_calibration\\GenerateAnyPathRepeat.py 1\"");
        
        if (python_result1 == 0) {
            std::cout << "已导入轨迹使用成功！" << std::endl;
        } else {
            std::cout << "已导入轨迹使用失败！" << std::endl;
        }
        indexFilePath = "D:\\UsmileProject\\hand_eye_calibration\\UsmileUi\\defaultconfig\\leftup\\all_segments.txt";
    }
    else {
        std::cout << "无效的选择！" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // @@@@@@@@@@@@@@@@@@@@@@@@@@调整牙刷起始姿态@@@@@@@@@@@@@@@@@@
    demo->RelMovJDemo(rotatetooljoint, 0, 3, 20, 50, 100);
    std::cout << "牙刷初始姿态已经调整好，请确认Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
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
                std::ofstream poseFile("D:\\UsmileProject\\hand_eye_calibration\\current_pose_from_getpose.txt");
                if (poseFile.is_open()) {
                    poseFile << gx << " " << gy << " " << gz << " "
                        << grx << " " << gry << " " << grz << std::endl;
                    poseFile.close();
                    std::cout << "当前刷头位置已经保存下去\n";
                }
                else {
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
    int python_result11 = std::system(
        "cmd /c "
        "\"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate "
        "&& python "
        "D:\\UsmileProject\\hand_eye_calibration\\TCPRotation10OthersDorobot.py\"");

    // ================= 读取轨迹 =================
    std::vector<PointData> brushpointsoffset_ee_poses;
    std::ifstream ee_poses_infile("D:\\UsmileProject\\hand_eye_calibration\\saved_ply\\ee_poses.txt");
    if (!ee_poses_infile.is_open())
    {
        std::cerr << "无法打开 ee_poses.txt" << std::endl;
        return -1;
    }
    double dx, dy, dz, rx, ry, rz;
    while (ee_poses_infile >> dx >> dy >> dz >> rx >> ry >> rz)
    {
        brushpointsoffset_ee_poses.push_back({ dx, dy, dz, rx, ry, rz });
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
        demo->moveRobotC(firstPose, firstPose);

        std::cout << "机械臂到达起始点，请确认按Enter" << std::endl;
        std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

        Eigen::Vector3d deltaOffset(0, 0, 0);
        fineTuneXYZ(demo, firstPose, deltaOffset);

        for (auto& p : brushpointsoffset_ee_poses)
        {
            p.x += deltaOffset.x();
            p.y += deltaOffset.y();
            p.z += deltaOffset.z();
        }

        std::vector<Dobot::CDescartesPoint> descartesPoints;
        for (const auto& p : brushpointsoffset_ee_poses)
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
    std::vector<Dobot::CDescartesPoint>  descartesPointsforce;

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
    for (const auto& p : brushpointsoffset_ee_poses)
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

    for (size_t i = 0; i < descartesPoints.size(); ++i)
    {
        auto& offset = descartesPoints[i];
        bool converged = false;
        int firstcount = 0;
        while (!converged)
        {
            float force[6];
           while (obj->GetCurrentForceData(force) != 28 && force[2] != 0)
            {
                std::cerr << "获取力控数据失败\n";
            }
            force[0] -= forcefirst[0];
            force[1] -= forcefirst[1];
            force[2] -= forcefirst[2];


            demo->moveRobotC(offset, offset);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            Eigen::Matrix3d rotationMatrix = eulerDegToRotationMatrix(offset.rx, offset.ry, offset.rz);
            Eigen::Vector3d brushDir = rotationMatrix.col(2);
            brushDir.normalize();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            Eigen::Vector3d measured(force[0], force[1], force[2]);
            double proj = measured.dot(brushDir);
            double err = proj - targetforcevalue;
            if(targetforcevalue == 0){
                err = 0.0;
            }

            std::cout << "proj = " << proj << "  err = " << err << std::endl;

            Eigen::Vector3d targetDir(0, 0, -1);
            double dotProduct = brushDir.dot(targetDir);
            double brushDirNorm = brushDir.norm();
            double targetDirNorm = targetDir.norm();
            double angleRad = std::acos(dotProduct / (brushDirNorm * targetDirNorm));
            double angleDeg = angleRad * 180.0 / M_PI;

            if (angleDeg > 6.0)
            {
                if (std::abs(err) <= 0.04)
                {
                    converged = true;
                    break;
                }
                else if (err > 0.04)
                {
                    Eigen::Vector3d delta = -0.6 * brushDir;
                    offset.x += delta.x();
                    offset.y += delta.y();
                    offset.z += delta.z();
                }
                else if (err < -0.04)
                {
                    Eigen::Vector3d delta = 0.6 * brushDir;
                    offset.x += delta.x();
                    offset.y += delta.y();
                    offset.z += delta.z();
                }
            }
            else
            {
                if (std::abs(err) <= 0.04)
                {
                    converged = true;
                    break;
                }
                else if (err > 0.04)
                {
                    Eigen::Vector3d delta = -0.6 * brushDir;
                    offset.x += delta.x();
                    offset.y += delta.y();
                    offset.z += delta.z();
                }
                else if (err < -0.04)
                {
                    Eigen::Vector3d delta = 0.6 * brushDir;
                    offset.x += delta.x();
                    offset.y += delta.y();
                    offset.z += delta.z();
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
    std::cout << "调整后的力控轨迹保存完毕" << std::endl;

    demo->moveRobotC(pointsafe, pointsafe);

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

   std::cout << "运行完整力控轨迹（每行一次movsDemoC）" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    Dobot::MovSParams params1;
    params1.tool = 0;
    params1.user = 0;
    params1.v = 80;
    params1.a = 80;
    params1.freq = 0.8;

    // std::string indexFilePath = "D:\\UsmileProject\\hand_eye_calibration\\all_segments.txt";
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
            // std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待1秒
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            demo->movsDemoC(selectedPoints, params1);
            // std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待力控轨迹完成
            std::this_thread::sleep_for(std::chrono::milliseconds(180*selectedPoints.size()));

            // std::this_thread::sleep_for(std::chrono::milliseconds(8));

            // double gx, gy, gz, grx, gry, grz;
            // double targetx, targety, targetz;
            // targetx = selectedPoints.back().x;
            // targety = selectedPoints.back().y;
            // targetz = selectedPoints.back().z;

            // while (true)
            // {
            //     if (demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
            //     {
            //         double diffx = fabs(gx - targetx);
            //         double diffy = fabs(gy - targety);
            //         double diffz = fabs(gz - targetz);

            //         if (diffx < 0.01 && diffy < 0.01 && diffz < 0.01)
            //         {
            //             printf("條件滿足，跳出循環！\n");
            //             break;
            //         }
            //         std::this_thread::sleep_for(std::chrono::milliseconds(8));

            //     }

                
            // }
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // std::cout << "执行第 " << lineNumber << " 行轨迹，共 " << selectedPoints.size() << " 个点\n";
        }
    }

    indexFile.close();

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
