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
#define MODE 0
bool capturing = true;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
constexpr double RAD2DEG = 180.0 / M_PI;

/* ======================= 数据结构 ======================= */
struct PointData
{
    double x, y, z, a, b, c;
};

int main()
{
    SetConsoleOutputCP(CP_UTF8);
    // CameraCapture *camera = new CameraCapture("169.254.7.168");

    SetConsoleCP(CP_UTF8);
    DobotTcpDemo *demo = new DobotTcpDemo();

    struct PointData
    {
        double x, y, z, a, b, c;
    };

    double dx = -0.560633;
                double dy = 0.828055;
                double dz = -0.003970;
    int repeat = 10;

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@回到初始态@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    Dobot::CDescartesPoint pointa{};
    pointa.x = 264.8929;
    pointa.y = -285.1852;
    pointa.z = 496.0669;
    pointa.rx = -179.7725;
    pointa.ry = -1.3507;
    pointa.rz = -145.9055;
    demo->moveRobotC(pointa, pointa);

    Dobot::CDescartesPoint pointa1{};
    pointa1.x = 282.4216-dx*2;
    pointa1.y = -337.7954-dy*2;
    pointa1.z = 373.6094 + 10-dz*2;
    pointa1.rx = -179.7725;
    pointa1.ry = -1.3507;
    pointa1.rz = -145.9055;

    Dobot::CDescartesPoint pointa2{};
    pointa2.x = pointa1.x +dx *repeat;
    pointa2.y = pointa1.y + dy* repeat;
    pointa2.z = pointa1.z + dz *repeat;
    pointa2.rx = -179.7725;
    pointa2.ry = -1.3507;
    pointa2.rz = -145.9055;

    Dobot::CDescartesPoint pointa3{};
    pointa3.x = pointa1.x +dx *repeat*2;
    pointa3.y = pointa1.y +dy *repeat*2;
    pointa3.z = pointa1.z +dz *repeat*2;
    pointa3.rx = -179.7725;
    pointa3.ry = -1.3507;
    pointa3.rz = -145.9055;

    Dobot::CDescartesPoint pointa4{};
    pointa4.x = pointa1.x +dx *repeat*3;
    pointa4.y = pointa1.y +dy *repeat*3;
    pointa4.z = pointa1.z +dz *repeat*3;
    pointa4.rx = -179.7725;
    pointa4.ry = -1.3507;
    pointa4.rz = -145.9055;

    Dobot::CDescartesPoint pointa5{};
    pointa5.x = pointa1.x +dx *repeat*4;
    pointa5.y = pointa1.y +dy *repeat*4;
    pointa5.z = pointa1.z +dz *repeat*4;
    pointa5.rx = -179.7725;
    pointa5.ry = -1.3507;
    pointa5.rz = -145.9055;

    Dobot::MovSParams params;
    params.tool = 0;
    params.user = 0;
    params.v = 80;
    params.a = 80;
    params.speed = 80;
    params.freq = 0.2;

    std::vector<Dobot::CDescartesPoint> selectedPoints;

    selectedPoints.push_back(pointa1);
    selectedPoints.push_back(pointa2);
    selectedPoints.push_back(pointa3);
    selectedPoints.push_back(pointa4);
    selectedPoints.push_back(pointa5);

    while (1)
    {
        demo->moveRobotC(pointa1, pointa1);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        demo->movsDemoC(selectedPoints, params);

        demo->moveRobotC(pointa, pointa);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    //     std::wcout << L"机械臂到达安全點，请确认按Enter" << std::endl;
    // std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
    }

    // camera->captureAndSave(10);

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@生成刷牙的轨迹@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    // std::cout << "Running Python script to generate brush other..." << std::endl;
    // int python_result1 =
    //     std::system("cmd /c "
    //                 "\"D:\\UsmileProject\\hand_eye_calibration\\."
    //                 "venv312\\Scripts\\activate && python "
    //                 "D:\\UsmileProject\\hand_eye_"
    //                 "calibration\\GeneratePathOffset66initAllnewdorobots.py\"");
    // std::this_thread::sleep_for(std::chrono::seconds(2));

    // std::cout << "正在退出程序請稍後：-）" << std::endl;

    // camera->~CameraCapture();
    demo->~DobotTcpDemo();

    // delete camera;
    delete demo;

    // camera = nullptr;
    demo = nullptr;

    return 0;
}
