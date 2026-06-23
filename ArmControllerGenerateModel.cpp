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
    CameraCapture *camera = new CameraCapture("169.254.7.168");

    SetConsoleCP(CP_UTF8);
    DobotTcpDemo *demo = new DobotTcpDemo();

    struct PointData
    {
        double x, y, z, a, b, c;
    };


    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@回到初始态@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
   
    Dobot::CDescartesPoint pointa{};
    pointa.x = 264.8929 + -6.9142260000002125 +14.163013999999862;
    pointa.y = -285.1852 -2.0350259999999025 -24.190834000000166;
    // pointa.z = 391.0669 + 48.16690799999924 +7.156301999999357;
    pointa.z = 492.4588;
    pointa.rx = -179.7725;
    pointa.ry = -1.3507;
    pointa.rz = -145.9055;
    demo->moveRobotC(pointa, pointa);

    std::wcout << L"机械臂到达起始点，请确认按Enter" << std::endl;
    std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

    camera->captureAndSave(10);

    // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@生成刷牙的轨迹@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    std::cout << "Running Python script to generate brush other..." << std::endl;
    int python_result1 =
        std::system("cmd /c "
                    "\"D:\\UsmileProject\\hand_eye_calibration\\."
                    "venv312\\Scripts\\activate && python "
                    "D:\\UsmileProject\\hand_eye_"
                    "calibration\\GeneratePathOffset66initAllnewdorobots.py\"");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "正在退出程序請稍後：-）" << std::endl;

    camera->~CameraCapture();
    demo->~DobotTcpDemo();

    delete camera;
    delete demo;

    camera = nullptr;
    demo = nullptr;

    return 0;
}
