#include <cmath>
#include <conio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "DobotTcpDemo.h"
#include <windows.h>

// ================= 4. 主程序 =================
// int main()
// {
//     SetConsoleOutputCP(CP_UTF8);
//     SetConsoleCP(CP_UTF8);
//     DobotTcpDemo *demo = new DobotTcpDemo();
    
//     // 先运动到安全点
//     Dobot::CDescartesPoint pointsafe{};
//     pointsafe.x = 264.8929;
//     pointsafe.y = -285.1852;
//     pointsafe.z = 491.0669;
//     pointsafe.rx = -179.7725;
//     pointsafe.ry = -1.3507;
//     pointsafe.rz = -145.9055;
//     demo->moveRobotC(pointsafe, pointsafe);
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     int python_result1 =
//             std::system("cmd /c "
//                         "\"D:\\UsmileProject\\hand_eye_calibration\\."
//                         "venv312\\Scripts\\activate && python "
//                         "D:\\MVS\\MVS\\Development\\Samples\\OpenCV\\Python\\GrabImage_Cv\\GrabSingleImage_OneShot_Cv.py 1.bmp\"");


//     delete demo;
//     return 0;
// }


// int main()
// {
//     SetConsoleOutputCP(CP_UTF8);
//     SetConsoleCP(CP_UTF8);
//     DobotTcpDemo *demo = new DobotTcpDemo();
    
//     // 先运动到安全点
//     Dobot::CDescartesPoint pointsafe{};
//     pointsafe.x = 264.8929;
//     pointsafe.y = -285.1852;
//     pointsafe.z = 491.0669;
//     pointsafe.rx = -179.7725;
//     pointsafe.ry = -1.3507;
//     pointsafe.rz = -145.9055;
//     demo->moveRobotC(pointsafe, pointsafe);
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // 定义所有需要移动到的点位
//     std::vector<Dobot::CDescartesPoint> targetPoints = {
//         // 第一个点
//         {214.2748, 275.5199, 378.6787, 178.9953, 0.4812, 150.5753},
//         // 第二个点
//         {151.9888, 327.7318, 365.0857, 175.5739, 22.8960, -150.8589},
//         // 第三个点
//         {329.6199, -199.9806, 356.1461, 171.6548, 52.3667, 133.4233},
//         // 第四个点
//         {247.7584, -273.0000, 420.2380, 149.1564, 47.2733, -86.0594},
//         // 第五个点
//         {246.0000, 217.6866, 422.0986, 179.9166, 0.0645, 144.7085}
//     };

//     // 循环移动到每个点位并拍照
//     for (size_t i = 0; i < targetPoints.size(); i++)
//     {
//         // 移动到目标点
//         demo->moveRobotC(targetPoints[i], targetPoints[i]);
//         std::this_thread::sleep_for(std::chrono::seconds(1));

//         // 生成照片文件名：1.bmp, 2.bmp, 3.bmp, 4.bmp, 5.bmp
//         std::string photoName = std::to_string(i + 1) + ".bmp";
        
//         // 构建Python命令
//         std::string python_command = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\MVS\\MVS\\Development\\Samples\\OpenCV\\Python\\GrabImage_Cv\\GrabSingleImage_OneShot_Cv.py " + photoName + "\"";
        
//         // 执行拍照
//         int python_result = std::system(python_command.c_str());
        
//         // 可选：检查拍照结果
//         if (python_result != 0)
//         {
//             std::cerr << "拍照失败: " << photoName << std::endl;
//         }
//         else
//         {
//             std::cout << "成功拍摄: " << photoName << std::endl;
//         }

//         // 拍照后等待一下，避免连续操作太快
//         std::this_thread::sleep_for(std::chrono::milliseconds(500));
//     }

//     // 所有点位拍摄完成后，可以选择回到安全点
//     std::cout << "所有点位拍摄完成，共拍摄 " << targetPoints.size() << " 张照片" << std::endl;
//     demo->moveRobotC(pointsafe, pointsafe);
//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     delete demo;
//     return 0;
// }

int main()
{
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    DobotTcpDemo *demo = new DobotTcpDemo();
    
    // 先运动到安全点
    Dobot::CDescartesPoint pointsafe{};
    pointsafe.x = 264.8929;
    pointsafe.y = -285.1852;
    pointsafe.z = 491.0669;
    pointsafe.rx = -179.7725;
    pointsafe.ry = -1.3507;
    pointsafe.rz = -145.9055;
    
    std::cout << "准备移动到安全点:" << std::endl;
    std::cout << "x: " << pointsafe.x << ", y: " << pointsafe.y << ", z: " << pointsafe.z 
              << ", rx: " << pointsafe.rx << ", ry: " << pointsafe.ry << ", rz: " << pointsafe.rz << std::endl;
    std::cout << "按回车键开始移动到安全点..." << std::endl;
    std::cin.get();
    
    demo->moveRobotC(pointsafe, pointsafe);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 定义所有需要移动到的点位
    std::vector<Dobot::CDescartesPoint> targetPoints = {
        // 第一个点
        {214.2748, -275.5199, 378.6787, -178.9953, 0.4812, -150.5753},
        // 第二个点
        {151.9888, -327.7318, 365.0857, 175.5739, 22.8960, -150.8589},

        {151.9888, -327.7318, 365.0857, -175.5739, 0.8960, -150.8589},
        // 第三个点
        // 第三个点
        {329.6199, -199.9806, 356.1461, 171.6548, -52.3667, -133.4233},
        // 第四个点
        {247.7584, -273.0000, 420.2380, 149.1564, -47.2733, -86.0594},
        // 第五个点
        {246.0000, -217.6866, 422.0986, 179.9166, -0.0645, -144.7085}
    };

    // 循环移动到每个点位并拍照
    for (size_t i = 0; i < targetPoints.size(); i++)
    {
        std::cout << "\n========== 第 " << (i + 1) << " 个点位 ==========" << std::endl;
        std::cout << "准备移动到点位:" << std::endl;
        std::cout << "x: " << targetPoints[i].x << std::endl;
        std::cout << "y: " << targetPoints[i].y << std::endl;
        std::cout << "z: " << targetPoints[i].z << std::endl;
        std::cout << "rx: " << targetPoints[i].rx << std::endl;
        std::cout << "ry: " << targetPoints[i].ry << std::endl;
        std::cout << "rz: " << targetPoints[i].rz << std::endl;
        
        std::cout << "按回车键开始移动到该点位..." << std::endl;
        std::cin.get(); // 等待用户按回车

        // 移动到目标点
        demo->moveRobotC(targetPoints[i], targetPoints[i]);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 生成照片文件名：1.bmp, 2.bmp, 3.bmp, 4.bmp, 5.bmp
        std::string photoName = std::to_string(i + 1) + ".bmp";
        
        std::cout << "准备拍照: " << photoName << std::endl;
        std::cout << "按回车键开始拍照..." << std::endl;
        std::cin.get(); // 等待用户按回车拍照
        
        // 构建Python命令
        std::string python_command = "cmd /c \"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate && python D:\\MVS\\MVS\\Development\\Samples\\OpenCV\\Python\\GrabImage_Cv\\GrabSingleImage_OneShot_Cv.py " + photoName + "\"";
        
        // 执行拍照
        int python_result = std::system(python_command.c_str());
        
        // 可选：检查拍照结果
        if (python_result != 0)
        {
            std::cerr << "拍照失败: " << photoName << std::endl;
        }
        else
        {
            std::cout << "成功拍摄: " << photoName << std::endl;
        }

        // 拍照后等待一下，避免连续操作太快
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "第 " << (i + 1) << " 个点位处理完成" << std::endl;
    }

    // 所有点位拍摄完成后，可以选择回到安全点
    std::cout << "\n所有点位拍摄完成，共拍摄 " << targetPoints.size() << " 张照片" << std::endl;
    std::cout << "准备返回安全点" << std::endl;
    std::cout << "按回车键返回安全点..." << std::endl;
    std::cin.get();
    
    demo->moveRobotC(pointsafe, pointsafe);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "程序执行完成，按回车键退出..." << std::endl;
    std::cin.get();

    delete demo;
    return 0;
}