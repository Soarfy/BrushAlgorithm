#include "VideoCapture.hpp"
#include <cmath>
#include <conio.h>
#include <crtdbg.h>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
//#include <iostream>      // 用于 std::cin
#include <limits>        // 用于 std::numeric_limits

// 新机械臂的头文件
#include "DobotTcpDemo.h"
#include <windows.h>


// 直接使用六维力原始代码
#include "kw-lib-all.h"
#define MODE 0
bool capturing = true;
NS_KW_USING

// 弧度定义
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// 将弧度转为角度
constexpr double RAD2DEG = 180.0 / M_PI;

int main() {
  std::cout << "Start BrushSideLeft" << std::endl;
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@机械臂初始化@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@相机初始化@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  CameraCapture camera("169.254.7.168");

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@6维力初始化@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
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
 // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@初始化6维力对象@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 auto control = uc.createIOControler();
 HeadTailProtocolCreator htc;
 auto proto = htc.createProtocol();
 SensorControlCreator scc;
 scc.ioCtrl = control;
 scc.proto = proto;
 auto obj = scc.createSensorControl();
 int hr = obj->StartCapture();
 if (hr != 0) {
   printf("start capture faield\n");
   return hr;
 }

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@检测机械臂链接状态@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  SetConsoleOutputCP(CP_UTF8); //  UTF-8
  SetConsoleCP(CP_UTF8);       // UTF-8ѡ
  DobotTcpDemo *demo = new DobotTcpDemo();

  struct PointData {
    double x, y, z, a, b, c;
  };

  struct BrushVector {
    double x, y, z;
  };

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@回到初始态@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  Dobot::CDescartesPoint pointa{};
  pointa.x = 264.8929;
  pointa.y = -285.1852;
  pointa.z = 391.0669;
  pointa.rx = -179.7725;
  pointa.ry = -1.3507;
  pointa.rz = -145.9055;

  Dobot::CDescartesPoint pointb{};
  pointb.x = 300;
  pointb.y = 0;
  pointb.z = 100;
  pointb.rx = 0;
  pointb.ry = 0;
  pointb.rz = 0;

  demo->moveRobotC(pointa, pointb);

  std::cout << "When robot move finished, please press Enter" << std::endl;
  std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n'); // 等待用户按下Enter



  camera.captureAndSave(10);

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@生成刷牙的轨迹@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 std::cout << "Running Python script to generate brush other..." << std::endl;
  int python_result1 =
       std::system("cmd /c "
                   "\"D:\\UsmileProject\\hand_eye_calibration\\."
                   "venv312\\Scripts\\activate && python "
                   "D:\\UsmileProject\\hand_eye_"
                   "calibration\\GeneratePathOffset66initAllnewdorobot.py\"");
   std::this_thread::sleep_for(std::chrono::seconds(2));

 std::cout << "Running Python script to generate brush other 6..." << std::endl;
  int python_result2 =
     std::system("cmd /c "
                 "\"D:\\UsmileProject\\hand_eye_calibration\\."
                 "venv312\\Scripts\\activate && python "
                 "D:\\UsmileProject\\hand_eye_"
                 "calibration\\GeneratePathOffset66BrushOther6dorobot.py\"");
  std::this_thread::sleep_for(std::chrono::seconds(2));


  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@生成的轨迹转移到机械臂末端@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 int python_result11 = std::system(
     "cmd /c "
     "\"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate "
     "&& python "
     "D:\\UsmileProject\\hand_eye_calibration\\TCPRotation10OthersDorobot.py\"");

//   先自己写一个txt文件用来走轨迹
  std::vector<PointData> brushpointsoffset_ee_poses;
  std::ifstream ee_poses_infile(
      "D:\\UsmileProject\\hand_eye_calibration\\saved_ply\\ee_poses.txt");
    //  "D:\\UsmileProject\\hand_eye_calibration\\saved_ply\\transforms.txt");
  if (!ee_poses_infile.is_open()) {
    std::cerr << "Failed to open ee_poses.txt" << std::endl;
    return -1;
  }

  double dx, dy, dz, rx, ry, rz;
  while (ee_poses_infile >> dx >> dy >> dz >> rx >> ry >> rz) {
    std::cout << "Read values: " << dx << " >> " << dy << " >> " << dz << ">> "
              << rx << ">> " << ry << ">> " << rz << std::endl;
    brushpointsoffset_ee_poses.push_back({dx, dy, dz, rx, ry, rz});
  }
  ee_poses_infile.close();


  

  // @@@@@@@@@@@@@@@@@@@@@@@@@@跑一遍没有修复的轨迹，边走边拍图用于后续的计算@@@@@@@@@@@@@@@@@@
   Dobot::CDescartesPoint rotatetooljoint{};
   rotatetooljoint.x = 0;
   rotatetooljoint.y = 0;
   rotatetooljoint.z = 0;
   rotatetooljoint.rx = 0;
   rotatetooljoint.ry = -30;
   rotatetooljoint.rz = 0;

  demo->RelMovJDemo(rotatetooljoint,0,3,20,50,100);

  std::cout << "When robot Rotate finished, please press Enter" << std::endl;
  std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n'); // 等待用户按下Enter

  int countvalue = 1;
   if (!brushpointsoffset_ee_poses.empty()) {
    Dobot::MovSParams params;
    params.tool = 0;   // 工具坐标系索引，默认值：0
    params.user = 0;   // 用户坐标系索引，默认值：0
    params.v = 80;     // 运动速度比例 (0,100]，默认值：100
    params.a = 80;     // 运动加速度比例 (0,100]，默认值：100
    params.freq = 0.7; // 滤波系数 (0,1]，默认值：0.2，CAD轨迹可设为1保证精度
    std::vector<Dobot::CDescartesPoint> descartesPoints;
    for (const auto &offset : brushpointsoffset_ee_poses) {
      Dobot::CDescartesPoint pc1{};
      pc1.x = offset.x;
      pc1.y = offset.y;
      pc1.z = offset.z;
      pc1.rx = offset.a;
      pc1.ry = offset.b;
      pc1.rz = offset.c;
      descartesPoints.push_back(pc1);

      if (countvalue == 1) {
          demo->moveRobotC(pc1, pc1);
          // camera.captureAndSave(countvalue);
          countvalue += 1;
          std::cout << "When robot Rotate finished, please press Enter" << std::endl;
          std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n'); // 等待用户按下Enter
      }
       
      
    }
    // 直接走整个轨迹
    demo->movsDemoC(descartesPoints, params);
  }

//   // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@添加力控@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//   // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@加载每个轨迹点刷头的方向@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//     std::vector<BrushVector> brushvectors;
//     std::ifstream brushvectorinfile("D:\\UsmileProject\\hand_eye_calibration\\brushing_vectors.txt");
//     if (!brushvectorinfile.is_open())
//     {
//       std::cerr << "Failed to open brushing_vectors.txt" << std::endl;
//       return -1;
//     }
//     double dxbrush, dybrush, dzbrush;
//     while (brushvectorinfile >> dxbrush >> dybrush >> dzbrush)
//     {
//       brushvectors.push_back({dxbrush, dybrush, dzbrush});
//     }
//     brushvectorinfile.close();


//   // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@根据期望的力道来调整轨迹@@@@@@@@@@@@@@@@@@@@@@@@@@@
//     double targetforcevalue = 0.2;
//     std::ofstream forcerepairedoutputfile("D:\\UsmileProject\\hand_eye_calibration\\transformsrepairedforcesideright.txt");

//     if (!forcerepairedoutputfile.is_open())
//     {
//         std::cerr << "Failed to open transformsrepairedforce.txt" << std::endl;
//         return -1;
//     }

//     // @@@@@@@@@@@@@@@@@@@对6维力清零@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//     float forcefirst[6];
//    /* while (obj->GetCurrentForceData(forcefirst) != 28 && forcefirst[2] != 0 && forcefirst[0] < 1.0)
//     {
//         std::cerr << "Get first data error\n";
//     }*/

//     int sampleCount = 0;
//     const int maxSamples = 30;
//     bool success = false;

//     // 采样最多10次，取最后一次满足条件的采样
//     while (sampleCount < maxSamples)
//     {
//         // 获取力数据
//         int result = obj->GetCurrentForceData(forcefirst);

//         // 检查条件是否满足
//         if (result == 28 && forcefirst[2] > -3.0 && forcefirst[0] < 1.0 && forcefirst[2] < -1.0)
//         {
//             success = true;
//             break;  // 条件满足，跳出循环
//         }

//         sampleCount++;

//         // 如果条件不满足且未达到最大采样次数，继续采样
//         if (sampleCount < maxSamples)
//         {
//             std::cerr << "采样 " << sampleCount << " 次失败，继续采样...\n";
//         }
//         else
//         {
//             std::cerr << "采样 " << sampleCount << " 次均失败\n";
//         }
//     }

//     // 检查最终结果
//     if (!success)
//     {
//         std::cerr << "Get first data error: 30次采样均未满足条件\n";
//     }

//     printf("forcefirst0: %.2f forcefirst1: %.2f forcefirst2: %.2f \n",
//         forcefirst[0], forcefirst[1], forcefirst[2]);

//     for (size_t i = 0; i < brushpointsoffset_ee_poses.size(); ++i)
//     { 
//         auto &offset = brushpointsoffset_ee_poses[i];
//         const auto &dir = brushvectors[i];
//         bool converged = false;

//         // #########################单位化牙刷方向向量#######################
//         Eigen::Vector3d brushDir(dir.x, dir.y, dir.z);
//         brushDir.normalize();

//         Dobot::CDescartesPoint pc1{};
//         pc1.x = offset.x;
//         pc1.y = offset.y;
//         pc1.z = offset.z;
//         pc1.rx = offset.a;
//         pc1.ry = offset.b;
//         pc1.rz = offset.c;

//         int firstcount = 0;
//         while (!converged)
//         {
        
//             float force[6];
//            while (obj->GetCurrentForceData(force) != 28 && force[2] != 0)
//             {
//                 std::cerr << "failed to get force data\n";
//             }

//             force[0] -= forcefirst[0];
//             force[1] -= forcefirst[1];
//             force[2] -= forcefirst[2];

//             std::cout << "start: " << std::endl;
//             // 挪动一下没有调整的位置
//             demo->moveRobotC(pc1, pc1);

//             Eigen::Vector3d measured(force[0], force[1], force[2]);

//             printf("force0: %.2f force1: %.2f force2: %.2f \n",
//                    force[0], force[1], force[2]);

//             // ######################投影到刷牙方向##########################
//             double proj = measured.dot(brushDir);

//             // ########################计算误差##############################
//             double err = proj - targetforcevalue;
            if(targetforcevalue == 0){
                err = 0.0;
            }
//             std::cout << "proj = " << proj << "  err = " << err << std::endl;

//             // ####################计算brushDir与向量(0,0,-1)之间的夹角###########
//             Eigen::Vector3d targetDir(0, 0, -1);
//             double dotProduct = brushDir.dot(targetDir);
//             double brushDirNorm = brushDir.norm();
//             double targetDirNorm = targetDir.norm();
//             double angleRad = std::acos(dotProduct / (brushDirNorm * targetDirNorm));
//             double angleDeg = angleRad * 180.0 / M_PI;

//             // 如果夹角大于6°就执行下面的代码
//             if (angleDeg > 6.0)
//             {
//                 if (std::abs(err) <= 0.04)
//                 {
//                     std::cout << "absbrushDirerr" << err << std::endl;
//                     converged = true;
//                     break;
//                 }
//                 else if (err > 0.04)
//                 {
//                     std::cout << "brushDirerr" << err << std::endl;
//                     Eigen::Vector3d delta = -0.2 * brushDir;
//                     pc1.x += delta.x();
//                     pc1.y += delta.y();
//                     pc1.z += delta.z();
//                 }
//                 else if (err < -0.04)
//                 {
//                     std::cout << "brushDir2err" << err << std::endl;
//                     Eigen::Vector3d delta = 0.2 * brushDir;
//                     pc1.x += delta.x();
//                     pc1.y += delta.y();
//                     pc1.z += delta.z();
//                 }
//             }
//             else
//             {
//                 if (std::abs(err) <= 0.04)
//                 {
//                     converged = true;
//                     break;
//                 }
//                 else if (err > 0.04)
//                 {
//                     Eigen::Vector3d delta = -0.2 * brushDir;
//                     pc1.x += delta.x();
//                     pc1.y += delta.y();
//                     pc1.z += delta.z();
//                 }
//                 else if (err < -0.04)
//                 {
//                     Eigen::Vector3d delta = 0.2 * brushDir;
//                     pc1.x += delta.x();
//                     pc1.y += delta.y();
//                     pc1.z += delta.z();
//                 }
//             }

//             std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         }

//         // ##########################保存修正后的轨迹##########################
//         forcerepairedoutputfile << pc1.x << " "
//                                 << pc1.y << " "
//                                 << pc1.z << " "
//                                 << pc1.rx << " "
//                                 << pc1.ry << " "
//                                 << pc1.rz << std::endl;
//     }

//     forcerepairedoutputfile.close();

  //camera.~CameraCapture();

  return 0;
}
