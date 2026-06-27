#include "VideoCapture.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <conio.h>
#include <crtdbg.h>
#include <fstream>
#include <iostream>
#include <open3d/Open3D.h>
#include <vector>
#include <yaml-cpp/yaml.h>


// 新机械臂的头文件
#include "DobotTcpDemo.h"
#include <iostream>
#include <vector>
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
  pointa.x = 200;
  pointa.y = 0;
  pointa.z = 100;
  pointa.rx = 0;
  pointa.ry = 0;
  pointa.rz = 0;

  Dobot::CDescartesPoint pointb{};
  pointb.x = 300;
  pointb.y = 0;
  pointb.z = 100;
  pointb.rx = 0;
  pointb.ry = 0;
  pointb.rz = 0;

  demo->moveRobotC(pointa, pointb);

  // @@@@@@@@@@@@@@@@@@@@@@@@@旋转拍摄替换图片和点云@@@@@@@@@@@@@@@@@@@@@@@@2
//   int rotate = std::system(
//       "cmd /c "
//       "\"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate "
//       "&& python D:\\UsmileProject\\hand_eye_calibration\\TcpclientLeft.py\"");
//   if (rotate != 0) {
//     std::cerr << "Python script TcpclientLeft.py execution failed!"
//               << std::endl;
//     return -1;
//   }

  camera.captureAndSave(10);

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@生成水平的轨迹@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  std::cout << "Running Python script to generate brush other..." << std::endl;
  int python_result1 =
      std::system("cmd /c "
                  "\"D:\\UsmileProject\\hand_eye_calibration\\."
                  "venv312\\Scripts\\activate && python "
                  "D:\\UsmileProject\\hand_eye_"
                  "calibration\\GeneratePathOffset66initAllnew.py\"");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  std::cout << "Running Python script to generate brush other 6..." << std::endl;
  int python_result2 =
      std::system("cmd /c "
                  "\"D:\\UsmileProject\\hand_eye_calibration\\."
                  "venv312\\Scripts\\activate && python "
                  "D:\\UsmileProject\\hand_eye_"
                  "calibration\\GeneratePathOffset66BrushOther6.py\"");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  if (python_result1 != 0) {
    std::cerr
        << "Python script GeneratePathOffset66BrushOther.py execution failed!"
        << std::endl;
    return -1;
  }

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@生成的轨迹转移到机械臂末端@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//   int python_result11 = std::system(
//       "cmd /c "
//       "\"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate "
//       "&& python "
//       "D:\\UsmileProject\\hand_eye_calibration\\TCPRotation10Others.py\"");

//   if (python_result11 != 0) {
//     std::cerr << "Python script TCPRotation10Others.py execution failed!"
//               << std::endl;
//     return -1;
//   }

//   先自己写一个txt文件用来走轨迹
  std::vector<PointData> brushpointsoffset_ee_poses;
  std::ifstream ee_poses_infile(
      "D:\\UsmileProject\\hand_eye_calibration\\saved_ply\\ee_poses.txt");
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
  if (!brushpointsoffset_ee_poses.empty()) {
    // int countvalue = 12;
    Dobot::MovSParams params;
    params.tool = 0;   // 工具坐标系索引，默认值：0
    params.user = 0;   // 用户坐标系索引，默认值：0
    params.v = 80;     // 运动速度比例 (0,100]，默认值：100
    params.a = 80;     // 运动加速度比例 (0,100]，默认值：100
    params.freq = 0.2; // 滤波系数 (0,1]，默认值：0.2，CAD轨迹可设为1保证精度
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
      // demo->moveRobotC(pc1, pc1);
      // camera.captureAndSave(countvalue);
      // countvalue += 1;
      
    }
    // 直接走整个轨迹
    demo->movsDemoC(descartesPoints, params);
  }

//   // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@基于已经修复的轨迹来实现力控调整部分@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//   std::vector<PointData> brushpointsoffsetrepair;
//   std::vector<PointData> brushpointsoffsetrepairforce;
//   std::vector<BrushVector> brushvectors;

//   //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@加载修复好的轨迹用于跑轨迹@@@@@@@@@@@@@@@@@@@@@@@
//   std::ifstream infilerepair(
//       "D:\\UsmileProject\\hand_eye_calibration\\transformsrepaired.txt");
//   if (!infilerepair.is_open()) {
//     std::cerr << "Failed to open transformsrepaired.txt" << std::endl;
//     return -1;
//   }
//   double dxrepair, dyrepair, dzrepair, rxrepair, ryrepair, rzrepair;
//   while (infilerepair >> dxrepair >> dyrepair >> dzrepair >> rxrepair >>
//          ryrepair >> rzrepair) {
//     brushpointsoffsetrepair.push_back(
//         {dxrepair, dyrepair, dzrepair, rxrepair, ryrepair, rzrepair});
//   }
//   infilerepair.close();

//   // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@加载每个轨迹点刷头的方向@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//   std::ifstream brushvectorinfile(
//       "D:\\UsmileProject\\hand_eye_calibration\\brushing_vectors.txt");
//   if (!brushvectorinfile.is_open()) {
//     std::cerr << "Failed to open brushing_vectors.txt" << std::endl;
//     return -1;
//   }
//   double dxbrush, dybrush, dzbrush;
//   while (brushvectorinfile >> dxbrush >> dybrush >> dzbrush) {
//     brushvectors.push_back({dxbrush, dybrush, dzbrush});
//   }
//   brushvectorinfile.close();

//   // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@根据期望的力道来调整轨迹@@@@@@@@@@@@@@@@@@@@@@@@@@@
//   double targetforcevalue = 0.2;
//   std::ofstream forcerepairedoutputfile(
//       "D:\\UsmileProject\\hand_eye_"
//       "calibration\\transformsrepairedforcesideleft.txt");

//   if (!forcerepairedoutputfile.is_open()) {
//     std::cerr << "Failed to open transformsrepairedforce.txt" << std::endl;
//     return -1;
//   }

//   // @@@@@@@@@@@@@@@@@@@对6维力清零@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//   float forcefirst[6];
//   while (obj->GetCurrentForceData(forcefirst) != 28 && forcefirst[2] != 0) {
//     std::cerr << "Get first data error\n";
//   }

//   printf("forcefirst0: %.2f forcefirst1: %.2f forcefirst2: %.2f \n",
//          forcefirst[0], forcefirst[1], forcefirst[2]);

//   // @@@@@@@@@@@@@@@@@@@@@@@@@@@边跑修复好的轨迹边调整力道@@@@@@@@@@@@@@@@@@@@@@@@@@@
//   std::vector<PointData> pointsnewstart;
//   c2::Speed forcespeed{20, 20, 20};
//   c2::Acc forceacc{10, 10, 10};

//   pointsnewstart = {
//       {25.411, 452.19, 422.686, -179.534, -0.215, 90.558},
//   };

//   // @@@@@@@@@@@@@@@@@@@@@先跑到起始位置@@@@@@@@@@@@@@@@@@@@@@@2
//   for (const auto &pt : pointsnewstart) {
//     c2::MovJointSegments definetraj;
//     c2::Point cpos1;
//     cpos1.type = c2::PointType::Cart;
//     cpos1.cpos.x = pt.x;
//     cpos1.cpos.y = pt.y;
//     cpos1.cpos.z = pt.z;
//     cpos1.cpos.a = pt.a;
//     cpos1.cpos.b = pt.b;
//     cpos1.cpos.c = pt.c;
//     definetraj.AddMovJ(cpos1, speed, acc, c2::ZoneType::Fine, zone);
//     arm.StartBrushTeethTrajectoryJoint(definetraj);
//   }

//   for (size_t i = 0; i < brushpointsoffsetrepair.size(); ++i) {
//     auto &offset = brushpointsoffsetrepair[i];
//     const auto &dir = brushvectors[i];
//     bool converged = false;

//     // #########################单位化牙刷方向向量#######################
//     Eigen::Vector3d brushDir(dir.x, dir.y, dir.z);
//     brushDir.normalize();

//     c2::Point cpos;
//     cpos.type = c2::PointType::Cart;
//     cpos.cpos.x = offset.x;
//     cpos.cpos.y = offset.y;
//     cpos.cpos.z = offset.z;
//     cpos.cpos.a = offset.a;
//     cpos.cpos.b = offset.b;
//     cpos.cpos.c = offset.c;

//     int firstcount = 0;
//     while (!converged) {

//       float force[6];
//      while (obj->GetCurrentForceData(force) != 28 && force[2] != 0) {
//         std::cerr << "failed to get force data\n";
//       }

//       force[0] -= forcefirst[0];
//       force[1] -= forcefirst[1];
//       force[2] -= forcefirst[2];

//       std::cout << "start: " << std::endl;
//       c2::MovJointSegments brushing;
//       brushing.AddMovJ(cpos, forcespeed, forceacc, c2::ZoneType::Fine, zone);
//       arm.StartBrushTeethTrajectoryJoint(brushing);

//       Eigen::Vector3d measured(force[0], force[1], force[2]);

//       // std::cout << "measured " << measured << std::endl;
//       // std::cout << " brushDir" << brushDir << std::endl;

//       // ######################投影到刷牙方向##########################
//       double proj = measured.dot(brushDir);

//       // ########################计算误差##############################
//       double err = proj - targetforcevalue;
            if(targetforcevalue == 0){
                err = 0.0;
            }
//       // std::cout << "proj = " << proj << "  err = " << err << std::endl;

//       // ####################计算brushDir与向量(0,0,-1)之间的夹角###########
//       Eigen::Vector3d targetDir(0, 0, -1);
//       double dotProduct = brushDir.dot(targetDir);
//       double brushDirNorm = brushDir.norm();
//       double targetDirNorm = targetDir.norm();
//       double angleRad = std::acos(dotProduct / (brushDirNorm * targetDirNorm));
//       double angleDeg = angleRad * 180.0 / M_PI;

//       // 如果夹角大于6°就执行下面的代码
//       if (angleDeg > 6.0) {
//         // std::cout << " hello " << std::endl;
//         if (std::abs(err) <= 0.04) {
//           converged = true;
//           break;
//         } else if (err > 0.04) {
//           // std::cout << "brushDir" << brushDir << std::endl;
//           Eigen::Vector3d delta = -0.2 * brushDir;
//           cpos.cpos.x += delta.x();
//           cpos.cpos.y += delta.y();
//           cpos.cpos.z += delta.z();
//         } else if (err < -0.04) {
//           // std::cout << "brushDir2" << brushDir << std::endl;
//           Eigen::Vector3d delta = 0.2 * brushDir;
//           cpos.cpos.x += delta.x();
//           cpos.cpos.y += delta.y();
//           cpos.cpos.z += delta.z();
//         }
//       } else {
//         // std::cout << " world " << std::endl;
//         if (std::abs(err) <= 0.04) {
//           converged = true;
//           break;
//         } else if (err > 0.04) {
//           // std::cout << "Else brushDir" << brushDir << std::endl;
//           Eigen::Vector3d delta = -0.2 * brushDir;
//           cpos.cpos.x += delta.x();
//           cpos.cpos.y += delta.y();
//           cpos.cpos.z += delta.z();
//         } else if (err < -0.04) {
//           // std::cout << "Else brushDir2" << brushDir << std::endl;
//           Eigen::Vector3d delta = 0.2 * brushDir;
//           cpos.cpos.x += delta.x();
//           cpos.cpos.y += delta.y();
//           cpos.cpos.z += delta.z();
//         }
//       }

//       std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }

//     // ##########################保存修正后的轨迹##########################
//     forcerepairedoutputfile << cpos.cpos.x << " " << cpos.cpos.y << " "
//                             << cpos.cpos.z << " " << cpos.cpos.a << " "
//                             << cpos.cpos.b << " " << cpos.cpos.c << std::endl;
//   }

//   forcerepairedoutputfile.close();
//   obj->StopCapture();

//   for (const auto &pt : points) {
//     c2::MovJointSegments definetraj;
//     c2::Point cpos1;
//     cpos1.type = c2::PointType::Cart;
//     cpos1.cpos.x = pt.x;
//     cpos1.cpos.y = pt.y;
//     cpos1.cpos.z = pt.z;
//     cpos1.cpos.a = pt.a;
//     cpos1.cpos.b = pt.b;
//     cpos1.cpos.c = pt.c;
//     definetraj.AddMovJ(cpos1, speed, acc, c2::ZoneType::Fine, zone);
//     arm.StartBrushTeethTrajectoryJoint(definetraj);
//   }

  
//   std::vector<PointData> pointsnewly;
//   pointsnewly = {
//       {25.411, 452.19, 422.686, -179.534, -0.215, 90.558},
//   };
//   for (const auto &pt : pointsnewly) {
//     c2::MovJointSegments definetraj;
//     c2::Point cpos1;
//     cpos1.type = c2::PointType::Cart;
//     cpos1.cpos.x = pt.x;
//     cpos1.cpos.y = pt.y;
//     cpos1.cpos.z = pt.z;
//     cpos1.cpos.a = pt.a;
//     cpos1.cpos.b = pt.b;
//     cpos1.cpos.c = pt.c;
//     definetraj.AddMovJ(cpos1, speed, acc, c2::ZoneType::Fine, zone);
//     arm.StartBrushTeethTrajectoryJoint(definetraj);
//   }
//   int rotate2 = std::system(
//       "cmd /c "
//       "\"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate "
//       "&& python D:\\UsmileProject\\hand_eye_calibration\\Tcpclient.py\"");
//   if (rotate2 != 0) {
//     std::cerr << "Python script Tcpclient.py execution failed!" << std::endl;
//     return -1;
//   }

  camera.~CameraCapture();
  std::cout << "changeRobotMode: "
            << arm.changeRobotMode(c2::UserCommand::ToReady) << std::endl;

  return 0;
}
