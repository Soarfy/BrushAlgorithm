#include "VideoCapture.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <conio.h>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>
#include <windows.h>

// 机械臂
#include "DobotTcpDemo.h"

// 六维力
#include "kw-lib-all.h"
#define MODE 0
NS_KW_USING

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr double STEP_MM = 0.2; // XYZ 微调步长（mm）

/* ======================= 数据结构 ======================= */

struct PointData {
  double x, y, z, a, b, c;
};

/* ======================= 键盘微调函数 ======================= */

void fineTuneXYZ(DobotTcpDemo *demo, Dobot::CDescartesPoint &curPose,
                 Eigen::Vector3d &totalOffset) {
  std::cout << "\n===== XYZ 微调模式 =====\n"
            << "W/S : +Y / -Y\n"
            << "A/D : -X / +X\n"
            << "Q/E : +Z / -Z\n"
            << "Enter : 结束微调\n";

  while (true) {
    if (_kbhit()) {
      char key = _getch();

      double dx = 0, dy = 0, dz = 0;

      if (key == 'w')
        dy += STEP_MM;
      else if (key == 's')
        dy -= STEP_MM;
      else if (key == 'a')
        dx -= STEP_MM;
      else if (key == 'd')
        dx += STEP_MM;
      else if (key == 'q')
        dz += STEP_MM;
      else if (key == 'e')
        dz -= STEP_MM;
      else if (key == 13) { // Enter
        std::cout << "微调结束\n";
        break;
      } else {
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

/* ======================= main ======================= */

int main() {
  std::cout << "Start BrushSideLeft" << std::endl;

  /* ========== 相机初始化 ========== */
  CameraCapture camera("169.254.7.168");

  /* ========== 六维力初始化 ========== */
#if MODE == 0
  SerialControlCreator uc;
  uc.serialPortName = "\\\\.\\COM3";
  uc.baudRate = 460800;
#endif

  auto control = uc.createIOControler();
  HeadTailProtocolCreator htc;
  auto proto = htc.createProtocol();
  SensorControlCreator scc;
  scc.ioCtrl = control;
  scc.proto = proto;
  auto sensor = scc.createSensorControl();
  if (sensor->StartCapture() != 0) {
    std::cerr << "Force sensor start failed\n";
    return -1;
  }

  /* ========== 机械臂初始化 ========== */
  DobotTcpDemo *demo = new DobotTcpDemo();

  /* ========== 回初始位姿 ========== */
  Dobot::CDescartesPoint home{};
  home.x = 264.8929;
  home.y = -285.1852;
  home.z = 391.0669;
  home.rx = -179.7725;
  home.ry = -1.3507;
  home.rz = -145.9055;

  demo->moveRobotC(home, home);

  std::cout << "Press Enter after robot arrives...\n";
  std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

  camera.captureAndSave(10);

  /* ========== Python 生成轨迹 ========== */
  // std::cout << "Running Python script to generate brush other..." << std::endl;
  // int python_result1 =
  //     std::system("cmd /c "
  //                 "\"D:\\UsmileProject\\hand_eye_calibration\\."
  //                 "venv312\\Scripts\\activate && python "
  //                 "D:\\UsmileProject\\hand_eye_"
  //                 "calibration\\GeneratePathOffset66initAllnew.py\"");
  // std::this_thread::sleep_for(std::chrono::seconds(2));

  // system(
  //     "cmd /c "
  //     "\"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate "
  //     "&& python "
  //     "D:\\UsmileProject\\hand_eye_"
  //     "calibration\\GeneratePathOffset66BrushOther6dorobot.py\"");

  // std::this_thread::sleep_for(std::chrono::seconds(2));

  // system(
  //     "cmd /c "
  //     "\"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate "
  //     "&& python "
  //     "D:\\UsmileProject\\hand_eye_calibration\\TCPRotation10OthersDorobot."
  //     "py\"");

  // std::this_thread::sleep_for(std::chrono::seconds(2));

  /* ========== 读取 EE 轨迹 ========== */
  std::vector<PointData> brushpointsoffset_ee_poses;
  std::ifstream infile(
      "D:\\UsmileProject\\hand_eye_calibration\\saved_ply\\ee_poses.txt");

  if (!infile.is_open()) {
    std::cerr << "Failed to open ee_poses.txt\n";
    return -1;
  }

  double x, y, z, rx, ry, rz;
  while (infile >> x >> y >> z >> rx >> ry >> rz) {
    brushpointsoffset_ee_poses.push_back({x, y, z, rx, ry, rz});
  }
  infile.close();

  if (brushpointsoffset_ee_poses.empty()) {
    std::cerr << "轨迹为空\n";
    return -1;
  }

  /* ========== 到第一个点 + 微调 ========== */
  Eigen::Vector3d totalOffset(0, 0, 0);

  Dobot::CDescartesPoint firstPose{};
  firstPose.x = brushpointsoffset_ee_poses[0].x;
  firstPose.y = brushpointsoffset_ee_poses[0].y;
  firstPose.z = brushpointsoffset_ee_poses[0].z;
  firstPose.rx = brushpointsoffset_ee_poses[0].a;
  firstPose.ry = brushpointsoffset_ee_poses[0].b;
  firstPose.rz = brushpointsoffset_ee_poses[0].c;

  demo->moveRobotC(firstPose, firstPose);
  fineTuneXYZ(demo, firstPose, totalOffset);

  /* ========== 写回修正后的轨迹 ========== */
  for (auto &p : brushpointsoffset_ee_poses) {
    p.x += totalOffset.x();
    p.y += totalOffset.y();
    p.z += totalOffset.z();
  }

  /* ========== 构造修正后的 MovS 轨迹 ========== */
  std::vector<Dobot::CDescartesPoint> descartesPoints;
  for (auto &p : brushpointsoffset_ee_poses) {
    Dobot::CDescartesPoint cp{};
    cp.x = p.x;
    cp.y = p.y;
    cp.z = p.z;
    cp.rx = p.a;
    cp.ry = p.b;
    cp.rz = p.c;
    descartesPoints.push_back(cp);
  }

  /* ========== 关键：先到修正后的第一个点 ========== */
  Dobot::CDescartesPoint correctedFirst = descartesPoints.front();

  std::cout << "[INFO] Move to corrected first point...\n";
  demo->moveRobotC(correctedFirst, correctedFirst);

  std::cout << "Press Enter to start MovS trajectory...\n";
  std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

  /* ========== 执行 MovS 连续轨迹 ========== */
  Dobot::MovSParams params;
  params.tool = 0;
  params.user = 0;
  params.v = 80;
  params.a = 80;
  params.freq = 0.2;

  demo->movsDemoC(descartesPoints, params);

  std::cout << "Trajectory finished.\n";

  // // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@添加力控@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@加载每个轨迹点刷头的方向@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // std::vector<BrushVector> brushvectors;
  // std::ifstream brushvectorinfile(
  //     "D:\\UsmileProject\\hand_eye_calibration\\brushing_vectors.txt");
  // if (!brushvectorinfile.is_open()) {
  //   std::cerr << "Failed to open brushing_vectors.txt" << std::endl;
  //   return -1;
  // }
  // double dxbrush, dybrush, dzbrush;
  // while (brushvectorinfile >> dxbrush >> dybrush >> dzbrush) {
  //   brushvectors.push_back({dxbrush, dybrush, dzbrush});
  // }
  // brushvectorinfile.close();

  // // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@根据期望的力道来调整轨迹@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // double targetforcevalue = 0.2;
  // std::ofstream forcerepairedoutputfile(
  //     "D:\\UsmileProject\\hand_eye_"
  //     "calibration\\transformsrepairedforcesideright.txt");

  // if (!forcerepairedoutputfile.is_open()) {
  //   std::cerr << "Failed to open transformsrepairedforce.txt" << std::endl;
  //   return -1;
  // }

  // // @@@@@@@@@@@@@@@@@@@对6维力清零@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // float forcefirst[6];

  // int sampleCount = 0;
  // const int maxSamples = 30;
  // bool success = false;

  // // 采样最多10次，取最后一次满足条件的采样
  // while (sampleCount < maxSamples) {
  //   // 获取力数据
  //   int result = obj->GetCurrentForceData(forcefirst);

  //   // 检查条件是否满足
  //   if (result == 28 && forcefirst[2] > -3.0 && forcefirst[0] < 1.0 && forcefirst[2] < -1.0) {
  //     success = true;
  //     break; // 条件满足，跳出循环
  //   }

  //   sampleCount++;

  //   // 如果条件不满足且未达到最大采样次数，继续采样
  //   if (sampleCount < maxSamples) {
  //     std::cerr << "采样 " << sampleCount << " 次失败，继续采样...\n";
  //   } else {
  //     std::cerr << "采样 " << sampleCount << " 次均失败\n";
  //   }
  // }

  // // 检查最终结果
  // if (!success) {
  //   std::cerr << "Get first data error: 30次采样均未满足条件\n";
  // }

  // printf("forcefirst0: %.2f forcefirst1: %.2f forcefirst2: %.2f \n",
  //        forcefirst[0], forcefirst[1], forcefirst[2]);

  // // @@@@@@@@@@@@@@@@@@@@@@边运动边调整力@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // for (size_t i = 0; i < brushpointsoffset_ee_poses.size(); ++i) {
  //   auto &offset = brushpointsoffset_ee_poses[i];
  //   const auto &dir = brushvectors[i];
  //   bool converged = false;

  //   // #########################单位化牙刷方向向量#######################
  //   Eigen::Vector3d brushDir(dir.x, dir.y, dir.z);
  //   brushDir.normalize();

  //   Dobot::CDescartesPoint pc1{};
  //   pc1.x = offset.x;
  //   pc1.y = offset.y;
  //   pc1.z = offset.z;
  //   pc1.rx = offset.a;
  //   pc1.ry = offset.b;
  //   pc1.rz = offset.c;

  //   int firstcount = 0;
  //   while (!converged) {

  //     float force[6];
  //    while (obj->GetCurrentForceData(force) != 28 && force[2] != 0) {
  //       std::cerr << "failed to get force data\n";
  //     }

  //     force[0] -= forcefirst[0];
  //     force[1] -= forcefirst[1];
  //     force[2] -= forcefirst[2];

  //     std::cout << "start: " << std::endl;
  //     // 挪动一下没有调整的位置
  //     demo->moveRobotC(pc1, pc1);

  //     Eigen::Vector3d measured(force[0], force[1], force[2]);

  //     printf("force0: %.2f force1: %.2f force2: %.2f \n", force[0], force[1],
  //            force[2]);

  //     // ######################投影到刷牙方向##########################
  //     double proj = measured.dot(brushDir);

  //     // ########################计算误差##############################
  //     double err = proj - targetforcevalue;
            if(targetforcevalue == 0){
                err = 0.0;
            }
  //     std::cout << "proj = " << proj << "  err = " << err << std::endl;

  //     // ####################计算brushDir与向量(0,0,-1)之间的夹角###########
  //     Eigen::Vector3d targetDir(0, 0, -1);
  //     double dotProduct = brushDir.dot(targetDir);
  //     double brushDirNorm = brushDir.norm();
  //     double targetDirNorm = targetDir.norm();
  //     double angleRad = std::acos(dotProduct / (brushDirNorm * targetDirNorm));
  //     double angleDeg = angleRad * 180.0 / M_PI;

  //     // 如果夹角大于6°就执行下面的代码
  //     if (angleDeg > 6.0) {
  //       if (std::abs(err) <= 0.04) {
  //         std::cout << "absbrushDirerr" << err << std::endl;
  //         converged = true;
  //         break;
  //       } else if (err > 0.04) {
  //         std::cout << "brushDirerr" << err << std::endl;
  //         Eigen::Vector3d delta = -0.2 * brushDir;
  //         pc1.x += delta.x();
  //         pc1.y += delta.y();
  //         pc1.z += delta.z();
  //       } else if (err < -0.04) {
  //         std::cout << "brushDir2err" << err << std::endl;
  //         Eigen::Vector3d delta = 0.2 * brushDir;
  //         pc1.x += delta.x();
  //         pc1.y += delta.y();
  //         pc1.z += delta.z();
  //       }
  //     } else {
  //       if (std::abs(err) <= 0.04) {
  //         converged = true;
  //         break;
  //       } else if (err > 0.04) {
  //         Eigen::Vector3d delta = -0.2 * brushDir;
  //         pc1.x += delta.x();
  //         pc1.y += delta.y();
  //         pc1.z += delta.z();
  //       } else if (err < -0.04) {
  //         Eigen::Vector3d delta = 0.2 * brushDir;
  //         pc1.x += delta.x();
  //         pc1.y += delta.y();
  //         pc1.z += delta.z();
  //       }
  //     }

  //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
  //   }

  //   // ##########################保存修正后的轨迹##########################
  //   forcerepairedoutputfile << pc1.x << " " << pc1.y << " " << pc1.z << " "
  //                           << pc1.rx << " " << pc1.ry << " " << pc1.rz
  //                           << std::endl;
  // }

  // forcerepairedoutputfile.close();

  return 0;
}
