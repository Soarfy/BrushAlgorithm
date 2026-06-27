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
// #include <iostream>      // 用于 std::cin
#include <limits> // 用于 std::numeric_limits

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

constexpr double STEP_MM = 1; // XYZ 微调步长（mm）

/* ======================= 数据结构 ======================= */

struct PointData
{
  double x, y, z, a, b, c;
};

/* ======================= 键盘微调函数 ======================= */
void fineTuneXYZ(DobotTcpDemo *demo, Dobot::CDescartesPoint &curPose,
                 Eigen::Vector3d &totalOffset)
{
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
        dz += -0.003970;
      }
      else if (key == 13)
      { // Enter
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

inline double degToRad(double degrees)
{
  return degrees * M_PI / 180.0;
}

Eigen ::Matrix3d eulerDegToRotationMatrix(double rx_deg, double ry_deg, double rz_deg)
{
  // 先将角度从度数转换为弧度
  double rx = degToRad(rx_deg);
  double ry = degToRad(ry_deg);
  double rz = degToRad(rz_deg);

  // 使用Eigen的AngleAxisd来创建旋转
  Eigen ::AngleAxisd rollAngle(rx, Eigen::Vector3d::UnitX());
  Eigen ::AngleAxisd pitchAngle(ry, Eigen::Vector3d::UnitY());
  Eigen ::AngleAxisd yawAngle(rz, Eigen::Vector3d::UnitZ());

  Eigen ::Matrix3d rotationMatrix = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();

  return rotationMatrix;
}

// 直接获取旋转后的Z轴方向向量（输入为度数）
Eigen ::Vector3d getRotatedZAxisFromDegrees(double rx_deg, double ry_deg, double rz_deg)
{
  // 先将角度从度数转换为弧度
  double rx = degToRad(rx_deg);
  double ry = degToRad(ry_deg);
  double rz = degToRad(rz_deg);

  // 创建旋转四元数
  Eigen ::AngleAxisd roll(rx, Eigen::Vector3d::UnitX());
  Eigen ::AngleAxisd pitch(ry, Eigen::Vector3d::UnitY());
  Eigen ::AngleAxisd yaw(rz, Eigen::Vector3d::UnitZ());

  // 组合旋转：R = Rz * Ry * Rx
  Eigen ::Quaterniond q = yaw * pitch * roll;

  // 旋转Z轴单位向量
  return q * Eigen::Vector3d::UnitZ();
}

int main()
{
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
  if (hr != 0)
  {
    printf("start capture faield\n");
    return hr;
  }

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@检测机械臂链接状态@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  SetConsoleOutputCP(CP_UTF8); //  UTF-8
  SetConsoleCP(CP_UTF8);       // UTF-8ѡ
  DobotTcpDemo *demo = new DobotTcpDemo();

  struct PointData
  {
    double x, y, z, a, b, c;
  };

  struct BrushVector
  {
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
  // std::cout << "Running Python script to generate brush other..." << std::endl;
  // int python_result1 =
  //     std::system("cmd /c "
  //                 "\"D:\\UsmileProject\\hand_eye_calibration\\."
  //                 "venv312\\Scripts\\activate && python "
  //                 "D:\\UsmileProject\\hand_eye_"
  //                 "calibration\\GeneratePathOffset66initAllnewdorobots.py\"");
  // std::this_thread::sleep_for(std::chrono::seconds(2));

  std::cout << "Running Python script to generate brush other 6..." << std::endl;
  int python_result2 =
      std::system("cmd /c "
                  "\"D:\\UsmileProject\\hand_eye_calibration\\."
                  "venv312\\Scripts\\activate && python "
                  "D:\\UsmileProject\\hand_eye_"
                  "calibration\\GeneratePathOffset66BrushOther6dorobot.py\"");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@先转角度然后将值给到python函数@@@@@@@@@@@@@@@@@@2
  // @@@@@@@@@@@@@@@@@@@@@@@@@@旋转牙刷面@@@@@@@@@@@@@@@@@@
  Dobot::CDescartesPoint rotatetooljoint{};
  rotatetooljoint.x = 0.1;
  rotatetooljoint.y = 0;
  rotatetooljoint.z = 0;
  rotatetooljoint.rx = -10;
  rotatetooljoint.ry = 30;
  rotatetooljoint.rz = 0;

  demo->RelMovJDemo(rotatetooljoint, 0, 3, 20, 50, 100);

  std::cout << "When robot Rotate finished, please press Enter" << std::endl;
  std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

  // ========== 从控制器读取真实位姿（GetPose） ==========
  double gx, gy, gz, grx, gry, grz;

  if (demo->getCurrentPose(0, 0, gx, gy, gz, grx, gry, grz))
  {

    std::ofstream poseFile(
        "D:\\UsmileProject\\hand_eye_calibration\\current_pose_from_getpose.txt");

    if (poseFile.is_open())
    {
      poseFile << gx << " "
               << gy << " "
               << gz << " "
               << grx << " "
               << gry << " "
               << grz << std::endl;

      poseFile.close();
      std::cout << "[INFO] GetPose saved to txt\n";
    }
  }
  else
  {
    std::cerr << "[ERROR] Failed to GetPose\n";
  }

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@生成的轨迹转移到机械臂末端@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  int python_result11 = std::system(
      "cmd /c "
      "\"D:\\UsmileProject\\hand_eye_calibration\\.venv312\\Scripts\\activate "
      "&& python "
      "D:\\UsmileProject\\hand_eye_calibration\\TCPRotation10OthersDorobot.py\"");

  // @@@@@@@@@@@@@@@@@@@@@@@@@先自己写一个txt文件用来走轨迹@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  std::vector<PointData> brushpointsoffset_ee_poses;
  std::ifstream ee_poses_infile(
      "D:\\UsmileProject\\hand_eye_calibration\\saved_ply\\ee_poses.txt");
  if (!ee_poses_infile.is_open())
  {
    std::cerr << "Failed to open ee_poses.txt" << std::endl;
    return -1;
  }

  double dx, dy, dz, rx, ry, rz;
  while (ee_poses_infile >> dx >> dy >> dz >> rx >> ry >> rz)
  {
    std::cout << "Read values: " << dx << " >> " << dy << " >> " << dz << ">> "
              << rx << ">> " << ry << ">> " << rz << std::endl;
    brushpointsoffset_ee_poses.push_back({dx, dy, dz, rx, ry, rz});
  }
  ee_poses_infile.close();

  // /* ========== 到第一个点 + 微调 ========== */
  Eigen::Vector3d totalOffset(0, 0, 0);

  Dobot::CDescartesPoint firstPose{};
  firstPose.x = brushpointsoffset_ee_poses[0].x;
  firstPose.y = brushpointsoffset_ee_poses[0].y;
  firstPose.z = brushpointsoffset_ee_poses[0].z;
  firstPose.rx = brushpointsoffset_ee_poses[0].a;
  firstPose.ry = brushpointsoffset_ee_poses[0].b;
  firstPose.rz = brushpointsoffset_ee_poses[0].c;

  demo->moveRobotC(firstPose, firstPose);
  std::cout << "When robot Rotate finished, please press Enter" << std::endl;
  std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

  fineTuneXYZ(demo, firstPose, totalOffset);

  /* ========== 写回修正后的轨迹 ========== */
  for (auto &p : brushpointsoffset_ee_poses)
  {
    p.x += totalOffset.x();
    p.y += totalOffset.y();
    p.z += totalOffset.z();
  }

  // /* ========== 构造修正后的 MovS 轨迹 ========== */
  std::vector<Dobot::CDescartesPoint> descartesPoints;
  for (auto &p : brushpointsoffset_ee_poses)
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

  // 退回到安全位置
  Dobot::CDescartesPoint pointsafe{};
  pointsafe.x = 264.8929;
  pointsafe.y = -285.1852;
  pointsafe.z = 491.0669;
  pointsafe.rx = -179.7725;
  pointsafe.ry = -1.3507;
  pointsafe.rz = -145.9055;
  demo->moveRobotC(pointsafe, pointsafe);

  Dobot::CDescartesPoint pointstart{};
  pointstart.x = descartesPoints[0].x + -0.827884 * 8;
  pointstart.y = descartesPoints[0].y + -0.560404 * 8;
  pointstart.z = descartesPoints[0].z + 0.023572 * 8;
  pointstart.rx = descartesPoints[0].rx;
  pointstart.ry = descartesPoints[0].ry;
  pointstart.rz = descartesPoints[0].rz;

  demo->moveRobotC(pointstart, pointstart);

  std::cout << "When robot Rotate finished, please press Enter" << std::endl;
  std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

  int countvalue = 1;
  if (!brushpointsoffset_ee_poses.empty())
  {
    Dobot::MovSParams params;
    params.tool = 0;   // 工具坐标系索引，默认值：0
    params.user = 0;   // 用户坐标系索引，默认值：0
    params.v = 80;     // 运动速度比例 (0,100]，默认值：100
    params.a = 80;     // 运动加速度比例 (0,100]，默认值：100
    params.freq = 0.2; // 滤波系数 (0,1]，默认值：0.2，CAD轨迹可设为1保证精度
    std::vector<Dobot::CDescartesPoint> descartesPoints;
    for (const auto &offset : brushpointsoffset_ee_poses)
    {
      Dobot::CDescartesPoint pc1{};
      pc1.x = offset.x;
      pc1.y = offset.y;
      pc1.z = offset.z;
      pc1.rx = offset.a;
      pc1.ry = offset.b;
      pc1.rz = offset.c;
      descartesPoints.push_back(pc1);

      // demo->moveRobotC(pc1, pc1);

      if (countvalue == 1)
      {
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

  std::cout << "When robot Rotate finished, please press Enter" << std::endl;
  std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');

  // 回收安全位置
  demo->moveRobotC(pointsafe, pointsafe);
  demo->moveRobotC(pointstart, pointstart);
  std::cout << "When robot Rotate finished, please press Enter" << std::endl;
  std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@添加力控@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@加载每个轨迹点刷头的方向@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  std::vector<BrushVector> brushvectors;
  std::vector<Dobot::CDescartesPoint> descartesPointsforce;
  std::ifstream brushvectorinfile("D:\\UsmileProject\\hand_eye_calibration\\brushing_vectors.txt");
  if (!brushvectorinfile.is_open())
  {
    std::cerr << "Failed to open brushing_vectors.txt" << std::endl;
    return -1;
  }
  double dxbrush, dybrush, dzbrush;
  while (brushvectorinfile >> dxbrush >> dybrush >> dzbrush)
  {
    brushvectors.push_back({dxbrush, dybrush, dzbrush});
  }
  brushvectorinfile.close();

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@根据期望的力道来调整轨迹@@@@@@@@@@@@@@@@@@@@@@@@@@@
  double targetforcevalue = 0.2;
  std::ofstream forcerepairedoutputfile("D:\\UsmileProject\\hand_eye_calibration\\transformsrepairedforcesideright.txt");

  if (!forcerepairedoutputfile.is_open())
  {
    std::cerr << "Failed to open transformsrepairedforce.txt" << std::endl;
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

  printf("forcefirst0: %.2f forcefirst1: %.2f forcefirst2: %.2f \n",
         forcefirst[0], forcefirst[1], forcefirst[2]);

  for (size_t i = 0; i < descartesPoints.size(); ++i)
  {
    auto &offset = descartesPoints[i];
    const auto &dir = brushvectors[i];
    bool converged = false;

    // #########################单位化牙刷方向向量#######################

    int firstcount = 0;
    while (!converged)
    {

      float force[6];
     while (obj->GetCurrentForceData(force) != 28 && force[2] != 0)
      {
        std::cerr << "failed to get force data\n";
      }

      force[0] -= forcefirst[0];
      force[1] -= forcefirst[1];
      force[2] -= forcefirst[2];

      // 先運動
      std::cout << "start: " << std::endl;
      demo->moveRobotC(offset, offset);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      // 實時獲取刷頭的向量也就是機械臂末端或者刷頭末端的z的朝向
      Eigen::Matrix3d rotationMatrix = eulerDegToRotationMatrix(offset.rx, offset.ry, offset.rz);
      Eigen::Vector3d brushDir = rotationMatrix.col(2);

      printf("vectors0: %.2f vectors1: %.2f vectors2: %.2f \n",
             brushDir.x(), brushDir.y(), brushDir.z());
      brushDir.normalize();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      Eigen::Vector3d measured(force[0], force[1], force[2]);

      printf("force0: %.2f force1: %.2f force2: %.2f \n",
             force[0], force[1], force[2]);

      // ######################投影到刷牙方向##########################
      double proj = measured.dot(brushDir);

      // ########################计算误差##############################
      double err = proj - targetforcevalue;
            if(targetforcevalue == 0){
                err = 0.0;
            }
      std::cout << "proj = " << proj << "  err = " << err << std::endl;

      // ####################计算brushDir与向量(0,0,-1)之间的夹角###########
      Eigen::Vector3d targetDir(0, 0, -1);
      double dotProduct = brushDir.dot(targetDir);
      double brushDirNorm = brushDir.norm();
      double targetDirNorm = targetDir.norm();
      double angleRad = std::acos(dotProduct / (brushDirNorm * targetDirNorm));
      double angleDeg = angleRad * 180.0 / M_PI;

      // 如果夹角大于6°就执行下面的代码
      if (angleDeg > 6.0)
      {
        if (std::abs(err) <= 0.04)
        {
          std::cout << "absbrushDirerr" << err << std::endl;
          converged = true;
          break;
        }
        else if (err > 0.04)
        {
          std::cout << "brushDirerr" << err << std::endl;
          Eigen::Vector3d delta = -0.6 * brushDir;
          offset.x += delta.x();
          offset.y += delta.y();
          offset.z += delta.z();
        }
        else if (err < -0.04)
        {
          std::cout << "brushDir2err" << err << std::endl;
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
  std::cout << "saved new path" << std::endl;

  // 跑一邊含有力控的
  demo->moveRobotC(pointsafe, pointsafe);
  demo->moveRobotC(pointstart, pointstart);
  std::cout << "When robot Rotate finished, please press Enter" << std::endl;
  std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
  demo->moveRobotC(descartesPointsforce[0], descartesPointsforce[0]);

  // 直接走整个轨迹
  Dobot::MovSParams params1;
  params1.tool = 0;   // 工具坐标系索引，默认值：0
  params1.user = 0;   // 用户坐标系索引，默认值：0
  params1.v = 80;     // 运动速度比例 (0,100]，默认值：100
  params1.a = 80;     // 运动加速度比例 (0,100]，默认值：100
  params1.freq = 0.2; // 滤波系数 (0,1]，默认值：0.2，CAD轨迹可设为1保证精度
  demo->movsDemoC(descartesPointsforce, params1);

  camera.~CameraCapture();
  demo->~DobotTcpDemo();
  delete demo;
  demo = nullptr; // 将指针设为nullptr，避免野指针

  return 0;
}
