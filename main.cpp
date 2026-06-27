#include "DobotTcpDemo.h"
#include <windows.h>
#include <iostream>
#include <vector>

int main()
{
    SetConsoleOutputCP(CP_UTF8);   //  UTF-8
    SetConsoleCP(CP_UTF8);         // UTF-8ѡ
    DobotTcpDemo* demo = new DobotTcpDemo();
    
    // #########################原来的往返关节运动#########################
    //demo->moveRobot();

    ////// 基于笛卡尔坐标系的运动控制
    ////pc1.x = 300; pc1.y = -450;   pc1.z = 449; pc1.rx = 170; pc1.ry = 0; pc1.rz = -179;
    Dobot::CDescartesPoint pointa{};
    ///*pointa.x = 290; pointa.y = -433; pointa.z = 448;
    //pointa.rx = 170;  pointa.ry = -2; pointa.rz = -178;*/

    pointa.x = 194.756; pointa.y = -346.646; pointa.z = 307.00;
    pointa.rx = -179.2547;  pointa.ry = 12.967; pointa.rz = -132.6958;

    Dobot::CDescartesPoint pointb{};
    pointb.x = 194.756; pointb.y = -346.646; pointb.z = 307.00;
    pointb.rx = -179.2547;  pointb.ry = 12.967; pointb.rz = -132.6958;

    demo->moveRobotC(pointa, pointb);



    // #########################原来的往返关节运动#########################
//    Dobot::CDescartesPoint fixedPoint{};
//    fixedPoint.x = 257.290797;
//    fixedPoint.y = -312.022714;
//    fixedPoint.z = 363.873350;
//    fixedPoint.rx = -179.772500;
//    fixedPoint.ry = -1.350700;
//    fixedPoint.rz = -145.697633;
//
//
//    // 創建所有點的vector並初始化
//    std::vector<Dobot::CDescartesPoint> points = {
//        {264.8929,-285.1852,391.0669,-179.7725,-1.3507,-145.9055},
//       {221.430058, -343.996852, 357.733578, -179.773175,  -1.350675,  -131.198826},
//{ 231.359129, -331.632605, 357.733578, -179.772740,  -1.350673,  -135.651577 },
//{ 242.226662, -320.057732, 357.733578, -179.772552,  -1.350690,  -140.107845 },
//{ 253.965141, -309.347999, 357.733578, -179.772502,  -1.350700,  -144.566311 },
//{ 266.499698, -299.573297, 357.733578, -179.772505,  -1.350698,  -149.025312 },
//{ 279.748870, -290.796620, 357.733578, -179.772511,  -1.350695,  -153.482913 },
//{ 293.625495, -283.073163, 357.733578, -179.772506,  -1.350697,  -157.936991 },
//{ 308.037764, -276.449598, 357.733578, -179.772504,  -1.350697,  -162.385315 },
//{ 322.890457, -270.963560, 357.733578, -179.772552,  -1.350660,  -166.825645 },
//{ 338.086158, -266.643343, 357.733578, -179.772710,  -1.350509,  -171.255813 },
//{ 353.526578, -263.507837, 357.733578, -179.773042,  -1.350128,  -175.673816 },
//{ 353.369157, -259.686982, 357.733578, -179.773042,  -1.350128,  -175.673816 }
//    };
//
//    // 使用for循環調用moveRobotC
//    for (const auto& point : points) {
//        demo->moveRobotC(point, fixedPoint);
//    }




    //// ########################基于joint坐标的moveS##########################
    //std::vector<Dobot::CJointPoint> jointPoints;
    //
    //Dobot::CJointPoint p1{};
    //p1.j1 = -90; p1.j2 = 20; p1.j3 = 0; p1.j4 = 0; p1.j5 = 0; p1.j6 = 0;
    //Dobot::CJointPoint p2{};
    //p2.j1 = -45; p2.j2 = 30; p2.j3 = 10; p2.j4 = 0; p2.j5 = 0; p2.j6 = 0;
    //Dobot::CJointPoint p3{};
    //p3.j1 =  45; p3.j2 = 30; p3.j3 = 10; p3.j4 = 0; p3.j5 = 0; p3.j6 = 0;
    //Dobot::CJointPoint p4{};
    //p4.j1 =  90; p4.j2 = 20; p4.j3 = 0;  p4.j4 = 0; p4.j5 = 0; p4.j6 = 0;
    //
    //jointPoints.push_back(p1);
    //jointPoints.push_back(p2);
    //jointPoints.push_back(p3);
    //jointPoints.push_back(p4);
    //
    //// 2. 构造 MovS 参数（所有参数都有默认值，只需要修改你想改变的参数）
    Dobot::MovSParams params;
    params.tool = 0;      // 工具坐标系索引，默认值：0
    params.user = 0;      // 用户坐标系索引，默认值：0
    params.v = 80;        // 运动速度比例 (0,100]，默认值：100
    params.a = 80;        // 运动加速度比例 (0,100]，默认值：100
    params.freq = 0.2;    // 滤波系数 (0,1]，默认值：0.2，CAD轨迹可设为1保证精度
   // 
   // //// 3. 调用 movsDemo
   // //demo->movsDemo(jointPoints, params);

   // // ####################使用笛卡尔坐标的 MovS 轨迹拟合示例################
    std::vector<Dobot::CDescartesPoint> descartesPoints;
    
    Dobot::CDescartesPoint pc1{};
    pc1.x = 194.756; pc1.y = -346.646;   pc1.z = 307.00; pc1.rx = -179.2547; pc1.ry = 12.967; pc1.rz = -132.6958;
    Dobot::CDescartesPoint pc2{};
    pc2.x = 232.8364; pc2.y = -305.1595;  pc2.z = 313.9742; pc2.rx = 178.4709; pc2.ry = 9.5169; pc2.rz = -146.1730;
    Dobot::CDescartesPoint pc3{};
    pc3.x = 291.0390; pc3.y = -264.5740;   pc3.z = 311.2894; pc3.rx = 178.8747; pc3.ry = 5.7035; pc3.rz = -162.1718;
    Dobot::CDescartesPoint pc4{};   
    pc4.x = 333.6250; pc4.y = -250.0627; pc4.z = 308.8812; pc4.rx = 179.0202; pc4.ry = 5.9597; pc4.rz = -176.7155;
    Dobot::CDescartesPoint pc5{};
    pc5.x = -75; pc5.y = -535; pc5.z = 448; pc5.rx = 169; pc5.ry = 0; pc5.rz = 139;
    
    descartesPoints.push_back(pc1);
    descartesPoints.push_back(pc2);
    descartesPoints.push_back(pc3);
    descartesPoints.push_back(pc4);
    descartesPoints.push_back(pc5);
    
    // 使用相同的 params 参数
     demo->movsDemoC(descartesPoints, params);
}
