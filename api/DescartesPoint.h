#pragma once
#include <sstream>
#include <string>

namespace Dobot
{
struct CDescartesPoint
{
public:
    /// <summary>
    /// X 轴位置，单位：毫米
    /// </summary>
    double x;

    /// <summary>
    /// Y 轴位置，单位：毫米
    /// </summary>
    double y;

    /// <summary>
    /// Z 轴位置，单位：毫米
    /// </summary>
    double z;

    /// <summary>
    /// Rx 轴位置，单位：度
    /// </summary>
    double rx;

    /// <summary>
    /// Ry 轴位置，单位：度
    /// </summary>
    double ry;

    /// <summary>
    /// Rz 轴位置，单位：度
    /// </summary>
    double rz;

    std::string ToString() const
    {
        std::ostringstream oss;
        oss << x << ',' << y << ',' << z << ',' << rx << ',' << ry << ',' << rz;
        return oss.str();
    }
};

struct CForcePoint
{
    public:
        /// <summary>
        /// X 轴目标力，单位：毫米
        /// </summary>
        double fx;

        /// <summary>
        /// Y 轴目标力，单位：毫米
        /// </summary>
        double fy;

        /// <summary>
        /// Z 轴目标力，单位：毫米
        /// </summary>
        double fz;

        /// <summary>
        /// Rx 轴目标力，单位：度
        /// </summary>
        double frx;

        /// <summary>
        /// Ry 轴目标力，单位：度
        /// </summary>
        double fry;

        /// <summary>
        /// Rz 轴目标力，单位：度
        /// </summary>
        double frz;

        std::string ToString()
        {
            std::ostringstream oss;
            oss << fx << ',' << fy << ',' << fz << ',' << frx << ',' << fry << ',' << frz;
            return oss.str();
        }
};

struct COffsetPoint
{
    public:
        /// <summary>
        /// X 轴偏移量
        /// </summary>
        double offsetX;

        /// <summary>
        /// Y 轴偏移量
        /// </summary>
        double offsetY;

        /// <summary>
        /// Z 轴偏移量
        /// </summary>
        double offsetZ;

        /// <summary>
        /// Rx 轴偏移量
        /// </summary>
        double offsetRx;

        /// <summary>
        /// Ry 轴偏移量
        /// </summary>
        double offsetRy;

        /// <summary>
        /// Rz 轴偏移量
        /// </summary>
        double offsetRz;

        std::string ToString()
        {
            std::ostringstream oss;
            oss << offsetX << ',' << offsetY << ',' << offsetZ << ',' << offsetRx << ',' << offsetRy << ',' << offsetRz;
            return oss.str();
        }
};

struct ModeDistanceIndexStatus
{
public:
    // 设置Distance模式
    int Mode;

    // 运行指定的距离
    int Distance;

    // 数字输出索引
    int Index;

    // 数字输出状态
    int Status;
    std::string ToString()
    {
        std::ostringstream oss;
        oss << "{" << Mode << ',' << Distance << ',' << Index << ',' << Status << "}";
        return oss.str();
    }
};

#include <string>
#include <sstream>

struct MovSParams
{
public:
    // 指定轨迹点位对应的工具坐标系索引，不指定时使用轨迹文件中记录的工具坐标系索引
    int tool = 0;  // 默认值：0

    // 指定轨迹点位对应的用户坐标系索引，不指定时使用轨迹文件中记录的用户坐标系索引
    int user = 0;  // 默认值：0

    // 执行该条指令时的机械臂运动速度比例。取值范围：(0,100]，设置 speed 后此参数将被忽略
    double v = 100;  // 默认值：100

    // 执行该条指令时的机械臂运动目标速度，取值范围：[1, 最大运动速度]，单位：mm/s
    // 设置此参数后，参数 v 将被忽略
    bool hasSpeed = true;  // 是否设置了 speed 参数
    double speed = 100;       // speed 的值，仅当 hasSpeed 为 true 时使用

    // 执行该条指令时的机械臂运动加速度比例。取值范围：(0,100]
    double a = 100;  // 默认值：100

    // 滤波系数
    double freq = 1;  // 默认值：0.2

    // 停止条件表达式
    std::string stopcond = "";  // 默认值：空字符串

    // ✅ 关键修改：加 const
    std::string ToString() const
    {
        std::ostringstream oss;
        bool first = true;
        //oss << "{";

        // tool
        if (!first) oss << ",";
        oss << "tool=" << tool;
        first = false;

        // user
        if (!first) oss << ",";
        oss << "user=" << user;
        first = false;

        // v
        if (!first) oss << ",";
        oss << "v=" << v;
        first = false;

        // speed（仅当 hasSpeed 为 true）
        if (hasSpeed) {
            if (!first) oss << ",";
            oss << "speed=" << speed;
            first = false;
        }

        // a
        if (!first) oss << ",";
        oss << "a=" << a;
        first = false;

        // freq
        if (!first) oss << ",";
        oss << "freq=" << freq;
        first = false;

        // stopcond
      /*  if (!first) oss << ",";
        oss << "stopcond=\"" << stopcond << "\"";*/

        //oss << "}";
        return oss.str();
    }
};

}    // namespace Dobot
