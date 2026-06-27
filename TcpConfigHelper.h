#pragma once

#include <string>

struct TcpXyz
{
    double x;
    double y;
    double z;
};

// 从 ../defaultconfig/tcp.json 读取 x/y/z；文件不存在或字段缺失时使用默认新标定 TCP
TcpXyz loadDefaultBrushTcp();

std::string makeTcpValueString(double x, double y, double z,
                               double rx = 0.0, double ry = 0.0, double rz = 0.0);
