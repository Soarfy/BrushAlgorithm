#include "TcpConfigHelper.h"

#include <cmath>
#include <fstream>

#include "nlohmann/json.hpp"

namespace
{
constexpr const char *kTcpJsonPath = "../defaultconfig/tcp.json";
constexpr double kDefaultTcpX = -9.352824;
constexpr double kDefaultTcpY = -186.998296;
constexpr double kDefaultTcpZ = 224.724733;

bool isUnsetBrushTcp(const TcpXyz &tcp)
{
    return std::abs(tcp.x) < 1e-9 && std::abs(tcp.y) < 1e-9 && std::abs(tcp.z) < 1e-9;
}

TcpXyz defaultBrushTcp()
{
    return TcpXyz{kDefaultTcpX, kDefaultTcpY, kDefaultTcpZ};
}
} // namespace

TcpXyz loadDefaultBrushTcp()
{
    TcpXyz tcp = defaultBrushTcp();
    try
    {
        std::ifstream file(kTcpJsonPath);
        if (!file.is_open())
        {
            return tcp;
        }

        nlohmann::json j;
        file >> j;
        TcpXyz fromJson = tcp;
        if (j.contains("x"))
        {
            fromJson.x = j["x"].get<double>();
        }
        if (j.contains("y"))
        {
            fromJson.y = j["y"].get<double>();
        }
        if (j.contains("z"))
        {
            fromJson.z = j["z"].get<double>();
        }
        if (!isUnsetBrushTcp(fromJson))
        {
            tcp = fromJson;
        }
    }
    catch (...)
    {
    }
    return tcp;
}

std::string makeTcpValueString(double x, double y, double z, double rx, double ry, double rz)
{
    return "{" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "," +
           std::to_string(rx) + "," + std::to_string(ry) + "," + std::to_string(rz) + "}";
}
