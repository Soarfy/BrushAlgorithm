#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

template <typename PointT>
bool loadForceTrajectoryFile(const std::string &path, std::vector<PointT> &out)
{
    std::ifstream in(path);
    if (!in.is_open())
        return false;

    out.clear();
    std::string line;
    int lineNo = 0;
    size_t skipped = 0;
    while (std::getline(in, line))
    {
        ++lineNo;
        const auto start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos)
            continue;
        if (line[start] == '#')
            continue;

        std::istringstream iss(line.substr(start));
        double x, y, z, a, b, c;
        if (iss >> x >> y >> z >> a >> b >> c)
        {
            out.push_back(PointT{x, y, z, a, b, c});
        }
        else
        {
            ++skipped;
            std::cerr << "跳过无效轨迹行 [" << lineNo << "]: " << line << std::endl;
        }
    }

    if (skipped > 0)
    {
        std::cerr << "共跳过 " << skipped << " 行无效轨迹数据" << std::endl;
    }
    return !out.empty();
}

inline bool backupForceTrajectoryFile(const std::string &path)
{
    std::error_code ec;
    if (!std::filesystem::exists(path, ec))
        return false;
    std::filesystem::copy_file(
        path, path + ".bak", std::filesystem::copy_options::overwrite_existing, ec);
    if (ec)
    {
        std::cerr << "无法备份轨迹到 " << path << ".bak: " << ec.message() << std::endl;
        return false;
    }
    return true;
}

inline std::string forceTrajectoryTempPathFor(const std::string &path)
{
    return path + ".tmp";
}

inline bool commitForceTrajectoryFile(const std::string &tempPath, const std::string &finalPath)
{
    std::error_code ec;
    std::filesystem::rename(tempPath, finalPath, ec);
    if (!ec)
        return true;

    std::filesystem::copy_file(
        tempPath, finalPath, std::filesystem::copy_options::overwrite_existing, ec);
    if (ec)
    {
        std::cerr << "无法写回轨迹文件 " << finalPath << ": " << ec.message() << std::endl;
        return false;
    }
    std::filesystem::remove(tempPath, ec);
    return true;
}
