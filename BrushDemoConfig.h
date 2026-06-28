#pragma once

#include <iostream>
#include "nlohmann/json.hpp"

struct BrushDemoConfig
{
    bool demoForceTrajectory = false;
    bool demoFloatBrush = false;
};

inline void loadBrushDemoConfig(const nlohmann::json &j, BrushDemoConfig &cfg)
{
    cfg.demoForceTrajectory = j.value("demoForceTrajectory", false);
    cfg.demoFloatBrush = j.value("demoFloatBrush", false);
    std::cout << "演示配置 demoForceTrajectory=" << (cfg.demoForceTrajectory ? "是" : "否")
              << ", demoFloatBrush=" << (cfg.demoFloatBrush ? "是" : "否") << std::endl;
}
