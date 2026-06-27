
#include "api/Dashboard.h"
#include "api/Feedback.h"
#include "api/ErrorInfoBean.h"
#include "api/ErrorInfoHelper.h"
#include <iostream>
#include <algorithm>
#include <regex>
#include <mutex>
#include <vector>

class DobotTcpDemo
{
public:
    DobotTcpDemo();
    ~DobotTcpDemo();
    void moveRobot();
    // 使用笛卡尔坐标的往返运动
    void moveRobotC(const Dobot::CDescartesPoint& pointa, const Dobot::CDescartesPoint& pointb);
    // MovS 轨迹拟合示例（使用关节点位）
    // 参数：jointPoints - 关节点位列表（至少 4 个点，至多 50 个点）
    //      params - MovS 可选参数结构体
    void movsDemo(const std::vector<Dobot::CJointPoint>& jointPoints, const Dobot::MovSParams& params);
    // MovS 轨迹拟合示例（使用笛卡尔点位）
    // 参数：descartesPoints - 笛卡尔点位列表（至少 4 个点，至多 50 个点）
    //      params - MovS 可选参数结构体
    void movsDemoC(const std::vector<Dobot::CDescartesPoint>& descartesPoints, const Dobot::MovSParams& params);

    void RelMovJDemo( Dobot::CDescartesPoint& pt,int user=0,int tool=0,int a=10,int v=10,int cp=100);

    bool getCurrentPose(int user, int tool,
                        double& x, double& y, double& z,
                        double& rx, double& ry, double& rz);

    void setToolDemo(int index, std::string value);
private:
    void getFeedBackInfo();
    void moveArriveFinish(const Dobot::CJointPoint& pt, int currentCommandID);
    void moveArriveFinish(const Dobot::CDescartesPoint& pt, int currentCommandID);
    std::vector<std::string> regexRecv(std::string getRecvInfo);
    void clearRobotError();
    void getCurrentCommandID(std::string recvData, int& currentCommandID);

private:
    Dobot::CDashboard m_Dashboard;
    Dobot::CFeedback m_CFeedback;
    Dobot::CFeedbackData feedbackData;
    Dobot::CErrorInfoBeans m_ErrorInfoBeans;
    Dobot::CErrorInfoHelper m_CErrorInfoHelper;
    bool isStateFinish{ false };
    std::thread threadGetFeedBackInfo;
    std::thread threadClearRobotError;
    std::mutex m_mutexValue;
    std::mutex m_mutexState;
    bool finishState{ false };
};
