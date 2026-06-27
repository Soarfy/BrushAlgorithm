
#include "DobotTcpDemo.h"
#include <fstream>
#include <vector>

DobotTcpDemo::DobotTcpDemo()
{
    std::string robotIp = "192.168.5.1";
    unsigned int controlPort = 29999;
    unsigned int feekPort = 30004;

    std::cout << "开始连接" << std::endl;
    m_Dashboard.Connect(robotIp, controlPort);
    m_CFeedback.Connect(robotIp, feekPort);

    //  while (!m_Dashboard.Connect(robotIp, controlPort)&&!m_CFeedback.Connect(robotIp, feekPort))
    // {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // }
    // m_Dashboard.EnableRobot();
    std::cout << "连接成功" << std::endl;
    m_CErrorInfoHelper.ParseControllerJsonFile("../alarmController.json");
    m_CErrorInfoHelper.ParseServoJsonFile("../alarmServo.json");
    threadGetFeedBackInfo = std::thread(&DobotTcpDemo::getFeedBackInfo, this);
    threadGetFeedBackInfo.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    threadClearRobotError = std::thread(&DobotTcpDemo::clearRobotError, this);
    threadClearRobotError.detach();
}

DobotTcpDemo::~DobotTcpDemo()
{
    m_Dashboard.Disconnect();
    m_CFeedback.Disconnect();
}

void DobotTcpDemo::RelMovJDemo( Dobot::CDescartesPoint& pt,int user,int tool,int a,int v,int cp){
    std::string result = m_Dashboard.RelMovJTool(pt, user, tool, a, v, cp);
    // std::cout << "RelMovJTool result: " << result << std::endl;
}

bool DobotTcpDemo::getCurrentPose(int user, int tool,
                                 double& x, double& y, double& z,
                                 double& rx, double& ry, double& rz)
{
    std::string recv = m_Dashboard.GetPose(user, tool);

    //std::string recv = m_Dashboard.GetPose( );

    // 期望格式：
    // 0,{X,Y,Z,Rx,Ry,Rz},GetPose(user,tool)
    size_t l = recv.find('{');
    size_t r = recv.find('}');
    if (l == std::string::npos || r == std::string::npos)
        return false;
    // std::cout << "get pose" <<std::endl;

    std::string data = recv.substr(l + 1, r - l - 1);
    // std::cout << "data" << recv << std::endl;

    std::stringstream ss(data);
    std::vector<double> vals;
    std::string token;

    while (std::getline(ss, token, ',')) {
        vals.push_back(std::stod(token));
    }

    if (vals.size() != 6)
        return false;

    x  = vals[0];
    y  = vals[1];
    z  = vals[2];
    rx = vals[3];
    ry = vals[4];
    rz = vals[5];

    return true;
}

void DobotTcpDemo::setToolDemo(int index, std::string value){
    std::cout << "set tools " << std::endl;
    m_Dashboard.SetTool(index,value);
}

void DobotTcpDemo::moveRobot()
{
    double pointa[] = { -90, 20, 0, 0, 0, 0 };
    double pointb[] = { 90, 20, 0, 0, 0, 0 };
    m_Dashboard.EnableRobot();
    Dobot::CJointPoint ptPointa;
    Dobot::CJointPoint ptPointb;
    memcpy(&ptPointa, pointa, sizeof(ptPointa));
    memcpy(&ptPointb, pointb, sizeof(ptPointb));
    int currentCommandID = 0;
    while (true) {
        getCurrentCommandID(m_Dashboard.MovJ(ptPointa), currentCommandID);
        moveArriveFinish(ptPointa, currentCommandID);
        getCurrentCommandID(m_Dashboard.MovJ(ptPointb), currentCommandID);
        moveArriveFinish(ptPointb, currentCommandID);
    }
}

// 使用 MovS 的简单示例
// 注意：调用前需要确保机械臂已经被人工移动到轨迹起始点
// 参数：jointPoints - 关节点位列表（至少 4 个点，至多 50 个点）
//      params - MovS 可选参数结构体
void DobotTcpDemo::movsDemo(const std::vector<Dobot::CJointPoint>& jointPoints, const Dobot::MovSParams& params)
{
    // 先使能机器人
    m_Dashboard.EnableRobot();

    // 调用 MovS 指令
    std::string result = m_Dashboard.MovS(jointPoints, params);
    std::cout << "MovS result: " << result << std::endl;
}

// 使用笛卡尔坐标的往返运动
// void DobotTcpDemo::moveRobotC(const Dobot::CDescartesPoint& pointa, const Dobot::CDescartesPoint& pointb)
// {
//    m_Dashboard.EnableRobot();
//    int currentCommandID = 0;

//    getCurrentCommandID(m_Dashboard.MovJ(pointa), currentCommandID);
//    while (true) {
//        getCurrentCommandID(m_Dashboard.MovJ(pointa), currentCommandID);
//        moveArriveFinish(pointa, currentCommandID);
//        getCurrentCommandID(m_Dashboard.MovJ(pointb), currentCommandID);
//        moveArriveFinish(pointb, currentCommandID);
//    }
// }

// void DobotTcpDemo::moveRobotC(const Dobot::CDescartesPoint& pointa,
//     const Dobot::CDescartesPoint& pointb)
// {
//     m_Dashboard.EnableRobot();
//     int currentCommandID = 0;
//     getCurrentCommandID(m_Dashboard.MovJ(pointa), currentCommandID);
//     // moveArriveFinish(pointa, currentCommandID);
//     // std::cout << "Arrived at pointA, exit moveRobotC" << std::endl;
// }

void DobotTcpDemo::moveRobotC(const Dobot::CDescartesPoint& pointa,
    const Dobot::CDescartesPoint& pointb)
{
    m_Dashboard.EnableRobot();
    m_Dashboard.MovJ(pointa);
    // m_Dashboard.DisableRobot();
    // int currentCommandID = 0;
    
    // // 持續發送直到獲得有效的 commandID
    // do {
    //     getCurrentCommandID(m_Dashboard.MovJ(pointa), currentCommandID);
        
    //     if (currentCommandID == 0) {
    //         std::cout << "Command ID is 0, retrying..." << std::endl;
    //         // 可以加入短暫延遲，避免過度頻繁發送
    //         // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // } while (currentCommandID == 0);
    
    // std::cout << "Got valid command ID: " << currentCommandID << std::endl;
    // moveArriveFinish(pointa, currentCommandID);
    // std::cout << "Arrived at pointA, exit moveRobotC" << std::endl;
}

// 使用 MovS 的简单示例（笛卡尔坐标版本）
// 注意：调用前需要确保机械臂已经被人工移动到轨迹起始点
// 参数：descartesPoints - 笛卡尔点位列表（至少 4 个点，至多 50 个点）
//      params - MovS 可选参数结构体
void DobotTcpDemo::movsDemoC(const std::vector<Dobot::CDescartesPoint>& descartesPoints, const Dobot::MovSParams& params)
{
    // 先使能机器人
    m_Dashboard.EnableRobot();

    // 调用 MovS 指令
    std::string result = m_Dashboard.MovS(descartesPoints, params);
    // std::cout << "MovS result: " << result << std::endl;

    // m_Dashboard.DisableRobot();
}

void DobotTcpDemo::moveArriveFinish(const Dobot::CDescartesPoint& pt, int currentCommandID)
{
    std::cout << "Wait moveArriveFinish" << std::endl;
    std::cout << "currentCommandID 1" << currentCommandID  << std::endl;
    while (true) {
        {
            std::unique_lock<std::mutex> lockValue(m_mutexValue);
            std::cout << "currentCommandID 2" << feedbackData.CurrentCommandId << std::endl;
            if (feedbackData.CurrentCommandId > currentCommandID) {
                break;
            }
            if (feedbackData.CurrentCommandId == currentCommandID && feedbackData.RobotMode == 5) {
                break;
            }
        }

        {
            std::unique_lock<std::mutex> lockValue(m_mutexState);
            if (finishState) {
                finishState = false;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DobotTcpDemo::getCurrentCommandID(std::string recvData, int& currentCommandID)
{
    std::cout << "recvData " << recvData << std::endl;
    currentCommandID = 2147483647;    // 初始值  int-max
    if (recvData.find("device does not connected") != std::string::npos) {
        std::cout << "device does not connected " << std::endl;
        return;
    }

    if (recvData.find("send error") != std::string::npos) {
        std::cout << "send error" << std::endl;
        return;
    }

    // recvData 为 0,{2},MovJ(joint={-90, 20, 0, 0, 0, 0})     vecRecv为所有数字的集合 [ 0,2,-90, 20, 0, 0, 0, 0]
    std::vector<std::string> vecRecv = regexRecv(recvData);

    // vecRecv[0]为指令是否下发成功   vecRecv[1]为返回运动指令currentCommandID
    if (vecRecv.size() >= 2U && std::stoi(vecRecv[0]) == 0) {
        currentCommandID = std::stoi(vecRecv[1]);
    }
}

void DobotTcpDemo::getFeedBackInfo()
{
    std::cout << "Start GetFeedBackInfo" << std::endl;
    while (true) {
        {
            std::unique_lock<std::mutex> lockValue(m_mutexValue);
            feedbackData = m_CFeedback.GetFeedbackData();
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void DobotTcpDemo::moveArriveFinish(const Dobot::CJointPoint& pt, int currentCommandID)
{
    std::cout << "Wait moveArriveFinish" << std::endl;
    while (true) {
        {
            std::unique_lock<std::mutex> lockValue(m_mutexValue);
            if (feedbackData.CurrentCommandId > currentCommandID) {
                break;
            }
            if (feedbackData.CurrentCommandId == currentCommandID && feedbackData.RobotMode == 5) {
                break;
            }
        }

        {
            std::unique_lock<std::mutex> lockValue(m_mutexState);
            if (finishState) {
                finishState = false;
                break;
            }
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
};

std::vector<std::string> DobotTcpDemo::regexRecv(std::string getRecvInfo)
{
    std::regex pattern("-?\\d+");
    std::smatch matches;
    std::string::const_iterator searchStart(getRecvInfo.cbegin());
    std::vector<std::string> vecErrorId;
    while (std::regex_search(searchStart, getRecvInfo.cend(), matches, pattern)) {
        for (auto& match : matches) {
            vecErrorId.push_back(match.str());
        }
        searchStart = matches.suffix().first;
    }
    return vecErrorId;
};

void DobotTcpDemo::clearRobotError()
{
    std::cout << "Start CheckRobotError" << std::endl;
    while (true) {
        {
            std::unique_lock<std::mutex> lockValue(m_mutexValue);
            if (feedbackData.ErrorStatus) {
                std::vector<std::string> errorIdVec = regexRecv(m_Dashboard.GetErrorID());
                for (int i = 1; i < errorIdVec.size(); i++) {
                    Dobot::CErrorInfoBean beanController;
                    Dobot::CErrorInfoBean beanServo;
                    if (std::stoi(errorIdVec[i]) != 0) {
                        printf("告警码：%s\n", errorIdVec[i].c_str());
                        if (m_CErrorInfoHelper.FindController(std::stoi(errorIdVec[i]), beanController)) {
                            printf("控制器告警：%d, 告警原因：%s,%s\n", beanController.id,
                                   beanController.zh_CN.description.c_str(), beanController.en.description.c_str());
                        } else {
                            if (m_CErrorInfoHelper.FindServo(std::stoi(errorIdVec[i]), beanServo)) {
                                printf("伺服告警：%d,告警原因：%s, %s\n", beanServo.id,
                                       beanServo.zh_CN.description.c_str(), beanServo.en.description.c_str());
                            }
                        }
                    }
                }
                char choose[50] = { "" };
                std::cout << "输入1, 将清除错误, 机器继续运行:" << std::endl;
                std::cin >> choose;
                std::cout << "您的选择： " << choose << std::endl;
                try {
                    int result = std::stoi(choose);
                    if (result == 1) {
                        std::cout << "清除错误，机器继续运行！" << std::endl;
                        m_Dashboard.ClearError();
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Exception caught: " << e.what() << std::endl;
                } catch (...) {
                    std::cerr << "Unknown exception caught." << std::endl;
                }
            } else {
                if (feedbackData.RobotMode == 11) {
                    std::cout << "机器发生碰撞 " << std::endl;
                    char choose[50] = { "" };
                    std::cout << "输入1, 将清除碰撞, 机器继续运行: " << std::endl;
                    std::cin >> choose;
                    std::cout << "您的选择： " << choose << std::endl;
                    try {
                        int result = std::stoi(choose);
                        if (result == 1) {
                            std::cout << "清除错误，机器继续运行！" << std::endl;
                            m_Dashboard.ClearError();
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "Exception caught: " << e.what() << std::endl;
                    } catch (...) {
                        std::cerr << "Unknown exception caught." << std::endl;
                    }
                }

                if (!feedbackData.EnableStatus) {
                    std::cout << "机器未使能 " << std::endl;
                    char choose[50] = { "" };
                    std::cout << "输入1, 机器将使能: " << std::endl;
                    std::cin >> choose;
                    std::cout << "您的选择： " << choose << std::endl;
                    try {
                        int result = std::stoi(choose);
                        if (result == 1) {
                            std::cout << "机器使能！" << std::endl;
                            m_Dashboard.EnableRobot();
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "Exception caught: " << e.what() << std::endl;
                    } catch (...) {
                        std::cerr << "Unknown exception caught." << std::endl;
                    }
                }

                if (!feedbackData.ErrorStatus && feedbackData.EnableStatus && feedbackData.RobotMode == 5) {
                    std::unique_lock<std::mutex> lockValue(m_mutexState);
                    finishState = true;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
}
