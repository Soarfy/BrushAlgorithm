// example.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "VideoCapture.hpp"
#include "xcamera.h"
#include "enumerate.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <string>

using namespace CAMERA;

class CameraCapture::Impl
{
public:
    Impl(const std::string &ip) : p_camera(nullptr), width(0), height(0), channels(1)
    {
        p_camera = (XCamera *)createXCamera();
        if (!p_camera)
            throw std::runtime_error("Failed to create camera instance");
        int ret = p_camera->connect(ip.c_str());
        if (ret != 0)
        {
            destroyXCamera(p_camera);
            throw std::runtime_error("Failed to connect camera");
        }
        p_camera->getCameraResolution(&width, &height);
        p_camera->getCameraChannels(&channels);
    }
    ~Impl()
    {
        if (p_camera)
        {
            p_camera->disconnect("169.254.7.168");
            destroyXCamera(p_camera);
        }
    }
    bool captureAndSave(int id)
    {
        std::vector<unsigned char> brightness_data(width * height * 3, 0);
        std::vector<float> point_cloud_data(width * height * 3, 0);
        // 深度图
        float *depth_data = (float *)malloc(sizeof(float) * width * height);
        memset(depth_data, 0, sizeof(float) * width * height);

        int ret = 0, num = 0;
        char status_json[20480] = {0}, config_json[20480] = {0}, timestamp_data[30] = {0};
        if (id < 11)
        {
            p_camera->readJson(config_json, "D:/UsmileProject/KW-P/KW-DCW-SDK-20250519/SDK_windows_2.2/camera_config.json");
        }
        else
        {
            p_camera->readJson(config_json, "D:/UsmileProject/KW-P/KW-DCW-SDK-20250519/SDK_windows_2.2/example_confignew.json");
            // p_camera->readJson(config_json, "D:/UsmileProject/KW-P/KW-DCW-SDK-20250519/SDK_windows_2.2/example_configcapture.json");
        }

        p_camera->setParamJson(config_json, status_json, num);
        ret = p_camera->captureData(num, timestamp_data);
        if (ret != 0)
            return false;
        if (channels == 1)
            ret = p_camera->getBrightnessData(brightness_data.data());
        else if (channels == 3)
            ret = p_camera->getColorBrightnessData(brightness_data.data(), Color::Rgb);
        else
            return false;
        if (ret != 0)
            return false;

        std::string bmp_name = "D:\\planR\\img\\" + std::to_string(id) + ".bmp";
        if (id < 12)
        {
            bmp_name = "D:\\planR\\img\\" + std::to_string(id) + ".bmp";
        }
        else
        {
            bmp_name = "D:\\planR\\img\\old6\\" + std::to_string(id) + ".bmp";
        }
        cv::Mat bright = (channels == 1)
                             ? cv::Mat(height, width, CV_8U, brightness_data.data())
                             : cv::Mat(height, width, CV_8UC3, brightness_data.data());
        cv::imwrite(bmp_name, bright);
        ret = p_camera->getPointcloudData(point_cloud_data.data());
        if (ret != 0)
            return false;
        if (id < 12)
        {
            std::string ply_name = "D:\\planR\\ply\\" + std::to_string(id) + ".ply";
            // 保存之前提前旋转转换
            p_camera->savePointcloudToPly(point_cloud_data.data(), brightness_data.data(), channels, ply_name.c_str());
            std::cout << "Saved: " << bmp_name << ", " << ply_name << std::endl;
        }else{
            std::string ply_name = "D:\\planR\\img\\old6ply\\" + std::to_string(id) + ".ply";
            p_camera->savePointcloudToPly(point_cloud_data.data(), brightness_data.data(), channels, ply_name.c_str());
            std::cout << "Saved: " << bmp_name << ", " << ply_name << std::endl;
        }

        // 保存
        // 获取深度图数据并保存
        ret = p_camera->getDepthData(depth_data);

        if (ret != 0)
            return false;

        cv::Mat depth = cv::Mat(height, width, CV_32F, depth_data);

        std::string diff_name = "D:/planR/piff/" + std::to_string(id) + ".tiff";
        if (id < 12)
        {
            diff_name = "D:/planR/piff/" + std::to_string(id) + ".tiff";
        }
        else
        {
            diff_name = "D:/planR/piff/old6/" + std::to_string(id) + ".tiff";
        }

        // 使用默认参数保存深度图像
        bool save_success = cv::imwrite(diff_name, depth);
        if (!save_success)
        {
            std::cerr << "Error: Failed to save depth image to " << diff_name << std::endl;
            free(depth_data);
            return false;
        }

        free(depth_data);
        return true;
    }


private:
    XCamera *p_camera;
    int width, height, channels;
};

CameraCapture::CameraCapture(const std::string &ip) : impl_(new Impl(ip)) {}
CameraCapture::~CameraCapture() { delete impl_; }
bool CameraCapture::captureAndSave(int id) { return impl_->captureAndSave(id); }
