#pragma once
#include <string>
#include <vector>

class CameraCapture {
public:
    CameraCapture(const std::string& ip);
    ~CameraCapture();
    bool captureAndSave(int id);
    // bool captureAndSavedepth(int id);
private:
    class Impl;
    Impl* impl_;
};


