// CameraWrapper.h
#pragma once
#include <functional>
#include <opencv2/opencv.hpp>
#include <MvCameraControl.h>

class CameraWrapper {
public:
    using FrameCB = std::function<void(const cv::Mat& bgr)>;

    CameraWrapper();
    ~CameraWrapper();

    void openFirstGigE();
    void close();
    void startGrabbing();
    void stopGrabbing();
    bool isOpen() const { return opened_; }

    void setFloat(const char* key, float v);
    void setInt  (const char* key, int   v);
    void setEnum (const char* key, int   v);

    void setFrameCallback(FrameCB cb) { userCb_ = std::move(cb); }

private:
    void* handle_ = nullptr;
    bool  opened_ = false;
    bool  grabbing_ = false;
    FrameCB userCb_;

    static void ImageCallback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pInfo, void* pUser);
    void onImage(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pInfo);
    static void check(const char* where, int err);
};
