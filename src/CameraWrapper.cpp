// CameraWrapper.cpp
#include "CameraWrapper.h"
#include <stdexcept>
#include <sstream>

void CameraWrapper::check(const char* where, int err) {
    if (err == MV_OK) return;
    std::ostringstream oss; oss << where << " failed: 0x" << std::hex << err;
    throw std::runtime_error(oss.str());
}

CameraWrapper::CameraWrapper() {}
CameraWrapper::~CameraWrapper() { try { close(); } catch(...) {} }

void CameraWrapper::openFirstGigE() {
    if (opened_) return;
    MV_CC_DEVICE_INFO_LIST devList{};
    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE, &devList);
    check("MV_CC_EnumDevices", ret);
    if (devList.nDeviceNum <= 0) throw std::runtime_error("No GigE device found");

    MV_CC_DEVICE_INFO* pDevInfo = devList.pDeviceInfo[0];
    if (!pDevInfo) throw std::runtime_error("Invalid device info");

    ret = MV_CC_CreateHandle(&handle_, pDevInfo);
    check("MV_CC_CreateHandle", ret);

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        MV_CC_DestroyHandle(handle_); handle_ = nullptr;
        check("MV_CC_OpenDevice", ret);
    }
    opened_ = true;

    // Optional: Packet size tuning
    MVCC_INTVALUE pkt{};
    ret = MV_CC_GetIntValue(handle_, "GevSCPSPacketSize", &pkt);
    if (ret == MV_OK && pkt.nCurValue < 1500) {
        MV_CC_SetIntValue(handle_, "GevSCPSPacketSize", 1500);
    }
}

void CameraWrapper::close() {
    if (!opened_) return;
    if (grabbing_) {
        MV_CC_StopGrabbing(handle_);
        grabbing_ = false;
    }
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    opened_ = false;
}

void CameraWrapper::startGrabbing() {
    if (!opened_) throw std::runtime_error("camera not open");
    if (grabbing_) return;

    int ret = MV_CC_RegisterImageCallBackEx(handle_, &CameraWrapper::ImageCallback, this);
    check("MV_CC_RegisterImageCallBackEx", ret);

    ret = MV_CC_StartGrabbing(handle_);
    check("MV_CC_StartGrabbing", ret);
    grabbing_ = true;
}

void CameraWrapper::stopGrabbing() {
    if (!opened_ || !grabbing_) return;
    int ret = MV_CC_StopGrabbing(handle_);
    check("MV_CC_StopGrabbing", ret);
    grabbing_ = false;
}

void CameraWrapper::setFloat(const char* key, float v) {
    if (!opened_) return;
    int ret = MV_CC_SetFloatValue(handle_, key, v);
    check("MV_CC_SetFloatValue", ret);
}
void CameraWrapper::setInt(const char* key, int v) {
    if (!opened_) return;
    int ret = MV_CC_SetIntValue(handle_, key, v);
    check("MV_CC_SetIntValue", ret);
}
void CameraWrapper::setEnum(const char* key, int v) {
    if (!opened_) return;
    int ret = MV_CC_SetEnumValue(handle_, key, v);
    check("MV_CC_SetEnumValue", ret);
}

void CameraWrapper::ImageCallback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pInfo, void* pUser) {
    if (!pUser || !pData || !pInfo) return;
    reinterpret_cast<CameraWrapper*>(pUser)->onImage(pData, pInfo);
}

void CameraWrapper::onImage(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pInfo) {
    if (!userCb_) return;
    const int w = static_cast<int>(pInfo->nWidth);
    const int h = static_cast<int>(pInfo->nHeight);
    const unsigned int pt = pInfo->enPixelType;

    cv::Mat bgr;
    switch (pt) {
        case PixelType_Gvsp_Mono8: {
            cv::Mat gray(h, w, CV_8UC1, pData);
            cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
            break;
        }
        case PixelType_Gvsp_BGR8_Packed: {
            cv::Mat tmp(h, w, CV_8UC3, pData);
            bgr = tmp.clone();
            break;
        }
        case PixelType_Gvsp_BayerRG8: {
            cv::Mat bayer(h, w, CV_8UC1, pData);
            cv::cvtColor(bayer, bgr, cv::COLOR_BayerRG2BGR);
            break;
        }
        case PixelType_Gvsp_BayerGB8: {
            cv::Mat bayer(h, w, CV_8UC1, pData);
            cv::cvtColor(bayer, bgr, cv::COLOR_BayerGB2BGR);
            break;
        }
        case PixelType_Gvsp_BayerBG8: {
            cv::Mat bayer(h, w, CV_8UC1, pData);
            cv::cvtColor(bayer, bgr, cv::COLOR_BayerBG2BGR);
            break;
        }
        case PixelType_Gvsp_BayerGR8: {
            cv::Mat bayer(h, w, CV_8UC1, pData);
            cv::cvtColor(bayer, bgr, cv::COLOR_BayerGR2BGR);
            break;
        }
        default:
            return;
    }
    userCb_(bgr);
}
