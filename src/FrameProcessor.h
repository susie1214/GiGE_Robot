// FrameProcessor.h
#pragma once
#include <opencv2/opencv.hpp>

class FrameProcessor {
public:
    cv::Mat run(const cv::Mat& src);

    // 기존 파라미터
    void setAlpha(double a) { alpha_ = a; }
    void setBeta(double b) { beta_ = b; }
    void setZoom(int z) { zoom_ = std::max(1, z); }
    void setGray(bool v) { useGray_ = v; }
    void setHSV(bool v) { useHSV_ = v; }
    void setBlur(bool v) { useBlur_ = v; }
    void setGauss(bool v) { useGauss_ = v; }

    // 추가 파라미터
    void setKernelSize(int k) { kernel_ = (k % 2 == 0 ? k + 1 : k); if (kernel_ < 3) kernel_ = 3; }
    void setSigmaX(double s) { sigmaX_ = (s <= 0.0 ? 0.1 : s); }
    void setFlipH(bool v) { flipH_ = v; }
    void setFlipV(bool v) { flipV_ = v; }

private:
    double alpha_{ 1.0 }, beta_{ 0.0 };
    int zoom_{ 1 };
    bool useGray_{ false }, useHSV_{ false }, useBlur_{ false }, useGauss_{ false };

    // 추가
    int kernel_{ 5 };         // 홀수
    double sigmaX_{ 1.2 };    // >0
    bool flipH_{ false };
    bool flipV_{ false };
};
