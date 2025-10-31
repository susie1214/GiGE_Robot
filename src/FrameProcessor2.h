// FrameProcessor.h
#pragma once
#include <opencv2/opencv.hpp>

class FrameProcessor {
public:
    void setAlpha(double a){ alpha_=a; }
    void setBeta(double b){ beta_=b; }
    void setZoom(int z){ zoom_ = (z<1?1:z); }
    void setGray(bool on){ useGray_=on; }
    void setHSV(bool on){ useHSV_=on; }
    void setBlur(bool on){ useBlur_=on; }
    void setGauss(bool on){ useGauss_=on; }

    cv::Mat run(const cv::Mat& src);

private:
    double alpha_{1.0};
    double beta_{0.0};
    int zoom_{1};
    bool useGray_{false}, useHSV_{false}, useBlur_{false}, useGauss_{false};
};
