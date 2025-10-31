// FrameProcessor.cpp
#include "FrameProcessor.h"

cv::Mat FrameProcessor::run(const cv::Mat& src) {
    if (src.empty()) return src;
    cv::Mat dst = src;

    // 대비/밝기
    if (alpha_ != 1.0 || beta_ != 0.0) {
        cv::Mat tmp; src.convertTo(tmp, CV_8UC3, alpha_, beta_);
        dst = tmp;
    }

    // 플립(좌우/상하/동시)
    if (flipH_ || flipV_) {
        int code = -2;
        if (flipH_ && flipV_) code = -1;     // both
        else if (flipH_)       code = 1;     // horizontal
        else if (flipV_)       code = 0;     // vertical
        if (code != -2) {
            cv::Mat t; cv::flip(dst, t, code); dst = t;
        }
    }

    // 그레이
    if (useGray_) {
        cv::Mat g, tmp;
        cv::cvtColor(dst, g, cv::COLOR_BGR2GRAY);
        cv::cvtColor(g, tmp, cv::COLOR_GRAY2BGR);
        dst = tmp;
    }

    // HSV(red mask)
    if (useHSV_) {
        cv::Mat hsv, m1, m2, mask, tmp;
        cv::cvtColor(dst, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 100, 50), cv::Scalar(10, 255, 255), m1);
        cv::inRange(hsv, cv::Scalar(160, 100, 50), cv::Scalar(179, 255, 255), m2);
        cv::bitwise_or(m1, m2, mask);
        cv::bitwise_and(dst, dst, tmp, mask);
        dst = tmp;
    }

    // Blur / Gaussian (가변 커널/시그마)
    const int k = (kernel_ % 2 == 0 ? kernel_ + 1 : kernel_);
    if (useBlur_) { cv::Mat t; cv::blur(dst, t, cv::Size(k, k)); dst = t; }
    if (useGauss_) { cv::Mat t; cv::GaussianBlur(dst, t, cv::Size(k, k), std::max(0.1, sigmaX_), 0); dst = t; }

    // Zoom (center crop 후 확대)
    if (zoom_ > 1) {
        int w = dst.cols, h = dst.rows;
        int rw = w / zoom_, rh = h / zoom_;
        int x = (w - rw) / 2, y = (h - rh) / 2;
        cv::Rect roi(x, y, rw, rh);
        cv::Mat crop = dst(roi).clone(), z;
        cv::resize(crop, z, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);
        dst = z;
    }
    return dst;
}
