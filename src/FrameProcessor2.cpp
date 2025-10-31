// FrameProcessor.cpp
#include "FrameProcessor.h"

cv::Mat FrameProcessor::run(const cv::Mat& src) {
    if (src.empty()) return src;
    cv::Mat dst = src;

    if (alpha_!=1.0 || beta_!=0.0) {
        cv::Mat tmp; src.convertTo(tmp, CV_8UC3, alpha_, beta_);
        dst = tmp;
    }
    if (useGray_) {
        cv::Mat g, tmp;
        cv::cvtColor(dst, g, cv::COLOR_BGR2GRAY);
        cv::cvtColor(g, tmp, cv::COLOR_GRAY2BGR);
        dst = tmp;
    }
    if (useHSV_) {
        cv::Mat hsv, m1, m2, mask, tmp;
        cv::cvtColor(dst, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0,100,50),   cv::Scalar(10,255,255),  m1);
        cv::inRange(hsv, cv::Scalar(160,100,50), cv::Scalar(179,255,255), m2);
        cv::bitwise_or(m1, m2, mask);
        cv::bitwise_and(dst, dst, tmp, mask);
        dst = tmp;
    }
    if (useBlur_)  { cv::Mat t; cv::blur(dst, t, cv::Size(5,5)); dst = t; }
    if (useGauss_) { cv::Mat t; cv::GaussianBlur(dst, t, cv::Size(5,5), 0); dst = t; }

    if (zoom_ > 1) {
        int w = dst.cols, h = dst.rows;
        int rw = w / zoom_, rh = h / zoom_;
        int x = (w - rw) / 2, y = (h - rh) / 2;
        cv::Rect roi(x,y,rw,rh);
        cv::Mat crop = dst(roi).clone(), z;
        cv::resize(crop, z, cv::Size(w,h), 0,0, cv::INTER_LINEAR);
        dst = z;
    }
    return dst;
}
