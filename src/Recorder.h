// Recorder.h
#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class Recorder {
public:
    bool open(const std::string& path, double fps, cv::Size size);
    void write(const cv::Mat& bgr);
    void close();
private:
    cv::VideoWriter writer_;
};
