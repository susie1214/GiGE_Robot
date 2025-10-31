// OcrEngine.h
#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class OcrEngine {
public:
    std::string run(const cv::Mat& bgr, const std::string& tessdataDir, const std::string& lang);
};
