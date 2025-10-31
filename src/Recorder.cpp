// Recorder.cpp
#include "Recorder.h"

bool Recorder::open(const std::string& path, double fps, cv::Size size) {
    int fourcc = cv::VideoWriter::fourcc('M','P','4','V');
    if (writer_.open(path, fourcc, fps, size)) return true;
    fourcc = cv::VideoWriter::fourcc('X','V','I','D');
    return writer_.open(path, fourcc, fps, size);
}
void Recorder::write(const cv::Mat& bgr) {
    if (writer_.isOpened()) writer_.write(bgr);
}
void Recorder::close() {
    if (writer_.isOpened()) writer_.release();
}
