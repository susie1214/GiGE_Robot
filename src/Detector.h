// Detector.h
#pragma once
#include <opencv2/opencv.hpp>
#include <string>
struct DetBox { cv::Rect box; int classId; float score; };

class Detector {
public:
    bool loadOnnx(const std::string& onnxPath, int inputSize=640, float confTh =0.25f, float iouTh =0.45f);
    bool isReady() const { return ready_; }
    std::vector<DetBox> infer(const cv::Mat& bgr);
    void draw(cv::Mat& bgr, const std::vector<DetBox>& dets,const std::vector<std::strings>& names);

    int inpustSize() const {return inp_;}
    float confTh() const {return confTh_;}
    float iouTh() const {return iouTh_;}
private:
    cv::dnn::Net net_;
    bool ready =false;
    int inp_ =  640;
    float confTh_ =0.25f;
    float iouTh_ =0.45f;
    bool islayout_Nx85(const cv::Mat& out) const;
    static inline void xywh2xyxy(float cx, float cy, float w, float h, float scaleX, float scaleY, float& x0, float& y0, float& x1, float& y1) {
        const float x =cx -w*0.5f;
        const float y =cy -h*0.5f;
        x0 = x*scaleX; y0 = y * scaleY;
        x1 = (x+w) * scaleX; y1 = (y+h) * scaleY;

    }
};
