// Detector.h
#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// ���� ��� ����ü
struct DetBox {
    cv::Rect box;
    int classId;
    float score;
};

class Detector {
public:
    // �� CHANGED: ���� �̸�/�⺻�� ����, �ϰ��� �������̽�
    bool loadOnnx(const std::string& onnxPath, int inputSize = 640, float confTh = 0.25f, float iouTh = 0.45f);

    bool isReady() const { return ready_; }                 // �� FIX: ����� ready_ �� ����
    std::vector<DetBox> infer(const cv::Mat& bgr);

    // �׸���: �� ����/�� ���� �� ���� �����ε� ����
    void draw(cv::Mat& bgr, const std::vector<DetBox>& dets);
    void draw(cv::Mat& bgr, const std::vector<DetBox>& dets, const std::vector<std::string>& names); // �� CHANGED: strings �� string

    // �� CHANGED: ��Ÿ ���� (inpustSize �� inputSize), ������ �ϰ�ȭ
    int   inputSize() const { return inp_; }
    float confTh()    const { return confTh_; }
    float iouTh()     const { return iouTh_; }

private:
    cv::dnn::Net net_;
    bool  ready_ = false;                // �� FIX: ����� ����(ready_)

    int   inp_ = 640;
    float confTh_ = 0.25f;
    float iouTh_ = 0.45f;

    // ��� �ټ� ����� (1,N,85)���� (1,85,N)���� �Ǵ�
    bool isLayout_Nx85(const cv::Mat& out) const;           // �� CHANGED: islayout_* �� isLayout_*

    // (cx,cy,w,h) �� (x0,y0,x1,y1), ���� ũ�� ������ ����
    static inline void xywh2xyxy(
        float cx, float cy, float w, float h,
        float scaleX, float scaleY,
        float& x0, float& y0, float& x1, float& y1
    ) {
        // �� FIX: �ܼ� ��Ÿ/��� ���� ���ɼ� ����
        const float x = cx - w * 0.5f;
        const float y = cy - h * 0.5f;
        x0 = x * scaleX;           y0 = y * scaleY;
        x1 = (x + w) * scaleX;     y1 = (y + h) * scaleY;
    }
};
