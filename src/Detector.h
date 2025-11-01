// Detector.h
#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// 검출 결과 구조체
struct DetBox {
    cv::Rect box;
    int classId;
    float score;
};

class Detector {
public:
    // ★ CHANGED: 인자 이름/기본값 정리, 일관된 인터페이스
    bool loadOnnx(const std::string& onnxPath, int inputSize = 640, float confTh = 0.25f, float iouTh = 0.45f);

    bool isReady() const { return ready_; }                 // ★ FIX: 멤버명 ready_ 로 통일
    std::vector<DetBox> infer(const cv::Mat& bgr);

    // 그리기: 라벨 없이/라벨 포함 두 가지 오버로드 제공
    void draw(cv::Mat& bgr, const std::vector<DetBox>& dets);
    void draw(cv::Mat& bgr, const std::vector<DetBox>& dets, const std::vector<std::string>& names); // ★ CHANGED: strings → string

    // ★ CHANGED: 오타 수정 (inpustSize → inputSize), 접근자 일관화
    int   inputSize() const { return inp_; }
    float confTh()    const { return confTh_; }
    float iouTh()     const { return iouTh_; }

private:
    cv::dnn::Net net_;
    bool  ready_ = false;                // ★ FIX: 멤버명 통일(ready_)

    int   inp_ = 640;
    float confTh_ = 0.25f;
    float iouTh_ = 0.45f;

    // 출력 텐서 모양이 (1,N,85)인지 (1,85,N)인지 판단
    bool isLayout_Nx85(const cv::Mat& out) const;           // ★ CHANGED: islayout_* → isLayout_*

    // (cx,cy,w,h) → (x0,y0,x1,y1), 원본 크기 스케일 적용
    static inline void xywh2xyxy(
        float cx, float cy, float w, float h,
        float scaleX, float scaleY,
        float& x0, float& y0, float& x1, float& y1
    ) {
        // ★ FIX: 단순 오타/산술 오류 가능성 제거
        const float x = cx - w * 0.5f;
        const float y = cy - h * 0.5f;
        x0 = x * scaleX;           y0 = y * scaleY;
        x1 = (x + w) * scaleX;     y1 = (y + h) * scaleY;
    }
};
