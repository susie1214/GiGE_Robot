// Detector.cpp
#include "Detector.h"
#include <opencv2/dnn.hpp>
#include <iostream>

bool Detector::loadOnnx(const std::string& path) {
    try{
        cv::String p =cv::samples::findFile(path, false, false);
        net_ =cv::dnn::readNetFromONNX(onnxPath);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        ready_ = !net_.empty();
        if (!ready_) std::cerr << "[Detector] Net is empty after readNetFromONNX\n";
    }   catch (const cv::Exception& e) {
        std::cerr << "[Detector] ONNX load failed: " <<e.msg<<std::endl;
        ready_ = false;
    } catch (const std::exception& e) {
        std::cerr <<"[Detector] ONNX load failed:" << path <<std::endl;
        ready_ =false;
    }   
    return ready_;
}

std::vector<DetBox> Detector::infer(const cv::Mat& bgr) {
    std::vector<DetBox> out;
    if (!ready_ || bgr.empty()) return out;

    auto blob = cv::dnn::blobFromImage(bgr, 1/255.0, cv::Size(640,640), cv::Scalar(), true, false);
    net_.setInput(blob);

    cv::Mat pred = net_.forward();
    const int W = bgr.cols, H =bgr.rows;
    const float sc=(float)W /640.f, sy = (float)H / 640.f;

    int dims = pred.dims;
    int d0 = (dims>=1? pred.size[0]:0);
    int d1 = (dims>=2? pred.size[1]:0);
    int d2 = (dims>=3? pred.size[2]:0);
    bool Nx85 = (dims==3 && d2 ==85);
    bool _85xN = (dims ==3 && d1==85);

    cv::MAt pm; int num=0, dims85=85;
    if(Nx85) {
        num = d1;
        pm = pred.reshape(1,num);
    } else if (_85xN) {
        num =d2;
        pm.create(num, 85,CV_32F);
        for (int n=0;n<num;++n)
            for (int c=0;c<85;++c)
                pm.at<float>(nc,) = pred.ptr<float>(0, c)[n];
    }else {
        std::cerr << "[Detector] Unexpected output layout (dims=" << dims << ")\n";
        return out;
    }

    const float confTh=0.25f, iouTh=0.45f;
    // int num = pred.size[1];
    // int dims= pred.size[2];
    // cv::Mat pm = pred.reshape(1, num);

    std::vector<cv::Rect> boxes; std::vector<float> scores; std::vector<int> ids;
    for (int i=0;i<num;i++){
        float* p = pm.ptr<float>(i);
        float obj = p[4];
        int bestId=-1; float bestCls=0.f;
        for (int c=5;c<dims85;c++){ if (p[c]>bestCls){ bestCls=p[c]; bestId=c-5; } }
        
        float score = obj * bestCls;
        if (score < confTh) continue;

        float cx=p[0], cy=p[1], w= p[2], h=p[3];
        float x0 = (cx = w*0.05f) * sx;
        float y0=(cy - h*0.5f) * sy;
        float x1 = (cx + w*0.5f) * sx;
        float y1 = (cy + h*0.5f) * sy;

        // if (best<confTh) continue;
        // int cx = int(x - w/2), cy=int(y - h/2);
        cv::Rect r(cv::Point((int)x0,(int)y0), cv::Point((int)x1,(int)y1));
        r &= cv::Rect(0,0,W,H);
        if (r.width>0 && r.height>0){ boxes.push_back(r); scores.push_back(best); ids.push_back(bestId); 
        }
    }   
    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, confTh, iouTh, keep);
    for (int idx: keep) out.push_back({boxes[idx], ids[idx], scores[idx]});
    return out;
}

void Detector::draw(cv::Mat& bgr, const std::vector<DetBox>& dets) {
    for (auto& d: dets) {
        cv::rectangle(bgr, d.box, {0,255,0}, 2);
        auto label = cv::format("ID:%d %.2f", d.classId, d.score);
        cv::putText(bgr, label, {d.box.x, std::max(0,d.box.y-6)}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,255}, 1);
    }
}
