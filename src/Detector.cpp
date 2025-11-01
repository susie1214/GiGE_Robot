// Detector.cpp
#include "Detector.h"
#include <opencv2/dnn.hpp>
#include <iostream>

// ========== �� �ε� ==========
bool Detector::loadOnnx(const std::string& onnxPath, int inputSize, float confTh, float iouTh) {
    try {
        // �� CHANGED: ���� �̸� ���ռ�(onxxPath ���), ���� ��� Ž�� ����
        net_ = cv::dnn::readNetFromONNX(onnxPath);

        // �����/�Ϲ� CPU Ÿ��
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        // �Ķ���� ����
        inp_ = inputSize;
        confTh_ = confTh;
        iouTh_ = iouTh;

        ready_ = !net_.empty();
        if (!ready_) std::cerr << "[Detector] Net is empty after readNetFromONNX\n";

    }
    catch (const cv::Exception& e) {
        std::cerr << "[Detector] ONNX load failed (cv): " << e.msg << std::endl;
        ready_ = false;
    }
    catch (const std::exception& e) {
        std::cerr << "[Detector] ONNX load failed (std): " << e.what() << std::endl; // �� FIX: ���� �޽��� ���
        ready_ = false;
    }
    return ready_;
}

// ========== ��� �ټ� ���̾ƿ� �Ǻ� ==========
bool Detector::isLayout_Nx85(const cv::Mat& out) const {
    // �� CHANGED: �Լ���/���� ����
    if (out.dims != 3) return true; // �⺻��
    int d0 = out.size[0], d1 = out.size[1], d2 = out.size[2];
    if (d2 == 85) return true;      // (1,N,85)
    if (d1 == 85) return false;     // (1,85,N)
    return true;                    // �⺻��
}

// ========== �߷� ==========
std::vector<DetBox> Detector::infer(const cv::Mat& bgr) {
    std::vector<DetBox> out;
    if (!ready_ || bgr.empty()) return out;

    // 1) ��ó��: BGR��RGB, 1/255, �Է�ũ��(inp_) ����
    cv::Mat blob = cv::dnn::blobFromImage(
        bgr, 1 / 255.0, cv::Size(inp_, inp_), cv::Scalar(),
        /*swapRB=*/true, /*crop=*/false
    );
    net_.setInput(blob);

    // 2) �߷�
    cv::Mat pred = net_.forward();    // (1,N,85) �Ǵ� (1,85,N)
    const int W = bgr.cols, H = bgr.rows;
    const float sx = static_cast<float>(W) / static_cast<float>(inp_);
    const float sy = static_cast<float>(H) / static_cast<float>(inp_);

    // 3) ���̾ƿ� ����ȭ �� (N,85)
    cv::Mat pm;
    int N = 0;
    const bool Nx85 = isLayout_Nx85(pred); // �� FIX: �Լ� ���
    if (Nx85) {
        // (1,N,85) �� (N,85)
        N = pred.size[1];
        pm = pred.reshape(1, N);
    }
    else {
        // (1,85,N) �� (N,85) ��ġ ����
        N = pred.size[2];
        pm.create(N, 85, CV_32F);
        for (int n = 0; n < N; ++n) {
            for (int c = 0; c < 85; ++c) {
                // �� FIX: ��Ÿ/�ε��� ���� ���� (nc, cv::MAt �� ����)
                pm.at<float>(n, c) = pred.ptr<float>(0, c)[n];
            }
        }
    }

    // 4) ��ó��: obj * clsMax, ��ǥ ������ ����
    std::vector<cv::Rect> boxes;
    std::vector<float>    scores;
    std::vector<int>      ids;

    for (int i = 0; i < N; ++i) {
        float* p = pm.ptr<float>(i);
        // p[0:4] = cx,cy,w,h  | p[4] = obj  | p[5:] = class scores(80)
        float obj = p[4];

        int   bestId = -1;
        float bestCls = 0.f;
        for (int c = 5; c < 85; ++c) {
            if (p[c] > bestCls) { bestCls = p[c]; bestId = c - 5; }
        }
        float score = obj * bestCls;                   // �� CHANGED: YOLOv8 ���� ����
        if (score < confTh_) continue;

        float x0, y0, x1, y1;
        xywh2xyxy(p[0], p[1], p[2], p[3], sx, sy, x0, y0, x1, y1); // �� CHANGED: ���� ��ȯ

        cv::Rect r(cv::Point((int)x0, (int)y0), cv::Point((int)x1, (int)y1));
        r &= cv::Rect(0, 0, W, H);
        if (r.width > 0 && r.height > 0) {
            boxes.push_back(r);
            scores.push_back(score);                   // �� FIX: score ���Ϳ� score ���� (best �� score)
            ids.push_back(bestId);
        }
    }

    // 5) NMS
    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, confTh_, iouTh_, keep);

    out.reserve(keep.size());
    for (int k : keep) {
        out.push_back({ boxes[k], ids[k], scores[k] });
    }
    return out;
}

// ========== �ð�ȭ ==========
void Detector::draw(cv::Mat& bgr, const std::vector<DetBox>& dets) {
    for (const auto& d : dets) {
        cv::rectangle(bgr, d.box, { 0,255,0 }, 2);
        auto label = cv::format("ID:%d %.2f", d.classId, d.score);
        cv::putText(bgr, label, { d.box.x, std::max(0, d.box.y - 6) },
            cv::FONT_HERSHEY_SIMPLEX, 0.5, { 0,255,255 }, 1);
    }
}

void Detector::draw(cv::Mat& bgr, const std::vector<DetBox>& dets, const std::vector<std::string>& names) {
    // �� CHANGED: Ŭ���� �̸����� ����ϴ� �����ε�
    for (const auto& d : dets) {
        cv::rectangle(bgr, d.box, { 0,255,0 }, 2);
        std::string name = (d.classId >= 0 && d.classId < (int)names.size())
            ? names[d.classId] : ("id=" + std::to_string(d.classId));
        auto label = cv::format("%s %.2f", name.c_str(), d.score);
        cv::putText(bgr, label, { d.box.x, std::max(0, d.box.y - 6) },
            cv::FONT_HERSHEY_SIMPLEX, 0.5, { 0,255,255 }, 1);
    }
}
