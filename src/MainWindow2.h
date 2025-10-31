// MainWindow.h
#pragma once
#include <QMainWindow>
#include <QTimer>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <mutex>
#include <memory>
#include <opencv2/opencv.hpp>
#include "CameraWrapper.h"
#include "FrameProcessor.h"
#include "Recorder.h"
#include "OcrEngine.h"
#include "Detector.h"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent=nullptr);
    ~MainWindow();

private slots:
    void onOpen();
    void onClose();
    void onStart();
    void onStop();
    void onSnapshot();
    void onRecord();
    void onOCR();
    void onDetect();
    void onAlphaChanged(double v);
    void onBetaChanged(double v);
    void onZoomChanged(int v);
    void onSetExposure();
    void onGrayToggled(bool on);
    void onHSVToggled(bool on);
    void onBlurToggled(bool on);
    void onGaussToggled(bool on);
    void onUiTick();

private:
    void buildUi();
    void updateView(const cv::Mat& bgr);
    void pushFrame(const cv::Mat& bgr);

private:
    std::unique_ptr<CameraWrapper> cam_;
    FrameProcessor proc_;
    Recorder rec_;
    OcrEngine ocr_;
    Detector det_;

    std::mutex frameMtx_;
    cv::Mat lastFrame_;
    bool running_{false};
    bool recording_{false};
    bool liveDetect_ = false;

    QLabel* lblView_{nullptr};
    QLabel* lblRec_{nullptr};
    QLabel* lblFps_{nullptr};

    QPushButton *btnOpen_{nullptr}, *btnClose_{nullptr}, *btnStart_{nullptr}, *btnStop_{nullptr};
    QPushButton *btnSnapshot_{nullptr}, *btnRecord_{nullptr}, *btnOCR_{nullptr}, *btnDetect_{nullptr};

    QDoubleSpinBox *spAlpha_{nullptr}, *spBeta_{nullptr};
    QSpinBox *spZoom_{nullptr}, *spExposure_{nullptr};
    QCheckBox *chkGray_{nullptr}, *chkHSV_{nullptr}, *chkBlur_{nullptr}, *chkGauss_{nullptr};
    QPlainTextEdit* txtOCR_{nullptr};

    QTimer uiTimer_;
};
