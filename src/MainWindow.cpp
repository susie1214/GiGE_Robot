// MainWindow.cpp
#include "MainWindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QDateTime>
#include <QPixmap>
#include <QApplication>

// BGR->RGB 변환은 FrameProcessor에서/혹은 여기서 선택적으로.
// 현재 QImage는 RGB888로 받도록 유지.
static QImage MatBGRToQImage(const cv::Mat& rgb) {
    return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    buildUi();
    cam_ = std::make_unique<CameraWrapper>();
    proc_.setAlpha(1.0);
    proc_.setBeta(0.0);
    proc_.setZoom(1);
    proc_.setKernelSize(5);
    proc_.setSigmaX(1.2);
    proc_.setFlipH(false);
    proc_.setFlipV(false);

    cam_->setFrameCallback([this](const cv::Mat& bgr) {
        pushFrame(bgr);
        if (recording_) rec_.write(bgr);
        });

    uiTimer_.setInterval(100);
    connect(&uiTimer_, &QTimer::timeout, this, &MainWindow::onUiTick);
    uiTimer_.start();
    fpsTick_.satart();
}

MainWindow::~MainWindow() {}

void MainWindow::buildUi() {
    // ===== 라이트 블루 테마(QSS) =====
    qApp->setStyleSheet(R"(
        QMainWindow { background: #EAF6FF; }
        QPushButton {
            background: qlineargradient(x1:0,y1:0, x2:0,y2:1,
                                        stop:0 #7EC8FF, stop:1 #5AAEF0);
            border: none; border-radius: 10px; padding: 6px 12px;
            color: white; font-weight: 600;
        }
        QPushButton:hover { background: #66BBFF; }
        QPushButton:pressed { background: #459DE6; }
        QLabel { color: #0F3554; }
        QCheckBox { color: #0F3554; }
        QGroupBox {
            border: 1px solid #CFE9FF; border-radius: 12px;
            margin-top: 8px; background: #F7FBFF;
        }
        QGroupBox::title {
            subcontrol-origin: margin; left: 10px;
            padding: 2px 6px; color: #2B6EA6; font-weight: 600;
        }
        QSlider::groove:horizontal { height: 6px; background: #CFE9FF; border-radius: 3px; }
        QSlider::handle:horizontal {
            width: 14px; background: white; border: 2px solid #5AAEF0;
            margin: -5px 0; border-radius: 9px;
        }
        QSlider::sub-page:horizontal { background: #5AAEF0; border-radius: 3px; }
    )");

    auto topBar = new QHBoxLayout;
    btnOpen_ = new QPushButton("Open");
    btnClose_ = new QPushButton("Close");
    btnStart_ = new QPushButton("Start");
    btnStop_ = new QPushButton("Stop");
    topBar->addWidget(btnOpen_);
    topBar->addWidget(btnClose_);
    topBar->addWidget(btnStart_);
    topBar->addWidget(btnStop_);
    topBar->addStretch();

    auto paramBar = new QHBoxLayout;
    spAlpha_ = new QDoubleSpinBox; spAlpha_->setRange(0.1, 3.0); spAlpha_->setSingleStep(0.1); spAlpha_->setValue(1.0);
    spBeta_ = new QDoubleSpinBox; spBeta_->setRange(-100, 100); spBeta_->setSingleStep(1.0); spBeta_->setValue(0.0);
    spZoom_ = new QSpinBox;       spZoom_->setRange(1, 4);      spZoom_->setValue(1);
    spExposure_ = new QSpinBox;    spExposure_->setRange(50, 50000); spExposure_->setValue(8000);
    auto btnSetExpo = new QPushButton("Set Exposure");
    paramBar->addWidget(new QLabel("alpha")); paramBar->addWidget(spAlpha_);
    paramBar->addWidget(new QLabel("beta"));  paramBar->addWidget(spBeta_);
    paramBar->addWidget(new QLabel("zoom"));  paramBar->addWidget(spZoom_);
    paramBar->addSpacing(20);
    paramBar->addWidget(new QLabel("Exposure(us)"));
    paramBar->addWidget(spExposure_); paramBar->addWidget(btnSetExpo);
    paramBar->addStretch();

    // ---- 기존 필터 바 + (추가) 커널/시그마/플립 ----
    auto filtBar = new QHBoxLayout;
    chkGray_ = new QCheckBox("Gray");
    chkHSV_ = new QCheckBox("HSV(red mask)");
    chkBlur_ = new QCheckBox("Blur");
    chkGauss_ = new QCheckBox("Gaussian");
    filtBar->addWidget(chkGray_);
    filtBar->addWidget(chkHSV_);
    filtBar->addWidget(chkBlur_);
    filtBar->addWidget(chkGauss_);

    // 추가: 커널/시그마
    spKernel_ = new QSpinBox; spKernel_->setRange(3, 31); spKernel_->setValue(5); spKernel_->setSingleStep(2);
    spSigma_ = new QDoubleSpinBox; spSigma_->setRange(0.5, 5.0); spSigma_->setDecimals(1); spSigma_->setSingleStep(0.1); spSigma_->setValue(1.2);
    filtBar->addSpacing(10);
    filtBar->addWidget(new QLabel("Kernel")); filtBar->addWidget(spKernel_);
    filtBar->addWidget(new QLabel("Sigma"));  filtBar->addWidget(spSigma_);

    // 추가: 플립
    chkFlipH_ = new QCheckBox("Flip H(좌우)");
    chkFlipV_ = new QCheckBox("Flip V(상하)");
    filtBar->addSpacing(10);
    filtBar->addWidget(chkFlipH_);
    filtBar->addWidget(chkFlipV_);
    filtBar->addStretch();

    auto featBar = new QHBoxLayout;
    btnSnapshot_ = new QPushButton("Snapshot");
    btnRecord_ = new QPushButton("Record");
    btnOCR_ = new QPushButton("OCR");
    btnDetect_ = new QPushButton("Detect");
    lblRec_ = new QLabel("");
    lblFps_ = new QLabel("FPS: -");
    featBar->addWidget(btnSnapshot_);
    featBar->addWidget(btnRecord_);
    featBar->addWidget(btnOCR_);
    featBar->addWidget(btnDetect_);
    featBar->addSpacing(20);
    featBar->addWidget(lblRec_);
    featBar->addWidget(lblFps_);
    featBar->addStretch();

    auto mid = new QHBoxLayout;
    lblView_ = new QLabel("No Video"); lblView_->setMinimumSize(960, 540); lblView_->setAlignment(Qt::AlignCenter);
    txtOCR_ = new QPlainTextEdit; txtOCR_->setPlaceholderText("OCR result...");
    mid->addWidget(lblView_, 3);
    mid->addWidget(txtOCR_, 2);

    auto root = new QWidget;
    auto vbox = new QVBoxLayout(root);
    vbox->addLayout(topBar);
    vbox->addLayout(paramBar);
    vbox->addLayout(filtBar);
    vbox->addLayout(featBar);
    vbox->addLayout(mid);
    setCentralWidget(root);

    connect(btnOpen_, &QPushButton::clicked, this, &MainWindow::onOpen);
    connect(btnClose_, &QPushButton::clicked, this, &MainWindow::onClose);
    connect(btnStart_, &QPushButton::clicked, this, &MainWindow::onStart);
    connect(btnStop_, &QPushButton::clicked, this, &MainWindow::onStop);

    connect(btnSnapshot_, &QPushButton::clicked, this, &MainWindow::onSnapshot);
    connect(btnRecord_, &QPushButton::clicked, this, &MainWindow::onRecord);
    connect(btnOCR_, &QPushButton::clicked, this, &MainWindow::onOCR);
    connect(btnDetect_, &QPushButton::clicked, this, &MainWindow::onDetect);

    connect(spAlpha_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onAlphaChanged);
    connect(spBeta_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onBetaChanged);
    connect(spZoom_, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onZoomChanged);
    connect(btnSetExpo, &QPushButton::clicked, this, &MainWindow::onSetExposure);

    connect(chkGray_, &QCheckBox::toggled, this, &MainWindow::onGrayToggled);
    connect(chkHSV_, &QCheckBox::toggled, this, &MainWindow::onHSVToggled);
    connect(chkBlur_, &QCheckBox::toggled, this, &MainWindow::onBlurToggled);
    connect(chkGauss_, &QCheckBox::toggled, this, &MainWindow::onGaussToggled);

    // --- 추가 연결 ---
    connect(spKernel_, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onKernelChanged);
    connect(spSigma_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onSigmaChanged);
    connect(chkFlipH_, &QCheckBox::toggled, this, &MainWindow::onFlipHToggled);
    connect(chkFlipV_, &QCheckBox::toggled, this, &MainWindow::onFlipVToggled);
}

void MainWindow::onOpen() {
    try {
        cam_->openFirstGigE();
        QMessageBox::information(this, "Info", "Camera OPEN");
    }
    catch (const std::exception& e) {
        QMessageBox::critical(this, "Open failed", e.what());
    }
}
void MainWindow::onClose() {
    onStop();
    cam_->close();
    QMessageBox::information(this, "Info", "Camera CLOSED");
}
void MainWindow::onStart() {
    if (!cam_->isOpen()) { QMessageBox::warning(this, "Warn", "Open camera first"); return; }
    cam_->setInt("Width", 1920);
    cam_->setInt("Height", 1080);
    cam_->setFloat("AcquisitionFrameRate", 30.0f);
    cam_->setEnum("TriggerMode", 0);
    cam_->startGrabbing();
    running_ = true;
}
void MainWindow::onStop() {
    if (!running_) return;
    cam_->stopGrabbing();
    running_ = false;
    if (recording_) onRecord();
}
void MainWindow::onSnapshot() {
    cv::Mat frame; { std::scoped_lock lk(frameMtx_); if (lastFrame_.empty()) return; frame = lastFrame_.clone(); }
    const auto name = QString("snap_%1.png").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    cv::imwrite(name.toStdString(), frame);
    QMessageBox::information(this, "Saved", name);
}
void MainWindow::onRecord() {
    if (!recording_) {
        const auto name = QString("rec_%1.mp4").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
        if (rec_.open(name.toStdString(), 30.0, cv::Size(1920, 1080))) {
            recording_ = true;
            lblRec_->setText("● REC");
        }
    }
    else {
        recording_ = false;
        lblRec_->setText("");
        rec_.close();
    }
}
void MainWindow::onOCR() {
    cv::Mat frame; { std::scoped_lock lk(frameMtx_); if (lastFrame_.empty()) return; frame = lastFrame_.clone(); }
    auto text = ocr_.run(frame, "assets/tessdata", "eng+kor");
    txtOCR_->setPlainText(QString::fromStdString(text));
}
void MainWindow::onDetect() {
    if (!liveDetect_) {
        if (!det_.isReady()) {
            if (!det_.loadOnnx("assets/yolov8s.onnx")) {
                QMessageBox::warning(this, "Detect", "ONNX load failed");
                return;
            }
        }
        liveDetect_ = true;
        btnDetect_->setText("Stop Detect");
    }
    else {
        liveDetect_ = false;
        btnDetect_->setText("Detect");
    }
}
void MainWindow::onAlphaChanged(double v) { proc_.setAlpha(v); }
void MainWindow::onBetaChanged(double v) { proc_.setBeta(v); }
void MainWindow::onZoomChanged(int v) { proc_.setZoom(v); }
void MainWindow::onSetExposure() {
    double us = spExposure_->value();
    cam_->setFloat("ExposureTime", static_cast<float>(us));
}
void MainWindow::onGrayToggled(bool on) { proc_.setGray(on); }
void MainWindow::onHSVToggled(bool on) { proc_.setHSV(on); }
void MainWindow::onBlurToggled(bool on) { proc_.setBlur(on); }
void MainWindow::onGaussToggled(bool on) { proc_.setGauss(on); }

// --- 추가 슬롯 구현 ---
void MainWindow::onKernelChanged(int v) {
    // 홀수 강제(사용자가 짝수로 올려도 맞춰줌)
    if (v % 2 == 0) { v += 1; spKernel_->setValue(v); }
    proc_.setKernelSize(v);
}
void MainWindow::onSigmaChanged(double v) {
    if (v < 0.1) v = 0.1;
    proc_.setSigmaX(v);
}
void MainWindow::onFlipHToggled(bool on) { proc_.setFlipH(on); }
void MainWindow::onFlipVToggled(bool on) { proc_.setFlipV(on); }

void MainWindow::onUiTick() {
    cv::Mat frame;
    {
        std::scoped_lock lk(frameMtx_);
        if (lastFrame_.empty()) return;
        frame = lastFrame_.clone();
    }

    auto out = proc_.run(frame);

    if (liveDetect_ && det_.isReady()) {
        auto res = det_.infer(out);
        det_.draw(out, res);
    }

    updateView(out);
    fpsCounter_++;
    if (fpsTick_.elapsed() >= 1000) {
        lblFps_->setText(QString("FPS: %1").arg(fpsCounter_));
        fpsCounter_ = 0;
        fpsTick_.restart();
    }
}
void MainWindow::updateView(const cv::Mat& bgr) {
    auto img = MatBGRToQImage(bgr);
    lblView_->setPixmap(QPixmap::fromImage(img).scaled(lblView_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}
void MainWindow::pushFrame(const cv::Mat& bgr) {
    std::scoped_lock lk(frameMtx_);
    lastFrame_ = bgr.clone();
}
