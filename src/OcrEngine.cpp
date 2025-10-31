// OcrEngine.cpp
#include "OcrEngine.h"
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

std::string OcrEngine::run(const cv::Mat& bgr, const std::string& tessdata, const std::string& lang) {
    if (bgr.empty()) return "";
    cv::Mat g, bin;
    cv::cvtColor(bgr, g, cv::COLOR_BGR2GRAY);
    cv::threshold(g, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    Pix* pix = pixCreate(bin.cols, bin.rows, 8);
    for (int y=0; y<bin.rows; ++y) {
        memcpy((uint8_t*)pixGetData(pix) + y*pixGetWpl(pix)*4, bin.ptr(y), bin.cols);
    }

    tesseract::TessBaseAPI api;
    if (api.Init(tessdata.c_str(), lang.c_str()) != 0) {
        pixDestroy(&pix);
        return "[OCR init failed]";
    }
    api.SetImage(pix);
    char* out = api.GetUTF8Text();
    std::string text = out ? out : "";
    delete [] out;
    api.End();
    pixDestroy(&pix);
    return text;
}
