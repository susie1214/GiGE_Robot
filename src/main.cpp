// main.cpp
// - Qt6 application entry point
#include <QApplication>
#include "MainWindow.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow w;
    w.setWindowTitle("MV-CU050-30GC Control Tool (Qt6)");
    w.resize(1280, 800);
    w.show();
    return app.exec();
}
