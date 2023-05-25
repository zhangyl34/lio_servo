#include <QtGui>
#include <QApplication>
#include "qtros/main_window.h"

int main(int argc, char **argv) {

    QApplication app(argc, argv);
    class1_ros_qt_demo::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
