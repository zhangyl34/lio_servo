#include <QtGui>
#include <QApplication>
#include "qtros/main_window.h"

/***CAN***/
//#include "simple_can_open.h"

int main(int argc, char **argv) {

    /***CAN***/
	// initCanDrive(1, 500);
	// initCanDrive(0, 1000);

    QApplication app(argc, argv);
    class1_ros_qt_demo::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
