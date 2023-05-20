#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "qtros/main_window.h"

namespace class1_ros_qt_demo {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent), main_argc(argc), main_argv(argv) {

    // 创建界面，将信号与槽连接
    ui.setupUi(this);
    // qApp 是 QApplication 对象的全局指针
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    // 显示页
    ui.tab_manager->setCurrentIndex(1);

    // 更新 log
    ui.listView2->setModel(lioNode.loggingModel());
    QObject::connect(&lioNode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView2()));

    // lio
    QObject::connect(ui.button2_rosStart, SIGNAL(released()), this, SLOT(startLIO()));
    QObject::connect(ui.button2_rosEnd, SIGNAL(released()), this, SLOT(endLIO()));

    // can
    QObject::connect(ui.button2_w1s, SIGNAL(released()), this, SLOT(button2_w1s_slot()));
	QObject::connect(ui.button2_w2s, SIGNAL(released()), this, SLOT(button2_w2s_slot()));
	QObject::connect(ui.button2_canSend, SIGNAL(released()), this, SLOT(button2_canSend_slot()));
	QObject::connect(ui.button2_canStop, SIGNAL(released()), this, SLOT(button2_canStop_slot()));

}

MainWindow::~MainWindow() {}

void MainWindow::updateLoggingView2() {
    ui.listView2->scrollToBottom();
}

// 关闭窗口时，默认调用
void MainWindow::closeEvent(QCloseEvent *event) {
	QMainWindow::closeEvent(event);
}

void MainWindow::startLIO() {
    lioNode.init(main_argc, main_argv);
}

void MainWindow::endLIO() {
    lioNode.exit();
}

void MainWindow::button2_w1s_slot() {
    motor_1_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit_ctrl_velocity_1->text().toFloat());
}

void MainWindow::button2_w2s_slot() {
    motor_2_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit_ctrl_velocity_2->text().toFloat());
}

void MainWindow::button2_canSend_slot() {
    motor_1_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit_ctrl_velocity_1->text().toFloat());
	motor_2_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit_ctrl_velocity_2->text().toFloat());
}

void MainWindow::button2_canStop_slot() {
	motor_1_drive_control.ctrlMotorQuickStop();
	motor_2_drive_control.ctrlMotorQuickStop();
}

}  // namespace class1_ros_qt_demo

