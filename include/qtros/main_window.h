#pragma once

#include <QtWidgets/QMainWindow>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "ui_main_window.h"
#include "lio/qnode_lio.h"

#include "motor_drive_control.h"

namespace class1_ros_qt_demo {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings();
	void WriteSettings();

    // 关闭窗口时，默认调用
    void closeEvent(QCloseEvent *event);

public Q_SLOTS:
    void updateLoggingView2();
	void startLIO();
	void endLIO();

	/***CAN***/
	void refreshUITimerFun();
	void button2_w1s_slot();
	void button2_w2s_slot();
	void button2_canSend_slot();
	void button2_canStop_slot();

	/***Close-loop control***/
	void updatePath();
	// path 生成时，考虑的是 wheels-center
	void button1_genPath_slot();
    void button1_pathLast_slot();
    void button1_pathNext_slot();
	// path 设置时，考虑的是 IMU
	void button1_setPath_slot();
	void button1_startTest_slot();
	void button1_stopTest_slot();
	void updateLoggingView1();

private:
	int main_argc;
	char** main_argv;
	Ui::MainWindowDesign ui;
	LIONode lioNode;

	/***CAN***/
	QTimer *update_vel_timer;
	MotorDriveControl motor_1_drive_control, motor_2_drive_control;

	/***Close-loop control***/
	cv::Mat drawWCPath();
	QTimer* timer_path;
	std::vector<std::vector<Eigen::Vector2d>> wheelsCenter_paths;  // 28,101
	int pathId;
	// 实际控制点是 IMU
	double cartPara_l;
	std::vector<std::vector<Eigen::Vector2d>> IMU_paths;  // 28,101
	std::vector<Eigen::Vector2d> IMU_controlPoint;        // 21
	std::vector<Eigen::Vector2d> IMU_simulationPath;
};

}  // namespace class1_ros_qt_demo

