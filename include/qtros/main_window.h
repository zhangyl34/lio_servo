#pragma once

#include <QtWidgets/QMainWindow>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h>

#include "ui_main_window.h"
#include "lio/qnode_lio.h"

#include "motor_drive_control.h"

namespace class1_ros_qt_demo {

struct CartPara {
	double wheelsRadius;    // mm
	double wheelsDistance;  // distance between the two differential wheels.
	double a;           // distance from wheels-center to the origin of IMU along x direction.
	double b;           // distance from wheels-center to the origin of IMU along y direction.
	double theta;       // angle from ws to c
	double linVelLim;   // mm/s
	double linAccLim;   // mm/s^2
	double angVelLim;   // rad/s
	double angAccLim;   // rad/s^2
	double communFreq;  // Hz, the communication frequency between the cart and the computer.
};

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
	MotorDriveControl motor_1_drive_control, motor_2_drive_control;  // left, right

	/***Close-loop control***/
	cv::Mat drawWCPath();
	cv::Mat drawIMUPath();
	QTimer* timer_path;
	std::vector<std::vector<Eigen::Vector2d>> wheelsCenter_paths;  // 28,101
	std::vector<Eigen::Vector2d> wheelsCenter_simulationPath;
	std::vector<float> init_oris;  // 28
	float tar_ori;                 // ori_c
	int pathId;
	// 实际控制点是 IMU
	std::vector<std::vector<Eigen::Vector2d>> IMU_paths;  // 28,101
	std::vector<Eigen::Vector2d> IMU_controlPoint;        // 21
	std::vector<Eigen::Vector2d> IMU_simulationPath;

	/***cart model***/
	// return (xDot,yDot,thetaDot)
	Eigen::Vector3d fromWheelsVel2CartVel3(const Eigen::Vector3d& cartCurrPosIn, const Eigen::Vector2d& wheelsVelIn) const;
	// return (v,w)
	Eigen::Vector2d fromCartVel2MiddleVel2(const double& xl_dot, const double& yl_dot, const double& theta) const;
	// return (vR,vL) mm/s
	Eigen::Vector2d fromMiddleVel2WheelsVel2(const Eigen::Vector2d& cartCurrVelIn) const;
	// return control 坐标系下 (xc,yc,oric)
	Eigen::Vector3d fromGPose2CPose(const std::vector<double>& GPose) const;
	CartPara cartPara;

};

}  // namespace class1_ros_qt_demo

