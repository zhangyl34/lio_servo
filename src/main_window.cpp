#include "qtros/main_window.h"

namespace class1_ros_qt_demo {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent), main_argc(argc), main_argv(argv),
    motor_1_drive_control(0x0F), motor_2_drive_control(0x17),
    cartPara_l(-1000), pathId(-1) {

    // 设置 .ui 文件
    ui.setupUi(this);
    // qApp 是 QApplication 对象的全局指针
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    // 读取参数
    ReadSettings();
    // 显示页
    ui.tab_manager->setCurrentIndex(0);

    // 更新 log
    ui.listView1->setModel(lioNode.loggingModel());
    QObject::connect(&lioNode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView1()));

    // LIO
    QObject::connect(ui.button2_rosStart, SIGNAL(released()), this, SLOT(startLIO()));
    QObject::connect(ui.button2_rosEnd, SIGNAL(released()), this, SLOT(endLIO()));

    /***CAN***/
    QObject::connect(ui.button2_w1s, SIGNAL(released()), this, SLOT(button2_w1s_slot()));
	QObject::connect(ui.button2_w2s, SIGNAL(released()), this, SLOT(button2_w2s_slot()));
	QObject::connect(ui.button2_canSend, SIGNAL(released()), this, SLOT(button2_canSend_slot()));
	QObject::connect(ui.button2_canStop, SIGNAL(released()), this, SLOT(button2_canStop_slot()));

    update_vel_timer = new QTimer(this);
	QObject::connect(update_vel_timer, SIGNAL(timeout()), this, SLOT(refreshUITimerFun()));

    /***Close-loop control***/
    timer_path = new QTimer(this);
    QObject::connect(timer_path, SIGNAL(timeout()), this, SLOT(updatePath()));
    QObject::connect(ui.button1_genPath, SIGNAL(released()), this, SLOT(button1_genPath_slot()));
    QObject::connect(ui.button1_pathLast, SIGNAL(released()), this, SLOT(button1_pathLast_slot()));
    QObject::connect(ui.button1_pathNext, SIGNAL(released()), this, SLOT(button1_pathNext_slot()));
    QObject::connect(ui.button1_setPath, SIGNAL(released()), this, SLOT(button1_setPath_slot()));
    QObject::connect(ui.button1_startTest, SIGNAL(released()), this, SLOT(button1_startTest_slot()));
    QObject::connect(ui.button1_stopTest, SIGNAL(released()), this, SLOT(button1_stopTest_slot()));

	ui.button1_startTest->setEnabled(false);
	ui.button1_stopTest->setEnabled(false);
}

MainWindow::~MainWindow() {
    /***CAN***/
    motor_1_drive_control.ctrlMotorQuickStop();
	motor_2_drive_control.ctrlMotorQuickStop();
}

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    QString startx = settings.value("startx", QString("1500.0")).toString();
    QString starty = settings.value("starty", QString("1500.0")).toString();
    QString startori = settings.value("startori", QString("0.2618")).toString();
    QString endx = settings.value("endx", QString("4500.0")).toString();
    QString endy = settings.value("endy", QString("5500.0")).toString();
    QString endori = settings.value("endori", QString("1.0472")).toString();
    ui.lineEdit1_startx->setText(startx);
    ui.lineEdit1_starty->setText(starty);
    ui.lineEdit1_startori->setText(startori);
    ui.lineEdit1_endx->setText(endx);
    ui.lineEdit1_endy->setText(endy);
    ui.lineEdit1_endori->setText(endori);
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    settings.setValue("startx",ui.lineEdit1_startx->text());
    settings.setValue("starty",ui.lineEdit1_starty->text());
    settings.setValue("startori",ui.lineEdit1_startori->text());
    settings.setValue("endx",ui.lineEdit1_endx->text());
    settings.setValue("endy",ui.lineEdit1_endy->text());
    settings.setValue("endori",ui.lineEdit1_endori->text());
}

// 关闭窗口时，默认调用
void MainWindow::closeEvent(QCloseEvent *event) {
    WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::updateLoggingView2() {
    // ui.listView2->scrollToBottom();
}

void MainWindow::startLIO() {
    // lioNode.init(main_argc, main_argv);
}

void MainWindow::endLIO() {
    // lioNode.exit();
}

/***CAN***/
void MainWindow::button2_w1s_slot() {
    motor_1_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit2_1->text().toFloat());
}

void MainWindow::button2_w2s_slot() {
    motor_2_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit2_2->text().toFloat());
}

void MainWindow::button2_canSend_slot() {
    motor_1_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit2_1->text().toFloat());
	motor_2_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit2_2->text().toFloat());
}

void MainWindow::button2_canStop_slot() {
	motor_1_drive_control.ctrlMotorQuickStop();
	motor_2_drive_control.ctrlMotorQuickStop();
}

// 更新电机状态与电机速度
void MainWindow::refreshUITimerFun() {
    QString status;

	MotorStatus motor_status = motor_1_drive_control.currentStatus();
	for (int i = 0; i < 8; i++) {
		if (drive_status_list[i].status == motor_status) {
			status = drive_status_list[i].status_name;
			break;
		}
	}
	ui.label2_5->setText(status);
	
	motor_status = motor_2_drive_control.currentStatus();
	for (int i = 0; i < 8; i++) {
		if (drive_status_list[i].status == motor_status) {
			status = drive_status_list[i].status_name;
			break;
		}
	}
	ui.label2_11->setText(status);

	ui.label2_7->setText(QString::number(motor_1_drive_control.currentVelocity()));
	ui.label2_13->setText(QString::number(motor_2_drive_control.currentVelocity()));
}

/***Close-loop control***/
// total 28 paths, in respect to wheels-center
void MainWindow::button1_genPath_slot() {

	ui.button1_startTest->setEnabled(false);
	ui.button1_stopTest->setEnabled(false);
	wheelsCenter_paths.clear();
	IMU_paths.clear();
	pathId = -1;

	Eigen::Vector3d initPoseRaw;
    initPoseRaw[0] = ui.lineEdit1_startx->text().toFloat();
    initPoseRaw[1] = ui.lineEdit1_starty->text().toFloat();
    initPoseRaw[2] = ui.lineEdit1_startori->text().toFloat();
    Eigen::Vector3d tarPosRaw;
    tarPosRaw[0] = ui.lineEdit1_endx->text().toFloat();
    tarPosRaw[1] = ui.lineEdit1_endy->text().toFloat();
    tarPosRaw[2] = ui.lineEdit1_endori->text().toFloat();
    Eigen::Vector3d initPose;
    initPose[0] = initPoseRaw[0] - tarPosRaw[0];
    initPose[1] = initPoseRaw[1] - tarPosRaw[1];
    initPose[2] = initPoseRaw[2];
    Eigen::Vector3d tarPos;
    tarPos[0] = 0;
    tarPos[1] = 0;
    tarPos[2] = tarPosRaw[2];

	// y = x^2
	Eigen::Matrix<double, 2, 3> parabolic(Eigen::Matrix<double, 2, 3>::Zero());
    parabolic(0, 0) = initPose(1) * initPose(1) / 4 / (initPose(0) - initPose(1) / tan(tarPos(2)));
    parabolic(0, 1) = -parabolic(0, 0) / tan(tarPos(2)) / tan(tarPos(2));
    parabolic(0, 2) = -2 * parabolic(0, 0) / tan(tarPos(2));
	parabolic(1, 0) = initPose(0) * initPose(0) / 4 / (initPose(1) - initPose(0) * tan(tarPos(2)));
	parabolic(1, 1) = -2 * parabolic(1, 0) * tan(tarPos(2));
	parabolic(1, 2) = -parabolic(1, 0) * tan(tarPos(2)) * tan(tarPos(2));

	// y = x^3
	const int angleNum = 13;  // -60 ~ +60
	Eigen::VectorXd initOri(angleNum);
	initOri << initPose(2) - 60 * EIGEN_PI / 180, initPose(2) - 50 * EIGEN_PI / 180, initPose(2) - 40 * EIGEN_PI / 180,
		initPose(2) - 30 * EIGEN_PI / 180, initPose(2) - 20 * EIGEN_PI / 180, initPose(2) - 10 * EIGEN_PI / 180,
		initPose(2) - 0 * EIGEN_PI / 180, initPose(2) + 10 * EIGEN_PI / 180, initPose(2) + 20 * EIGEN_PI / 180,
		initPose(2) + 30 * EIGEN_PI / 180, initPose(2) + 40 * EIGEN_PI / 180, initPose(2) + 50 * EIGEN_PI / 180,
		initPose(2) + 60 * EIGEN_PI / 180;
	Eigen::MatrixXd cubic1(angleNum, 4);
	Eigen::MatrixXd cubic2(angleNum, 4);
	for (int i=0; i<angleNum; i++) {
		cubic1(i, 0) = 0;
		cubic1(i, 1) = tan(tarPos(2));
		cubic1(i, 2) = (3 * initPose(1) - initPose(0) * (tan(initOri(i)) + 2 * tan(tarPos(2)))) / initPose(0) / initPose(0);
		cubic1(i, 3) = (initPose(0) * (tan(initOri(i)) + tan(tarPos(2))) - 2 * initPose(1)) / initPose(0) / initPose(0) / initPose(0);
		cubic2(i, 0) = 0;
		cubic2(i, 1) = 1 / tan(tarPos(2));
		cubic2(i, 2) = (3 * initPose(0) - initPose(1) * (1 / tan(initOri(i)) + 2 / tan(tarPos(2)))) / initPose(1) / initPose(1);
		cubic2(i, 3) = (initPose(1) * (1 / tan(initOri(i)) + 1 / tan(tarPos(2))) - 2 * initPose(0)) / initPose(1) / initPose(1) / initPose(1);
	}

	// generate path
    const int pointNum = 101;
	std::vector<Eigen::Vector2d> wheelsCenter_path;  // wheels-center
	std::vector<Eigen::Vector2d> IMU_path;           // IMU
	for (int i=0; i<pointNum; i++) {
		double z2 = initPose(1) / 2 * (1 + cos(EIGEN_PI * i / (pointNum-1)));
		wheelsCenter_path.emplace_back(Eigen::Vector2d((z2 - parabolic(0, 2)) * (z2 - parabolic(0, 2)) / 4 / parabolic(0, 0) + parabolic(0, 1) + tarPosRaw[0], z2 + tarPosRaw[1]));
		float theta = atan(2 * parabolic(0, 0) / (z2 - parabolic(0, 2)));
		if (theta > 0) {
			theta = -CV_PI + theta;
		}
		IMU_path.emplace_back(Eigen::Vector2d((z2 - parabolic(0, 2)) * (z2 - parabolic(0, 2)) / 4 / parabolic(0, 0) + parabolic(0, 1) + tarPosRaw[0] + cartPara_l * cos(theta), z2 + tarPosRaw[1] + cartPara_l * sin(theta)));
	}
	wheelsCenter_paths.emplace_back(wheelsCenter_path);
	IMU_paths.emplace_back(IMU_path);

	wheelsCenter_path.clear();
	IMU_path.clear();
	for (int i=0; i<pointNum; i++) {
		double z1 = initPose(0) / 2 * (1 + cos(EIGEN_PI * i / (pointNum-1)));
		wheelsCenter_path.emplace_back(Eigen::Vector2d(z1 + tarPosRaw[0], (z1 - parabolic(1, 1)) * (z1 - parabolic(1, 1)) / 4 / parabolic(1, 0) + parabolic(1, 2) + tarPosRaw[1]));
		float theta = atan((z1 - parabolic(1, 1))/(2 * parabolic(1, 0)));
		if (theta > 0) {
			theta = -CV_PI + theta;
		}
		IMU_path.emplace_back(Eigen::Vector2d(z1 + tarPosRaw[0] + cartPara_l * cos(theta), (z1 - parabolic(1, 1)) * (z1 - parabolic(1, 1)) / 4 / parabolic(1, 0) + parabolic(1, 2) + tarPosRaw[1] + cartPara_l * sin(theta)));
	}
	wheelsCenter_paths.emplace_back(wheelsCenter_path);
	IMU_paths.emplace_back(IMU_path);

	for (int pathId_local=0; pathId_local<angleNum; pathId_local++) {
		wheelsCenter_path.clear();
		IMU_path.clear();
		for (int i=0; i<pointNum; i++) {
			double z1 = initPose(0) / 2 * (1 + cos(EIGEN_PI * i / (pointNum-1)));
			wheelsCenter_path.emplace_back(Eigen::Vector2d(z1 + tarPosRaw[0], cubic1(pathId_local, 0) + cubic1(pathId_local, 1) * z1 + cubic1(pathId_local, 2) * z1 * z1 + cubic1(pathId_local, 3) * z1 * z1 * z1 + tarPosRaw[1]));
			float theta = atan(cubic1(pathId_local, 1) + 2 * z1 * cubic1(pathId_local, 2) + 3 * z1 * z1 * cubic1(pathId_local, 3));
			if (theta > 0) {
				theta = -CV_PI + theta;
			}
			IMU_path.emplace_back(Eigen::Vector2d(z1 + tarPosRaw[0] + cartPara_l * cos(theta), cubic1(pathId_local, 0) + cubic1(pathId_local, 1) * z1 + cubic1(pathId_local, 2) * z1 * z1 + cubic1(pathId_local, 3) * z1 * z1 * z1 + tarPosRaw[1] + cartPara_l * sin(theta)));
		}
		wheelsCenter_paths.emplace_back(wheelsCenter_path);
		IMU_paths.emplace_back(IMU_path);
	}

	for (int pathId_local = 0; pathId_local < angleNum; pathId_local++) {
		wheelsCenter_path.clear();
		IMU_path.clear();
		for (int i=0; i<pointNum; i++) {
			double z2 = initPose(1) / 2 * (1 + cos(EIGEN_PI*i / (pointNum-1)));
			wheelsCenter_path.emplace_back(Eigen::Vector2d(cubic2(pathId_local, 0) + cubic2(pathId_local, 1) * z2 + cubic2(pathId_local, 2) * z2 * z2 + cubic2(pathId_local, 3) * z2 * z2 * z2 + tarPosRaw[0], z2 + tarPosRaw[1]));	
			float theta = atan(1/(cubic2(pathId_local, 1) + 2 * z2 * cubic2(pathId_local, 2) + 3 * z2 * z2 * cubic2(pathId_local, 3)));
			if (theta > 0)
				theta = -CV_PI + theta;
			IMU_path.emplace_back(Eigen::Vector2d(cubic2(pathId_local, 0) + cubic2(pathId_local, 1) * z2 + cubic2(pathId_local, 2) * z2 * z2 + cubic2(pathId_local, 3) * z2 * z2 * z2 + tarPosRaw[0] + cartPara_l * cos(theta), z2 + tarPosRaw[1] + cartPara_l * sin(theta)));
		}
		wheelsCenter_paths.emplace_back(wheelsCenter_path);
		IMU_paths.emplace_back(IMU_path);
	}

	cv::Mat imgPath = drawWCPath();
	QImage srcQImage = QImage((uchar*)(imgPath.data), imgPath.cols, imgPath.rows, imgPath.step, QImage::Format_RGB888);
	ui.label1_3->setPixmap(QPixmap::fromImage(srcQImage));
	ui.label1_3->show();

    char buffer[256];
    sprintf(buffer, "Path ID (0-%d): %d", static_cast<int>(wheelsCenter_paths.size()) - 1, pathId);
    ui.label1_1->setText(buffer);
}

void MainWindow::button1_pathLast_slot() {
	if (pathId <= 0) {
		return;
	}
	else {
		pathId--;
		cv::Mat imgPath = drawWCPath();
		QImage srcQImage = QImage((uchar*)(imgPath.data), imgPath.cols, imgPath.rows, imgPath.step, QImage::Format_RGB888);
		ui.label1_3->setPixmap(QPixmap::fromImage(srcQImage));
		ui.label1_3->show();

		char buffer[256];
		sprintf(buffer, "Path ID (0-%d): %d", static_cast<int>(wheelsCenter_paths.size()) - 1, pathId);
		ui.label1_1->setText(buffer);
	}
}

void MainWindow::button1_pathNext_slot() {
	if (pathId >= (static_cast<int>(wheelsCenter_paths.size())-1)) {
		return;
	}
	else {
		pathId++;
		cv::Mat imgPath = drawWCPath();
		QImage srcQImage = QImage((uchar*)(imgPath.data), imgPath.cols, imgPath.rows, imgPath.step, QImage::Format_RGB888);
		ui.label1_3->setPixmap(QPixmap::fromImage(srcQImage));
		ui.label1_3->show();

		char buffer[256];
		sprintf(buffer, "Path ID (0-%d): %d", static_cast<int>(wheelsCenter_paths.size()) - 1, pathId);
		ui.label1_1->setText(buffer);
	}
}

cv::Mat MainWindow::drawWCPath() {
    cv::Scalar color2(17,204,254);
	cv::Scalar color1(127,104,254);
    int halfWidth = 150;
	int height = 350;
	cv::Mat pathImg = cv::Mat(cv::Size(halfWidth*2,height), CV_8UC3, cv::Scalar(241,240,237));  //255,254,249
    if (pathId<0 || pathId>=wheelsCenter_paths.size()) {
		return pathImg;
	}
	for (int i=0; i<wheelsCenter_paths[pathId].size(); i++) {
        cv::circle(pathImg, cv::Point(wheelsCenter_paths[pathId].at(i)[0] / 20.0, wheelsCenter_paths[pathId].at(i)[1] / 20.0), 2, color2, -1);
		cv::circle(pathImg, cv::Point(IMU_paths[pathId].at(i)[0] / 20.0, IMU_paths[pathId].at(i)[1] / 20.0), 2, color1, -1);
	}
    return pathImg;
}

void MainWindow::button1_setPath_slot() {
	if (pathId<0 || pathId>=wheelsCenter_paths.size()) {
		return;
	}
	ui.button1_startTest->setEnabled(true);
	ui.button1_stopTest->setEnabled(true);
	char buffer[256];
    sprintf(buffer, "Selected ID: %d", pathId);
    ui.label1_10->setText(buffer);

    // 生成 IMU path 的 20 个控制点
	IMU_controlPoint.clear();

	// 生成 IMU path 的仿真轨迹
	IMU_simulationPath.clear();

	// 绘图

}

void MainWindow::button1_startTest_slot() {
    // 1000ms 更新一次 label1_3
	timer_path->start(1000);
	// 开启 lio 节点
    lioNode.init(main_argc, main_argv);

    // Eigen::Vector3d log_tarPos = mC.getTarPos();
	// neal::logger(LOG_INFO_, "target: " + std::to_string(log_tarPos[0]) + ' ' + std::to_string(log_tarPos[1]) + ' ' + std::to_string(log_tarPos[2]));
	// Eigen::Vector4d log_motionPlane = mC.getTrolleyPlane();

	// /***control loop***/
	// std::vector<Eigen::Vector2d> pathRaw = mC.getRoadSign();
	// double disThresh = 10;  // 10 mm
	// int dpSign = 1;  // desiredPath sign post (from 0-19).
	// Eigen::Vector2d tarPosi(pathRaw.at(dpSign)[0], pathRaw.at(dpSign)[1]);
	// Eigen::Vector3d cartPosi(mC.getCartCurrPos());  // cart registered inital pose.
	// pose3d cartPosRaw;  // path logger.
	// Eigen::Vector2d cartVeli(Eigen::Vector2d::Zero());
	// Eigen::Vector2d middleVeli(Eigen::Vector2d::Zero());
	// Eigen::Vector2d wheelsVeli(Eigen::Vector2d::Zero());
	// Eigen::Vector3d cartVel3i(Eigen::Vector3d::Zero());
	// bool trackSuccess = false;  // marker tracker.
	// cv::Mat imgCaptured;
	// int failCount = 0;
	// float costThreshold = 5.0;
	// // while doesn't reach the target position.
	// while (pow(cartPosi(0) - pathRaw.at(pathRaw.size() - 1)[0], 2) + pow(cartPosi(1) - pathRaw.at(pathRaw.size() - 1)[1], 2) > disThresh * disThresh) {
	// 	// track another capturedImg and update cartPosi.
	// 	trackSuccess = monitor.estimatePose(imgCaptured, costThreshold);  // search in the ROI.
	// 	failCount = 0;
	// 	while (!trackSuccess && failCount < 10) {  // try 10 times.
	// 		failCount++;
	// 		std::cout << "trackNextCaptured failed!" << std::endl;
	// 		trackSuccess = monitor.estimatePose(imgCaptured, costThreshold, true);  // search in the whole image.
	// 	}
	// 	if (!trackSuccess) {
	// 		break;
	// 	}
	// 	// update focal voltage.
	// 	float currentBestFV = monitor.fromDistance2FV();
	// 	currentBestFV = caspian->setFocusVoltage(currentBestFV);
	// 	char buff[8];
	// 	sprintf(buff, "%0.1f", currentBestFV);
	// 	ui.lineEdit_focalVol->setText(buff);
	
	// 	cartPosRaw = monitor.getPose();  // dim = 6.
	// 	mC.updateCartCurrPos(cartPosRaw);  // used for imgshow.
	// 	cartPosi = mC.getCartCurrPos();  // dim = 3.
	// 	neal::logger(LOG_INFO_, "cart_pose_raw: " + std::to_string(cartPosRaw.tvec[0]) + ' ' + std::to_string(cartPosRaw.tvec[1]) + ' '	+ std::to_string(cartPosRaw.tvec[2]) + 
	// 		' ' + std::to_string(cartPosRaw.rvec[0]) + ' ' + std::to_string(cartPosRaw.rvec[1]) + ' ' + std::to_string(cartPosRaw.rvec[2]));
	// 	neal::logger(LOG_INFO_, "cart_pose: " + std::to_string(cartPosi[0]) + ' ' + std::to_string(cartPosi[1]) + ' ' + std::to_string(cartPosi[2]));

	// 	// update tarPosi when get close to the previous one.
	// 	double disErr = sqrt(pow(cartPosi(0) - tarPosi(0), 2) + pow(cartPosi(1) - tarPosi(1), 2));
	// 	if (dpSign < (static_cast<int>(pathRaw.size()) - 1)) {  // < 19
	// 		double preErr = sqrt(pow(cartPosi(0) - pathRaw.at(dpSign - 1)[0], 2) + pow(cartPosi(1) - pathRaw.at(dpSign - 1)[1], 2));
	// 		/*parameter 0.5 needs to be adjusted.*/
	// 		if (preErr > disErr)
	// 		{
	// 			//roadSign.emplace_back(Eigen::Vector2d(cartPosi(0), cartPosi(1)));
	// 			dpSign++;
	// 			tarPosi = pathRaw.at(dpSign);  // update tarPosi.
	// 			disErr = sqrt(pow(cartPosi(0) - tarPosi(0), 2) + pow(cartPosi(1) - tarPosi(1), 2));
	// 			//std::cout << cartPosi(0) << cartPosi(1) << std::endl;
	// 		}
	// 	}

	// 	// compute next velocity.
	// 	if (cartPara.l == 0) {  // p control.
	// 		/* kp=1*/
	// 		double angErr = atan2(tarPosi(1) - cartPosi(1), tarPosi(0) - cartPosi(0)) - cartPosi(2);
	// 		if (angErr > EIGEN_PI) {
	// 			angErr -= 2 * EIGEN_PI;
	// 		}
	// 		else if (angErr < -EIGEN_PI) {
	// 			angErr += 2 * EIGEN_PI;
	// 		}
	// 		Eigen::Vector2d cartVelNi((std::min)({ disErr, cartPara.linVelLim }),
	// 			fabs(angErr) > cartPara.angVelLim ? cartPara.angVelLim * angErr / fabs(angErr) : angErr);
	// 		if ((cartVelNi(0) - cartVeli(0)) * cartPara.communFreq > cartPara.linAccLim)  // v_dot
	// 			cartVelNi(0) = cartVeli(0) + cartPara.linAccLim / cartPara.communFreq;
	// 		else if ((cartVelNi(0) - cartVeli(0)) * cartPara.communFreq < -cartPara.linAccLim)
	// 			cartVelNi(0) = cartVeli(0) - cartPara.linAccLim / cartPara.communFreq;
	// 		if ((cartVelNi(1) - cartVeli(1)) * cartPara.communFreq > cartPara.angAccLim)  // w_dot
	// 			cartVelNi(1) = cartVeli(1) + cartPara.angAccLim / cartPara.communFreq;
	// 		else if ((cartVelNi(1) - cartVeli(1)) * cartPara.communFreq < -cartPara.angAccLim)
	// 			cartVelNi(1) = cartVeli(1) - cartPara.angAccLim / cartPara.communFreq;
	// 		cartVeli = cartVelNi;
	// 		wheelsVeli = mC.fromMiddleVel2WheelsVel2(cartVeli);  // vR, vL
	// 	}
	// 	else {  // cart coordinate coincide with the marker coordinate.
	// 		/* k1=k2=1*/
	// 		double xm_dot = (tarPosi(0) - cartPosi(0));
	// 		double ym_dot = (tarPosi(1) - cartPosi(1));
	// 		double m_dot = sqrt(pow(xm_dot, 2) + pow(ym_dot, 2));
	// 		double threshold = sqrt(pow(cartPara.linVelLim, 2) + pow(cartPara.l * cartPara.angVelLim, 2));
	// 		if (m_dot > threshold) {
	// 			xm_dot = xm_dot / m_dot * threshold;
	// 			ym_dot = ym_dot / m_dot * threshold;
	// 		}
	// 		Eigen::Vector2d middleVelNi = mC.fromCartVel2MiddleVel2(xm_dot, ym_dot, cartPosi(2));
	// 		middleVelNi(0) = cartPara.linVelLim * tanh(middleVelNi(0)/10.0);
	// 		middleVelNi(1) = cartPara.angVelLim * tanh(middleVelNi(1)*5.0);

	// 		if ((middleVelNi(0) - middleVeli(0)) * cartPara.communFreq > cartPara.linAccLim)  // v_dot
	// 			middleVelNi(0) = middleVeli(0) + cartPara.linAccLim / cartPara.communFreq;
	// 		else if ((middleVelNi(0) - middleVeli(0)) * cartPara.communFreq < -cartPara.linAccLim)
	// 			middleVelNi(0) = middleVeli(0) - cartPara.linAccLim / cartPara.communFreq;
	// 		if ((middleVelNi(1) - middleVeli(1)) * cartPara.communFreq > cartPara.angAccLim)  // w_dot
	// 			middleVelNi(1) = middleVeli(1) + cartPara.angAccLim / cartPara.communFreq;
	// 		else if ((middleVelNi(1) - middleVeli(1)) * cartPara.communFreq < -cartPara.angAccLim)
	// 			middleVelNi(1) = middleVeli(1) - cartPara.angAccLim / cartPara.communFreq;
	// 		middleVeli = middleVelNi;
	// 		wheelsVeli = mC.fromMiddleVel2WheelsVel2(middleVeli);  // vR, vL
	// 	}

	// 	// refresh Qt widget
	// 	//cv::Mat img_temp;  // update label_imgFlow
	// 	//float resizeScale = (std::min)({ 270.0 / imgCaptured.cols, 210.0 / imgCaptured.rows });
	// 	//cv::resize(imgCaptured, img_temp, cv::Size(), resizeScale, resizeScale);
	// 	//QImage srcQImage = QImage((uchar*)(img_temp.data), img_temp.cols, img_temp.rows, img_temp.step, QImage::Format_RGB888);
	// 	//ui.label_imgCap->setPixmap(QPixmap::fromImage(srcQImage));
	// 	//ui.label_imgCap->resize(srcQImage.size());
	// 	//ui.label_imgCap->show();

	// 	// send wheelsVeli
	// 	float motor1_speed = wheelsVeli.x() * 60.0 / (2.0 * CV_PI * cartPara.wheelsRadius);  // rpm
	// 	float motor2_speed = wheelsVeli.y() * 60.0 / (2.0 * CV_PI * cartPara.wheelsRadius);
	// 	if (fabs(motor1_speed) > 100 || fabs(motor2_speed) > 100) {  // if motor speed too high.
	// 		neal::logger(LOG_ERROR, "motor1_speed: " + std::to_string(motor1_speed));
	// 		neal::logger(LOG_ERROR, "motor2_speed: " + std::to_string(motor2_speed));
	// 		break;
	// 	}

	// 	neal::logger(LOG_INFO_, "motor1_speed: " + std::to_string(motor1_speed));
	// 	neal::logger(LOG_INFO_, "motor2_speed: " + std::to_string(motor2_speed));
	// 	motor_1_drive_control.ctrlMotorMoveByVelocity(motor1_speed);  // x = vR, form rpm to count/ms
	// 	motor_2_drive_control.ctrlMotorMoveByVelocity(motor2_speed);  // y = vL
	//  }

	//  // stop test.
	// motor_1_drive_control.ctrlMotorQuickStop();
	// motor_2_drive_control.ctrlMotorQuickStop();
	// neal::logger(LOG_INFO_, "motor1_speed: " + std::to_string(0.0));
	// neal::logger(LOG_INFO_, "motor2_speed: " + std::to_string(0.0));

}

void MainWindow::button1_stopTest_slot() {
    timer_path->stop();
    lioNode.exit();

    /***CAN***/
    // motor_1_drive_control.ctrlMotorQuickStop();
	// motor_2_drive_control.ctrlMotorQuickStop();
}

void MainWindow::updateLoggingView1() {
    ui.listView1->scrollToBottom();
}

// 更新频率低一点
void MainWindow::updatePath() {
	// video_frame = GxCamera->getImgFlow();
	// cv::Mat img_temp;
	// cv::resize(video_frame, img_temp, cv::Size(), 0.2, 0.2);
	// //cv::imshow("frame", img_temp);
	// //cv::waitKey(1);
	// QImage srcQImage = QImage((uchar*)(img_temp.data), img_temp.cols, img_temp.rows, img_temp.step, QImage::Format_Grayscale8);
	// ui.label_imgFlow->setPixmap(QPixmap::fromImage(srcQImage));
	// ui.label_imgFlow->resize(srcQImage.size());
	// ui.label_imgFlow->show();
}

}  // namespace class1_ros_qt_demo

