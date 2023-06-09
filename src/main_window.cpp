#include "qtros/main_window.h"

namespace class1_ros_qt_demo {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent), main_argc(argc), main_argv(argv),
    motor_1_drive_control(0x0F), motor_2_drive_control(0x17),
	pathId(-1) {

    // 设置 .ui 文件
    ui.setupUi(this);
    // qApp 是 QApplication 对象的全局指针
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    // 读取参数
    ReadSettings();
    // 显示页
    ui.tab_manager->setCurrentIndex(0);

    /***LIO***/
    QObject::connect(ui.button2_rosStart, SIGNAL(released()), this, SLOT(startLIO()));
    QObject::connect(ui.button2_rosEnd, SIGNAL(released()), this, SLOT(endLIO()));

	ui.listView2->setModel(lioNode.loggingModel());
    QObject::connect(&lioNode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView2()));

    /***CAN***/
    QObject::connect(ui.button2_w1s, SIGNAL(released()), this, SLOT(button2_w1s_slot()));
	QObject::connect(ui.button2_w2s, SIGNAL(released()), this, SLOT(button2_w2s_slot()));
	QObject::connect(ui.button2_canSend, SIGNAL(released()), this, SLOT(button2_canSend_slot()));
	QObject::connect(ui.button2_canStop, SIGNAL(released()), this, SLOT(button2_canStop_slot()));

    update_vel_timer = new QTimer(this);
	QObject::connect(update_vel_timer, SIGNAL(timeout()), this, SLOT(refreshUITimerFun()));

    /***Close-loop control***/
    QObject::connect(ui.button1_genPath, SIGNAL(released()), this, SLOT(button1_genPath_slot()));
    QObject::connect(ui.button1_pathLast, SIGNAL(released()), this, SLOT(button1_pathLast_slot()));
    QObject::connect(ui.button1_pathNext, SIGNAL(released()), this, SLOT(button1_pathNext_slot()));
    QObject::connect(ui.button1_setPath, SIGNAL(released()), this, SLOT(button1_setPath_slot()));
    QObject::connect(ui.button1_startTest, SIGNAL(released()), this, SLOT(button1_startTest_slot()));
    QObject::connect(ui.button1_stopTest, SIGNAL(released()), this, SLOT(button1_stopTest_slot()));

	ui.button1_startTest->setEnabled(false);
	ui.button1_stopTest->setEnabled(false);

	/***cart model***/
	cartPara.wheelsRadius = 75;
	cartPara.wheelsDistance = 580;
	cartPara.a = 577.5+181.67-44;  // careful
	cartPara.b = -16;              // careful
	cartPara.theta = 0;            // careful
	cartPara.linVelLim = 40;
	cartPara.linAccLim = 15;
	cartPara.angVelLim = 0.1;
	cartPara.angAccLim = 0.04;
	cartPara.communFreq = 10;  // careful
}


MainWindow::~MainWindow() {
	lioNode.exit();
    /***CAN***/
    motor_1_drive_control.ctrlMotorQuickStop();
	motor_2_drive_control.ctrlMotorQuickStop();
}

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    QString startx = settings.value("startx", QString("1500.0")).toString();
    QString starty = settings.value("starty", QString("5500.0")).toString();
    QString startori = settings.value("startori", QString("-35")).toString();
    QString endx = settings.value("endx", QString("4500.0")).toString();
    QString endy = settings.value("endy", QString("1500.0")).toString();
    QString endori = settings.value("endori", QString("-65")).toString();
	QString leftv = settings.value("leftv", QString("42")).toString();
    QString rightv = settings.value("rightv", QString("42")).toString();
    ui.lineEdit1_startx->setText(startx);
    ui.lineEdit1_starty->setText(starty);
    ui.lineEdit1_startori->setText(startori);
    ui.lineEdit1_endx->setText(endx);
    ui.lineEdit1_endy->setText(endy);
    ui.lineEdit1_endori->setText(endori);
	ui.lineEdit2_1->setText(leftv);
    ui.lineEdit2_2->setText(leftv);
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    settings.setValue("startx",ui.lineEdit1_startx->text());
    settings.setValue("starty",ui.lineEdit1_starty->text());
    settings.setValue("startori",ui.lineEdit1_startori->text());
    settings.setValue("endx",ui.lineEdit1_endx->text());
    settings.setValue("endy",ui.lineEdit1_endy->text());
    settings.setValue("endori",ui.lineEdit1_endori->text());
	settings.setValue("leftv",ui.lineEdit2_1->text());
    settings.setValue("rightv",ui.lineEdit2_2->text());
}

// 关闭窗口时，默认调用
void MainWindow::closeEvent(QCloseEvent *event) {
    WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::updateLoggingView2() {
    ui.listView2->scrollToBottom();
}

void MainWindow::startLIO() {
    lioNode.init(main_argc, main_argv);
	sleep(1);  // 主线程睡眠 1s，不影响 lio 线程

	Eigen::Matrix3d R_W_G = lioNode.readRWG();
	neal::logger(neal::LOG_INFO, "G_R_W: " + std::to_string(R_W_G(0,0)) + ' ' + std::to_string(R_W_G(0,1)) +
		' ' + std::to_string(R_W_G(0,2)) + ' ' + std::to_string(R_W_G(1,0)) + ' ' + std::to_string(R_W_G(1,1)) +
		' ' + std::to_string(R_W_G(1,2)) + ' ' + std::to_string(R_W_G(2,0)) + ' ' + std::to_string(R_W_G(2,1)) +
		' ' + std::to_string(R_W_G(2,2)));
	
}

void MainWindow::endLIO() {
    lioNode.exit();
}

/***CAN***/
void MainWindow::button2_w1s_slot() {
    motor_1_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit2_1->text().toFloat());
}

void MainWindow::button2_w2s_slot() {
    motor_2_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit2_2->text().toFloat());
}

void MainWindow::button2_canSend_slot() {
	// update_vel_timer->start(1000);
    motor_1_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit2_1->text().toFloat());
	motor_2_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit2_2->text().toFloat());
}

void MainWindow::button2_canStop_slot() {
	// update_vel_timer->stop();
	motor_1_drive_control.ctrlMotorQuickStop();
	motor_2_drive_control.ctrlMotorQuickStop();
}

// 更新电机速度
void MainWindow::refreshUITimerFun() {
	// ui.label2_7->setText(QString::number(motor_1_drive_control.currentVelocity()));
	// ui.label2_13->setText(QString::number(motor_2_drive_control.currentVelocity()));
}

/***Close-loop control***/
// total 28 paths, in respect to wheels-center
void MainWindow::button1_genPath_slot() {

	ui.button1_startTest->setEnabled(false);
	ui.button1_stopTest->setEnabled(false);
	wheelsCenter_paths.clear();
	init_oris.clear();
	IMU_paths.clear();
	pathId = -1;

	Eigen::Vector3d initPoseRaw;
    initPoseRaw[0] = ui.lineEdit1_startx->text().toFloat();
    initPoseRaw[1] = ui.lineEdit1_starty->text().toFloat();
    initPoseRaw[2] = ui.lineEdit1_startori->text().toFloat() * EIGEN_PI / 180.0;
    Eigen::Vector3d tarPosRaw;
    tarPosRaw[0] = ui.lineEdit1_endx->text().toFloat();
    tarPosRaw[1] = ui.lineEdit1_endy->text().toFloat();
    tarPosRaw[2] = ui.lineEdit1_endori->text().toFloat() * EIGEN_PI / 180.0;
    Eigen::Vector3d initPose;
    initPose[0] = initPoseRaw[0] - tarPosRaw[0];
    initPose[1] = initPoseRaw[1] - tarPosRaw[1];
    initPose[2] = initPoseRaw[2];
    Eigen::Vector3d tarPos;
    tarPos[0] = 0;
    tarPos[1] = 0;
    tarPos[2] = tarPosRaw[2];

	tar_ori = tarPosRaw[2];

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
		// [-pi/2,pi/2]
		float theta = atan(2 * parabolic(0, 0) / (z2 - parabolic(0, 2)));
		if (theta > 0) {
			theta = -EIGEN_PI + theta;
		}
		if (i==0) {
			init_oris.emplace_back(theta+cartPara.theta);
		}
		IMU_path.emplace_back(Eigen::Vector2d((z2 - parabolic(0, 2)) * (z2 - parabolic(0, 2)) / 4 / parabolic(0, 0) + parabolic(0, 1) + tarPosRaw[0] + cartPara.a*cos(theta) - cartPara.b*sin(theta),
											  z2 + tarPosRaw[1] + cartPara.a*sin(theta) + cartPara.b*cos(theta)));
	}
	wheelsCenter_paths.emplace_back(wheelsCenter_path);
	IMU_paths.emplace_back(IMU_path);

	wheelsCenter_path.clear();
	IMU_path.clear();
	for (int i=0; i<pointNum; i++) {
		double z1 = initPose(0) / 2 * (1 + cos(EIGEN_PI * i / (pointNum-1)));
		wheelsCenter_path.emplace_back(Eigen::Vector2d(z1 + tarPosRaw[0], (z1 - parabolic(1, 1)) * (z1 - parabolic(1, 1)) / 4 / parabolic(1, 0) + parabolic(1, 2) + tarPosRaw[1]));
		float theta = atan((z1 - parabolic(1, 1))/(2 * parabolic(1, 0)));
		if (initPose(0) > 0) {
			theta = -EIGEN_PI + theta;
		}
		if (i==0) {
			init_oris.emplace_back(theta+cartPara.theta);
		}
		IMU_path.emplace_back(Eigen::Vector2d(z1 + tarPosRaw[0] + cartPara.a*cos(theta) - cartPara.b*sin(theta),
											  (z1 - parabolic(1, 1)) * (z1 - parabolic(1, 1)) / 4 / parabolic(1, 0) + parabolic(1, 2) + tarPosRaw[1] + cartPara.a*sin(theta) + cartPara.b*cos(theta)));
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
			if (initPose(0) > 0) {
				theta = -EIGEN_PI + theta;
			}
			if (i==0) {
				init_oris.emplace_back(theta+cartPara.theta);
			}
			IMU_path.emplace_back(Eigen::Vector2d(z1 + tarPosRaw[0] + cartPara.a*cos(theta) - cartPara.b*sin(theta),
												  cubic1(pathId_local, 0) + cubic1(pathId_local, 1) * z1 + cubic1(pathId_local, 2) * z1 * z1 + cubic1(pathId_local, 3) * z1 * z1 * z1 + tarPosRaw[1] + cartPara.a*sin(theta) + cartPara.b*cos(theta)));
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
			if (theta > 0) {
				theta = -EIGEN_PI + theta;
			}
			if (i==0) {
				init_oris.emplace_back(theta+cartPara.theta);
			}
			IMU_path.emplace_back(Eigen::Vector2d(cubic2(pathId_local, 0) + cubic2(pathId_local, 1) * z2 + cubic2(pathId_local, 2) * z2 * z2 + cubic2(pathId_local, 3) * z2 * z2 * z2 + tarPosRaw[0] + cartPara.a*cos(theta) - cartPara.b*sin(theta),
			                                      z2 + tarPosRaw[1] + cartPara.a*sin(theta) + cartPara.b*cos(theta)));
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
    cv::Scalar color1(17,204,254);   // blue: wheels-center
	cv::Scalar color2(127,104,254);  // purple: IMU
    int halfWidth = 150;
	int height = 350;
	cv::Mat pathImg = cv::Mat(cv::Size(halfWidth*2,height), CV_8UC3, cv::Scalar(241,240,237));
    if (pathId<0 || pathId>=wheelsCenter_paths.size()) {
		return pathImg;
	}
	for (int i=0; i<wheelsCenter_paths[pathId].size(); i++) {
        cv::circle(pathImg, cv::Point(wheelsCenter_paths[pathId].at(i)[0] / 20.0, height-wheelsCenter_paths[pathId].at(i)[1] / 20.0), 2, color1, -1);
		cv::circle(pathImg, cv::Point(IMU_paths[pathId].at(i)[0] / 20.0, height-IMU_paths[pathId].at(i)[1] / 20.0), 2, color2, -1);
	}
    return pathImg;
}

cv::Mat MainWindow::drawIMUPath() {
	cv::Scalar color2(127,104,254);  // purple: simulation IMU path
	cv::Scalar color3(127,204,54);   // green: simulation wheels-center path
    int halfWidth = 150;
	int height = 350;
	cv::Mat pathImg = cv::Mat(cv::Size(halfWidth*2,height), CV_8UC3, cv::Scalar(241,240,237));
    if (pathId<0 || pathId>=wheelsCenter_paths.size()) {
		return pathImg;
	}
	for (int i=0; i<IMU_simulationPath.size(); i++) {
		cv::circle(pathImg, cv::Point(IMU_simulationPath.at(i)[0] / 20.0, height-IMU_simulationPath.at(i)[1] / 20.0), 2, color2, -1);
		cv::circle(pathImg, cv::Point(wheelsCenter_simulationPath.at(i)[0] / 20.0, height-wheelsCenter_simulationPath.at(i)[1] / 20.0), 2, color3, -1);
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

    // 生成 IMU path 的 21 个控制点
	IMU_controlPoint.clear();
	for (int i=0; i<21; i++) {
		IMU_controlPoint.emplace_back(IMU_paths[pathId].at(i*5));
	}

	// 生成 IMU path 的仿真轨迹
	IMU_simulationPath.clear();
	wheelsCenter_simulationPath.clear();
	/*parameter 0.05 needs to be adjusted.*/
	double disThresh = 10;  // 10mm
	int cpid = 1;           // control point id (1-20).
	Eigen::Vector2d tarPosi(IMU_controlPoint.at(cpid)(0), IMU_controlPoint.at(cpid)(1));
	Eigen::Vector3d cartPosi;
	cartPosi << IMU_paths[pathId][0](0), IMU_paths[pathId][0](1), init_oris[pathId];
	Eigen::Vector2d cartVeli(Eigen::Vector2d::Zero());
	Eigen::Vector2d middleVeli(Eigen::Vector2d::Zero());
	Eigen::Vector2d wheelsVeli(Eigen::Vector2d::Zero());
	Eigen::Vector3d cartVel3i(Eigen::Vector3d::Zero());
	int iterNum = 0;
	while ((sqrt(pow(cartPosi(0) - IMU_controlPoint.back()(0), 2) + pow(cartPosi(1) - IMU_controlPoint.back()(1), 2)) > disThresh) && iterNum < 10000) {
		iterNum++;
		double disErr = sqrt(pow(cartPosi(0) - tarPosi(0), 2) + pow(cartPosi(1) - tarPosi(1), 2));
		// 更新 control point
		if (cpid < (IMU_controlPoint.size()-1)) {  // cpid<20
			double preErr = sqrt(pow(cartPosi(0) - IMU_controlPoint.at(cpid - 1)(0), 2) + pow(cartPosi(1) - IMU_controlPoint.at(cpid - 1)(1), 2));
			/*parameter 0.5 needs to be adjusted.*/
			if (disErr < preErr) {  // update tarPosi when get close to the previous one.
				cpid++;
				tarPosi = IMU_controlPoint.at(cpid);  // update tarPosi.
				disErr = sqrt(pow(cartPosi(0) - tarPosi(0), 2) + pow(cartPosi(1) - tarPosi(1), 2));
			}
		}

		// compute next position and orientatio
		cartVel3i = fromWheelsVel2CartVel3(cartPosi, wheelsVeli);
		Eigen::Vector3d cartPosNi;
		cartPosNi << cartVel3i(0) / cartPara.communFreq + cartPosi(0),
			cartVel3i(1) / cartPara.communFreq + cartPosi(1),
			cartVel3i(2) / cartPara.communFreq + cartPosi(2);
		cartPosi = cartPosNi;

		// compute next velocity.
		/* k1=k2=1*/
		double xm_dot = (tarPosi(0) - cartPosi(0));
		double ym_dot = (tarPosi(1) - cartPosi(1));
		double m_dot = sqrt(pow(xm_dot, 2) + pow(ym_dot, 2));
		double threshold = sqrt(pow(cartPara.linVelLim, 2) + pow(cartPara.a * cartPara.angVelLim, 2));
		if (m_dot > threshold) {
			xm_dot = xm_dot / m_dot * threshold;
			ym_dot = ym_dot / m_dot * threshold;
		}
		Eigen::Vector2d middleVelNi = fromCartVel2MiddleVel2(xm_dot, ym_dot, cartPosi(2));
		middleVelNi(0) = cartPara.linVelLim * tanh(middleVelNi(0) / 10.0);
		middleVelNi(1) = cartPara.angVelLim * tanh(middleVelNi(1) * 5.0);

		if ((middleVelNi(0) - middleVeli(0)) * cartPara.communFreq > cartPara.linAccLim)  // v_dot
			middleVelNi(0) = middleVeli(0) + cartPara.linAccLim / cartPara.communFreq;
		else if ((middleVelNi(0) - middleVeli(0)) * cartPara.communFreq < -cartPara.linAccLim)
			middleVelNi(0) = middleVeli(0) - cartPara.linAccLim / cartPara.communFreq;
		if ((middleVelNi(1) - middleVeli(1)) * cartPara.communFreq > cartPara.angAccLim)  // w_dot
			middleVelNi(1) = middleVeli(1) + cartPara.angAccLim / cartPara.communFreq;
		else if ((middleVelNi(1) - middleVeli(1)) * cartPara.communFreq < -cartPara.angAccLim)
			middleVelNi(1) = middleVeli(1) - cartPara.angAccLim / cartPara.communFreq;
		middleVeli = middleVelNi;
		wheelsVeli = fromMiddleVel2WheelsVel2(middleVeli);  // vR, vL

		IMU_simulationPath.emplace_back(Eigen::Vector2d(cartPosi(0), cartPosi(1)));
		wheelsCenter_simulationPath.emplace_back(Eigen::Vector2d(cartPosi(0) - cartPara.a*cos(cartPosi(2)-cartPara.theta) + cartPara.b*sin(cartPosi(2)-cartPara.theta),
																 cartPosi(1) - cartPara.a*sin(cartPosi(2)-cartPara.theta) - cartPara.b*cos(cartPosi(2)-cartPara.theta)));
	}

	// 绘制两条 IMU 轨迹
    cv::Mat imgPath = drawIMUPath();
	QImage srcQImage = QImage((uchar*)(imgPath.data), imgPath.cols, imgPath.rows, imgPath.step, QImage::Format_RGB888);
	ui.label1_3->setPixmap(QPixmap::fromImage(srcQImage));
	ui.label1_3->show();

    // 显示角度误差
	float ori_err = tar_ori - cartPosi(2);
	while (ori_err > EIGEN_PI) {
		ori_err -= 2*EIGEN_PI;
	}
	while (ori_err < -EIGEN_PI) {
		ori_err += 2*EIGEN_PI;
	}
    sprintf(buffer, "Ori error: %0.1f deg", static_cast<double>(ori_err * 180.0 / EIGEN_PI));
    ui.label1_11->setText(buffer);

}

void MainWindow::button1_startTest_slot() {

	// 开启 lio 节点
    if (!lioNode.init(main_argc, main_argv)) {
		return;
	}
	sleep(1);  // 1s

	// 记录参数
	neal::logger(neal::LOG_INFO, "initial: " + std::to_string(IMU_paths[pathId][0](0)) +
		' ' + std::to_string(IMU_paths[pathId][0](1)) + ' ' + std::to_string(init_oris[pathId]));
	neal::logger(neal::LOG_INFO, "target: " + std::to_string(IMU_controlPoint.back()(0)) +
		' ' + std::to_string(IMU_controlPoint.back()(1)) + ' ' + std::to_string(tar_ori));
	Eigen::Matrix3d R_W_G = lioNode.readRWG();
	neal::logger(neal::LOG_INFO, "G_R_W: " + std::to_string(R_W_G(0,0)) + ' ' + std::to_string(R_W_G(0,1)) +
		' ' + std::to_string(R_W_G(0,2)) + ' ' + std::to_string(R_W_G(1,0)) + ' ' + std::to_string(R_W_G(1,1)) +
		' ' + std::to_string(R_W_G(1,2)) + ' ' + std::to_string(R_W_G(2,0)) + ' ' + std::to_string(R_W_G(2,1)) +
		' ' + std::to_string(R_W_G(2,2)));

	double disThresh = 10;  // 10mm
	int cpid = 1;           // control point id (1-20).
	Eigen::Vector2d tarPosi(IMU_controlPoint.at(cpid)(0), IMU_controlPoint.at(cpid)(1));
	neal::logger(neal::LOG_INFO, "tar_pos: " + std::to_string(tarPosi(0)) + ' ' + std::to_string(tarPosi(1)));
	Eigen::Vector3d cartPosi(fromGPose2CPose(lioNode.read3DPose()));
	Eigen::Vector3d cartPosi_raw(fromGPose2CPose(lioNode.read3DPose()));
	Eigen::Vector2d cartVeli(Eigen::Vector2d::Zero());
	Eigen::Vector2d middleVeli(Eigen::Vector2d::Zero());
	Eigen::Vector2d wheelsVeli(Eigen::Vector2d::Zero());
	Eigen::Vector3d cartVel3i(Eigen::Vector3d::Zero());
	Eigen::Matrix4d kalman_P = 3*Eigen::Matrix4d::Identity();
	while (pow(cartPosi(0) - IMU_controlPoint.back()(0), 2) + pow(cartPosi(1) - IMU_controlPoint.back()(1), 2) > disThresh * disThresh) {
		double disErr = sqrt(pow(cartPosi(0) - tarPosi(0), 2) + pow(cartPosi(1) - tarPosi(1), 2));
		// 更新 control point
		if (cpid < (IMU_controlPoint.size()-1)) {  // cpid<20
			double preErr = sqrt(pow(cartPosi(0) - IMU_controlPoint.at(cpid - 1)(0), 2) + pow(cartPosi(1) - IMU_controlPoint.at(cpid - 1)(1), 2));
			/*parameter 0.5 needs to be adjusted.*/
			if (disErr < preErr) {  // update tarPosi when get close to the previous one.
				cpid++;
				tarPosi = IMU_controlPoint.at(cpid);  // update tarPosi.
				neal::logger(neal::LOG_INFO, "tar_pos: " + std::to_string(tarPosi(0)) + ' ' + std::to_string(tarPosi(1)));
				disErr = sqrt(pow(cartPosi(0) - tarPosi(0), 2) + pow(cartPosi(1) - tarPosi(1), 2));
			}
		}

		cartPosi_raw = fromGPose2CPose(lioNode.read3DPose());
		// control 坐标系下，x y ori
		neal::logger(neal::LOG_INFO, "IMU3D_pose_before_filter: " + std::to_string(cartPosi_raw[0]) + ' ' + std::to_string(cartPosi_raw[1]) + ' ' + std::to_string(cartPosi_raw[2]));

		// Kalman filter
		Eigen::Matrix4d kalman_F;
		kalman_F << 1, 0, 1/cartPara.communFreq, 0,
					0, 1, 0, 1/cartPara.communFreq,
					0, 0, 1, 0,
					0, 0, 0, 1;
		Eigen::Matrix<double,2,4> kalman_H;
		kalman_H << 1, 0, 0, 0,
					0, 1, 0, 0;
		double kalman_q1 = 5.0, kalman_q2 = 5.0;  // 假设速度 40mm/s，位置过程噪声 q/2/10/10*40=1mm
		Eigen::Matrix4d kalman_Q;
		kalman_Q << kalman_q1/3/cartPara.communFreq/cartPara.communFreq/cartPara.communFreq, 0, kalman_q1/2/cartPara.communFreq/cartPara.communFreq, 0,
					0, kalman_q2/3/cartPara.communFreq/cartPara.communFreq/cartPara.communFreq, 0, kalman_q2/2/cartPara.communFreq/cartPara.communFreq,
					kalman_q1/2/cartPara.communFreq/cartPara.communFreq, 0, kalman_q1/cartPara.communFreq, 0,
					0, kalman_q2/2/cartPara.communFreq/cartPara.communFreq, 0, kalman_q2/cartPara.communFreq;
		Eigen::Matrix2d kalman_R;
		kalman_R << 5, 0,
					0, 5;
		Eigen::Vector4d kalman_x;
		cartVel3i = fromWheelsVel2CartVel3(cartPosi, wheelsVeli);
		kalman_x << cartPosi(0), cartPosi(1), cartVel3i(0), cartVel3i(1);
		Eigen::Vector4d kalman_x_ = kalman_F*kalman_x;
		Eigen::Matrix4d kalman_P_ = kalman_F*kalman_P*kalman_F.transpose() + kalman_Q;
		Eigen::Matrix<double,4,2> kalman_K = kalman_P_*kalman_H.transpose()*(kalman_H*kalman_P_*kalman_H.transpose()+kalman_R).inverse();
		kalman_x = kalman_x_ + kalman_K*(cartPosi_raw.block<2,1>(0,0)-kalman_H*kalman_x_);
		kalman_P = (Eigen::Matrix4d::Identity()-kalman_K*kalman_H)*kalman_P_;
		cartPosi << kalman_x(0), kalman_x(1), cartPosi_raw(2);
		// control 坐标系下，x y ori
		neal::logger(neal::LOG_INFO, "IMU3D_pose_after_filter: " + std::to_string(cartPosi[0]) + ' ' + std::to_string(cartPosi[1]) + ' ' + std::to_string(cartPosi[2]));

		// compute next velocity.
		/* k1=k2=1*/
		double xm_dot = (tarPosi(0) - cartPosi(0));
		double ym_dot = (tarPosi(1) - cartPosi(1));
		double m_dot = sqrt(pow(xm_dot, 2) + pow(ym_dot, 2));
		double threshold = sqrt(pow(cartPara.linVelLim, 2) + pow(cartPara.a * cartPara.angVelLim, 2));
		if (m_dot > threshold) {
			xm_dot = xm_dot / m_dot * threshold;
			ym_dot = ym_dot / m_dot * threshold;
		}
		Eigen::Vector2d middleVelNi = fromCartVel2MiddleVel2(xm_dot, ym_dot, cartPosi(2));
		middleVelNi(0) = cartPara.linVelLim * tanh(middleVelNi(0)/10.0);
		middleVelNi(1) = cartPara.angVelLim * tanh(middleVelNi(1)*5.0);

		if ((middleVelNi(0) - middleVeli(0)) * cartPara.communFreq > cartPara.linAccLim)  // v_dot
			middleVelNi(0) = middleVeli(0) + cartPara.linAccLim / cartPara.communFreq;
		else if ((middleVelNi(0) - middleVeli(0)) * cartPara.communFreq < -cartPara.linAccLim)
			middleVelNi(0) = middleVeli(0) - cartPara.linAccLim / cartPara.communFreq;
		if ((middleVelNi(1) - middleVeli(1)) * cartPara.communFreq > cartPara.angAccLim)  // w_dot
			middleVelNi(1) = middleVeli(1) + cartPara.angAccLim / cartPara.communFreq;
		else if ((middleVelNi(1) - middleVeli(1)) * cartPara.communFreq < -cartPara.angAccLim)
			middleVelNi(1) = middleVeli(1) - cartPara.angAccLim / cartPara.communFreq;
		middleVeli = middleVelNi;
		wheelsVeli = fromMiddleVel2WheelsVel2(middleVeli);  // vR, vL

		// send wheelsVeli
		float motorl_speed = wheelsVeli.y() * 60.0 / (2.0 * EIGEN_PI * cartPara.wheelsRadius);  // rpm
		float motorr_speed = wheelsVeli.x() * 60.0 / (2.0 * EIGEN_PI * cartPara.wheelsRadius);
		neal::logger(neal::LOG_INFO, "motorl_speed: " + std::to_string(motorl_speed));
		neal::logger(neal::LOG_INFO, "motorr_speed: " + std::to_string(motorr_speed));
		motor_1_drive_control.ctrlMotorMoveByVelocity(motorl_speed);  // vL, rpm
		motor_2_drive_control.ctrlMotorMoveByVelocity(motorr_speed);  // vR

		usleep(99*1000);  // lio 位姿更新频率 10Hz；sleep 不能输入小数
	}

	// stop test.
	motor_1_drive_control.ctrlMotorQuickStop();
	motor_2_drive_control.ctrlMotorQuickStop();

}

void MainWindow::button1_stopTest_slot() {
    // 此时界面已经卡死了
	// lioNode.exit();

    // /***CAN***/
    // motor_1_drive_control.ctrlMotorQuickStop();
	// motor_2_drive_control.ctrlMotorQuickStop();
}

void MainWindow::updateLoggingView1() {
    // ui.listView1->scrollToBottom();
}

/***cart model***/
Eigen::Vector3d MainWindow::fromWheelsVel2CartVel3(
	const Eigen::Vector3d& cartCurrPosIn, const Eigen::Vector2d& wheelsVelIn) const {
	
	Eigen::Vector3d cartVel3;  // (xDot, yDot, thetaDot).
	double v = (wheelsVelIn(0) + wheelsVelIn(1)) / 2;
	double w = (wheelsVelIn(0) - wheelsVelIn(1)) / cartPara.wheelsDistance;
	cartVel3 << cos(cartCurrPosIn(2)-cartPara.theta)*v - cartPara.a*sin(cartCurrPosIn(2)-cartPara.theta)*w - cartPara.b*cos(cartCurrPosIn(2)-cartPara.theta)*w,
				sin(cartCurrPosIn(2)-cartPara.theta)*v + cartPara.a*cos(cartCurrPosIn(2)-cartPara.theta)*w - cartPara.b*sin(cartCurrPosIn(2)-cartPara.theta)*w,
				w;
	return cartVel3;
}

Eigen::Vector2d MainWindow::fromCartVel2MiddleVel2(
	const double& xl_dot, const double& yl_dot, const double& theta) const {

	Eigen::Vector2d middleCurrVel;  // (v, w)
	middleCurrVel << (cos(theta-cartPara.theta)-cartPara.b/cartPara.a*sin(theta-cartPara.theta))*xl_dot + (sin(theta-cartPara.theta)+cartPara.b/cartPara.a*cos(theta-cartPara.theta))*yl_dot,
					  -sin(theta-cartPara.theta)/cartPara.a*xl_dot + cos(theta-cartPara.theta)/cartPara.a*yl_dot;
	return middleCurrVel;
}

Eigen::Vector2d MainWindow::fromMiddleVel2WheelsVel2(
	const Eigen::Vector2d& cartCurrVelIn) const {

	Eigen::Vector2d wheelsVelOut; // (vR, vL).
	wheelsVelOut << cartCurrVelIn(0) + cartCurrVelIn(1)*cartPara.wheelsDistance / 2,
		cartCurrVelIn(0) - cartCurrVelIn(1)*cartPara.wheelsDistance / 2;
	return wheelsVelOut;
}

Eigen::Vector3d MainWindow::fromGPose2CPose(
	const std::vector<double>& GPose) const {

	Eigen::Vector3d CPose;
	CPose << IMU_paths[pathId][0](0)+GPose[0]*cos(init_oris[pathId])-GPose[1]*sin(init_oris[pathId]),
			 IMU_paths[pathId][0](1)+GPose[0]*sin(init_oris[pathId])+GPose[1]*cos(init_oris[pathId]),
			 init_oris[pathId]+GPose[2];
	return CPose;
}

}  // namespace class1_ros_qt_demo

