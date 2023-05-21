#include "qtros/main_window.h"

namespace class1_ros_qt_demo {

using namespace Qt;

/***CAN***/
// DriveStatusStr drive_status_list[] = {
// 	{ SW_NOT_READY_TO_SWITCH_ON, MASK_NOT_READY_TO_SWITCH_ON, NOT_READY_TO_SWITCH_ON, "NOT_READY_TO_SWITCH_ON"},
// 	{ SW_SWITCH_ON_DISABLED, MASK_SWITCH_ON_DISABLED, SWITCH_ON_DISABLED, "SWITCH_ON_DISABLED" },
// 	{ SW_READY_TO_SWITCH_ON, MASK_READY_TO_SWITCH_ON, READY_TO_SWITCH_ON, "READY_TO_SWITCH_ON" },
// 	{ SW_SWITCHED_ON, MASK_SWITCHED_ON, SWITCHED_ON, "SWICHED_ON" },
// 	{ SW_OPERATION_ENABLED, MASK_OPERATION_ENABLED, OPERATION_ENABLED, "OPERATION_ENABLED" },
// 	{ SW_QUICK_STOP_ACTIVE, MASK_QUICK_STOP_ACTIVE, QUICK_STOP_ACTIVE, "QUICK_STOP_ACTIVE" },
// 	{ SW_FAULT_REACTION_ACTIVE, MASK_FAULT_REACTION_ACTIVE, FAULT_REACTION_ACTIVE, "FAULT_REACTION_ACTIVE" },
// 	{ SW_FAULT, MASK_FAULT, FAULT, "FAULT" },
// };

// MotorDriveControl::MotorDriveControl(U8 node_id, U8 can_port, utility::SR_TASK_ID devTaskId) :
// 	SimpleCanopenNode(node_id, can_port), utility::CMsgThread(sr::HIGH, sr::SMALL, "Motor Drive", devTaskId, sizeof(MotorCtrlMsg)) {
// 	this->node_id = node_id;
// 	cur_status = NOT_READY_TO_SWITCH_ON;
// 	ctrl_target_velocity = 0.0F;
	
// 	this->start();  // 开启 run 线程，控制状态机的切换。

// 	resetNode();  // 0x81
// 	sleepms(100);
// }

// void MotorDriveControl::recvPdo(U16 pdo, U8* data) {
// 	static U16 status_word = 0U;
// 	if (pdo == 0x180) {
// 		status_word = data[0] + (data[1] << 8);
// 		//FILE_LOG("recv pdo:0x%x, 0x%x", node_id, status_word);
// 		for (int i = 0; i < sizeof(drive_status_list) / sizeof(drive_status_list[0]); i++) {
// 			if ((status_word & drive_status_list[i].mask) == drive_status_list[i].status_word) {
// 				//cur_status = drive_status_list[i].status;
// 				MotorCtrlMsg msg;
// 				msg.event = MCE_STATUS_CHANGE;
// 				msg.obj.motor_status = drive_status_list[i].status;
// 				sendItcMsg((S8*)&msg, sizeof(msg));
// 				break;
// 			}
// 		}
// 	}
// }

// void MotorDriveControl::recvEmergency(U8* data) {
// 	FILE_LOG("recv emergency msg, node id=%d, :0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x!", node_id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
// }

// void MotorDriveControl::recvOthers(U16 id, U8* data) {
// 	//FILE_LOG("recv other id=0x%d, data:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x!", id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
// }

// void MotorDriveControl::recvReboot() {
// 	FILE_LOG("recv boot up msg, node id=%d", node_id);
// 	MotorCtrlMsg msg;
// 	msg.event = MSG_RECV_BOOT_UP;
// 	sendItcMsg((S8*)&msg, sizeof(msg));
// }

// void MotorDriveControl::ctrlMotorMoveByVelocity(F32 velocity) {
// 	MotorCtrlMsg msg;
// 	msg.event = MCE_CTRL_MOTOR_VELOCITY_MODE;
// 	msg.obj.velocity = velocity;
// 	bool rc= sendItcMsg((S8*)&msg, sizeof(msg));  // whether send the message to the queue successfully.
// 	//neal::logger(LOG_INFO, "to qu : " + std::to_string(rc));
// }

// void MotorDriveControl::ctrlMotorQuickStop() {
// 	sendControlWord(CW_QUICK_STOP);
// }

// MotorStatus MotorDriveControl::currentStatus() {
// 	return cur_status;
// }

// // get current rpm. can't be used directly. OD should only be visited in MotorDriveControl::run() thread.
// F32 MotorDriveControl::currentVelocity() {

// 	S8 sdo_buffer[4] = { 0 };
// 	if (SR_FALSE == readOD(sdo_buffer, 4, 0x606C, 0)) {
// 		FILE_LOG("read od 0x606C fail");
// 	}
// 	//U16 t_v_decimal = sdo_buffer[0] + (sdo_buffer[1] << 8);
// 	S16 t_v_integer = sdo_buffer[2] + (sdo_buffer[3] << 8);
// 	F32 t_v = (F32)t_v_integer;
// 	//F32 velocity = t_v;
// 	F32 velocity = (t_v * 60.0 * 1000.0) / (32.0 * 4.0 * 53.0);
// 	neal::logger(LOG_WARN, "Do_not_read_velocity!");
// 	return velocity;
// }

// void MotorDriveControl::configPdo() {
// 	S8 data[4] = { 0xFE };
// 	writeOD(data, 1, 0x1800, 0x2);  // 异步
// 	sleepms(100);
// 	data[0] = 0xE8;  // 1000ms
// 	data[1] = 0x3;
// 	writeOD(data, 2, 0x1800, 0x5);  // 事件定时器触发时间
// 	sleepms(100);
// 	data[0] = 0x10;
// 	data[1] = 0;
// 	data[2] = 0;
// 	data[3] = 0;
// 	writeOD(data, 4, 0x6065, 0x0);
// 	//S16 send_val = 0;  // right wheels, +
// 	//if (this->node_id == 0x1C)  // left wheels, -
// 	//{
// 	//	send_val = 64;  // means: -
// 	//}
// 	//FILE_LOG("polarity:%d", send_val);
// 	//S8 send_polarity[8] = { send_val };  // 64 means negative polarity.
// 	//writeOD(send_polarity, 1, 0x607E, 0);
// 	//sleepms(200);

// 	if (SR_FALSE == setNodeRun()) {
// 		FILE_LOG("set node %d run status error!", node_id);
// 	}
// }

// void MotorDriveControl::run() {
// 	MotorCtrlMsg buf;
// 	while (true) {
// 		if (0U != this->recvItcMsg(&buf, sizeof(buf), sr::NO_TIME_OUT)) {
// 			//neal::logger(LOG_TEST, "into can start."+std::to_string(buf.event));
// 			if (buf.event == MCE_STATUS_CHANGE) {
// 				//FILE_LOG("change status:cur status node id=%d, =%d, %d!\n", node_id, buf.obj.motor_status, cur_status);
// 				if ((buf.obj.motor_status == READY_TO_SWITCH_ON) && (cur_status == SWITCH_ON_DISABLED)) {
// 					sendControlWord(CW_SWITCH_ON);
// 				}
// 				else if ((buf.obj.motor_status == SWITCHED_ON) && (cur_status == READY_TO_SWITCH_ON)) {
// 					sendControlWord(CW_ENABLE_OPERATION);
// 				}
// 				else if ((buf.obj.motor_status == OPERATION_ENABLED) && (cur_status == SWITCHED_ON)) {
// 					S8 sdo_data[] = { 0x3 };
// 					writeOD(sdo_data, 1, 0x6060, 0);
// 					sendTargetVelocity(ctrl_target_velocity);
// 				}
// 				else if (buf.obj.motor_status == SW_FAULT) {
// 					sendFaultReset();
// 				}
// 				cur_status = buf.obj.motor_status;
// 			}
// 			else if (buf.event == MCE_CTRL_MOTOR_VELOCITY_MODE) {
// 				ctrl_target_velocity = buf.obj.velocity;
// 				if (cur_status == SWITCH_ON_DISABLED) {
// 					sendTargetVelocity(0.0);
// 					sendControlWord(CW_SHUTDOWN);
// 				}
// 				else if (cur_status == OPERATION_ENABLED) {
// 					sendTargetVelocity(ctrl_target_velocity);
// 				}
// 				else {
// 					neal::logger(LOG_WARN, "the_state_cannot_send_speed: " + std::to_string(cur_status));
// 					FILE_LOG("the state cannot send speed:%d!", cur_status);
// 				}
				
// 			}
// 			else if (buf.event == MSG_RECV_BOOT_UP) {
// 				configPdo();
// 			}
// 			//neal::logger(LOG_TEST, "into can end."+std::to_string(buf.event));
// 		}
// 	}
// }

// void MotorDriveControl::sendFaultReset() {
// 	sendControlWord(CW_FAULT_RESET);
// 	sleepms(2);
// 	sendControlWord(CW_DISABLE_VOLTAGE);
// }

// void MotorDriveControl::sendControlWord(U16 ctrl_val) {
// 	U8 send_data[2] = { ctrl_val, ctrl_val >> 8};
// 	sendTpdo(0x200, send_data, 2);
// }

// // 输入单位 rpm，先转换为 count/ms，再发送
// void MotorDriveControl::sendTargetVelocity(F32 velocity) {
// 	S8 send_data[8] = { 0 };
// 	F32 t_v = fabs((velocity * 500.0 * 4.0 * 66.666667) / (60.0 * 1000.0));  // count per millisecond.
// 	S16 t_v_integer = trunc(t_v);
// 	U16 t_v_decimal = (t_v - t_v_integer) * 65535.0;

// 	S16 send_val = 0;  // right wheels, +
// 	if (velocity < 0) {  // left wheels, -
// 		send_val = 64;  // means: -
// 	}
// 	//FILE_LOG("polarity:%d", send_val);
// 	S8 send_polarity[8] = { send_val };  // 64 means negative polarity.
// 	//FILE_LOG("ver:%f, %d, %d\n", t_v, t_v_integer, t_v_decimal);
// 	send_data[0] = t_v_decimal;
// 	send_data[1] = t_v_decimal >> 8;
// 	send_data[2] = t_v_integer;
// 	send_data[3] = t_v_integer >> 8;
// 	//FILE_LOG("send val:0x%x, 0x%x, 0x%x, 0x%x\n", send_data[0], send_data[1], send_data[2], send_data[3]);
// 	writeOD(send_polarity, 1, 0x607E, 0);
// 	bool rc2 = writeOD(send_data, 4, 0x60FF, 0);
// 	//neal::logger(LOG_WARN, "sendTargetVelocity: " + std::to_string(rc1)+ " "+std::to_string(rc2));
// }


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent), main_argc(argc), main_argv(argv)
    // motor_1_drive_control(0x1D, 0, 6), motor_2_drive_control(0x1C, 0, 7)
{
    // 设置 .ui 文件
    ui.setupUi(this);
    // qApp 是 QApplication 对象的全局指针
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    // 显示页
    ui.tab_manager->setCurrentIndex(0);

    // 更新 log
    ui.listView2->setModel(lioNode.loggingModel());
    QObject::connect(&lioNode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView2()));

    // LIO
    QObject::connect(ui.button2_rosStart, SIGNAL(released()), this, SLOT(startLIO()));
    QObject::connect(ui.button2_rosEnd, SIGNAL(released()), this, SLOT(endLIO()));

    /***CAN***/
    // QObject::connect(ui.button2_w1s, SIGNAL(released()), this, SLOT(button2_w1s_slot()));
	// QObject::connect(ui.button2_w2s, SIGNAL(released()), this, SLOT(button2_w2s_slot()));
	// QObject::connect(ui.button2_canSend, SIGNAL(released()), this, SLOT(button2_canSend_slot()));
	// QObject::connect(ui.button2_canStop, SIGNAL(released()), this, SLOT(button2_canStop_slot()));

    // refresh_ui_timer = sr::createTimer(TIME_UNIT_MS, 500, refreshUITimerFun, this, SR_TRUE);
	// sr::startTimer(refresh_ui_timer);

    /***Close-loop control***/
    timer_path = new QTimer(this);
    QObject::connect(timer_path, SIGNAL(timeout()), this, SLOT(updatePath()));
    QObject::connect(ui.button1_readPath, SIGNAL(released()), this, SLOT(button1_readPath_slot()));
    QObject::connect(ui.button1_startTest, SIGNAL(released()), this, SLOT(button1_startTest_slot()));
    QObject::connect(ui.button1_stopTest, SIGNAL(released()), this, SLOT(button1_stopTest_slot()));
}

MainWindow::~MainWindow() {
    /***CAN***/
    // motor_1_drive_control.ctrlMotorQuickStop();
	// motor_2_drive_control.ctrlMotorQuickStop();
}

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

/***CAN***/
// void MainWindow::button2_w1s_slot() {
//     motor_1_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit2_1->text().toFloat());
// }

// void MainWindow::button2_w2s_slot() {
//     motor_2_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit2_2->text().toFloat());
// }

// void MainWindow::button2_canSend_slot() {
//     motor_1_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit2_1->text().toFloat());
// 	motor_2_drive_control.ctrlMotorMoveByVelocity(ui.lineEdit2_2->text().toFloat());
// }

// void MainWindow::button2_canStop_slot() {
// 	motor_1_drive_control.ctrlMotorQuickStop();
// 	motor_2_drive_control.ctrlMotorQuickStop();
// }

// // 更新电机状态与电机速度
// void MainWindow::refreshUITimerFun(void* param) {
// 	MainWindow* mw = (MainWindow*)param;
//     QString status;

// 	MotorStatus motor_status = mw->motor_1_drive_control.currentStatus();
// 	for (int i = 0; i < sizeof(drive_status_list) / sizeof(drive_status_list[0]); i++) {
// 		if (drive_status_list[i].status == motor_status) {
// 			status = drive_status_list[i].status_name;
// 			break;
// 		}
// 	}
// 	mw->ui.label2_5->setText(status);
	
// 	motor_status = mw->motor_2_drive_control.currentStatus();
// 	for (int i = 0; i < sizeof(drive_status_list) / sizeof(drive_status_list[0]); i++) {
// 		if (drive_status_list[i].status == motor_status) {
// 			status = drive_status_list[i].status_name;
// 			break;
// 		}
// 	}
// 	mw->ui.label2_11->setText(status);

// 	mw->ui.label2_7->setText(QString::number(mw->motor_1_drive_control.currentVelocity()));
// 	mw->ui.label2_13->setText(QString::number(mw->motor_2_drive_control.currentVelocity()));
// }

/***Close-loop control***/
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

void MainWindow::button1_readPath_slot() {
	pathIn.clear();

    // read path

    // draw path
    cv::Scalar color2(17, 204, 254);
    int halfWidth = 150;
	int height = 350;
	cv::Mat pathImg = cv::Mat(cv::Size(halfWidth*2,height), CV_8UC3, cv::Scalar(0,0,0));
    for (int i=0; i<pathIn.size(); i++) {
        cv::circle(pathImg, cv::Point(halfWidth + pathIn.at(i)[0] / 10.0, height - pathIn.at(i)[1] / 10.0), 2, color2, -1);
    }
	QImage srcQImage = QImage((uchar*)(pathImg.data), pathImg.cols, pathImg.rows, pathImg.step, QImage::Format_RGB888);
	ui.label1_3->setPixmap(QPixmap::fromImage(srcQImage));
	ui.label1_3->resize(srcQImage.size());
	ui.label1_3->show();
}

void MainWindow::button1_startTest_slot() {
    timer_path->start(33);

}

void MainWindow::button1_stopTest_slot() {
    timer_path->stop();

}

}  // namespace class1_ros_qt_demo

