#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.h"
#include "lio/qnode_lio.h"

/***CAN***/
// #include "simple_canopen_node.h"
// #include "sr_portable.h"
// #include "sr_socket.h"
// #include "sr_timer.h"
// #include "broker_msg.h"
// #include "thread/msg_thread.h"

namespace class1_ros_qt_demo {

/***CAN***/
// typedef U16 CtrlWord;
// const CtrlWord CW_SHUTDOWN = 0x0006;
// const CtrlWord CW_SWITCH_ON = 0x0007;
// const CtrlWord CW_ENABLE_OPERATION = 0x000F;
// const CtrlWord CW_DISABLE_VOLTAGE = 0x0000;
// const CtrlWord CW_QUICK_STOP = 0x0002;
// const CtrlWord CW_FAULT_RESET = 0x0080;

// typedef U16 StatusWord;
// const StatusWord SW_NOT_READY_TO_SWITCH_ON = 0x0000;
// const StatusWord SW_SWITCH_ON_DISABLED = 0x0040;
// const StatusWord SW_READY_TO_SWITCH_ON = 0x0021;
// const StatusWord SW_SWITCHED_ON = 0x0023;
// const StatusWord SW_OPERATION_ENABLED = 0x0027;
// const StatusWord SW_QUICK_STOP_ACTIVE = 0x0007;
// const StatusWord SW_FAULT_REACTION_ACTIVE = 0x000F;
// const StatusWord SW_FAULT = 0x0008;

// const U16 MASK_NOT_READY_TO_SWITCH_ON = 0x004F;
// const U16 MASK_SWITCH_ON_DISABLED = 0x004F;
// const U16 MASK_READY_TO_SWITCH_ON = 0x006F;
// const U16 MASK_SWITCHED_ON = 0x006F;
// const U16 MASK_OPERATION_ENABLED = 0x006F;
// const U16 MASK_QUICK_STOP_ACTIVE = 0x006F;
// const U16 MASK_FAULT_REACTION_ACTIVE = 0x004F;
// const U16 MASK_FAULT = 0x004F;

// typedef enum {
// 	NOT_READY_TO_SWITCH_ON,
// 	SWITCH_ON_DISABLED,
// 	READY_TO_SWITCH_ON,
// 	SWITCHED_ON,
// 	OPERATION_ENABLED,
// 	QUICK_STOP_ACTIVE,
// 	FAULT_REACTION_ACTIVE,
// 	FAULT,
// } MotorStatus;

// typedef struct {
// 	StatusWord status_word;
// 	U16 mask;
// 	MotorStatus status;
// 	char* status_name;
// } DriveStatusStr;

// typedef enum {
// 	MCE_STATUS_CHANGE,
// 	MCE_CTRL_MOTOR_VELOCITY_MODE,
// 	MSG_RECV_BOOT_UP,
// } MotorCtrlEvent;

// typedef struct {
// 	MotorCtrlEvent event;
// 	union {
// 		MotorStatus motor_status;
// 		F32 velocity;
// 	} obj;
// } MotorCtrlMsg;

// class MotorDriveControl : public SimpleCanopenNode, sr::utility::CMsgThread {
// public:
// 	MotorDriveControl(U8 node_id, U8 can_port, sr::utility::SR_TASK_ID devTaskId);
// 	void ctrlMotorMoveByVelocity(F32 velocity);
// 	void ctrlMotorQuickStop();
// 	MotorStatus currentStatus();
// 	F32 currentVelocity();
// 	void configPdo();
// private:
// 	void recvPdo(U16 pdo, U8* data);
// 	void recvEmergency(U8* data);
// 	void recvOthers(U16 id, U8* data);
// 	void recvReboot();
// 	void run();
// 	void sendFaultReset();
// 	void sendControlWord(U16 ctrl_val);
// 	void sendTargetVelocity(F32 velocity);
// 	U8 node_id;
// 	MotorStatus cur_status;
// 	F32 ctrl_target_velocity;
// };


class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

    // 关闭窗口时，默认调用
    void closeEvent(QCloseEvent *event);

public Q_SLOTS:
    void updateLoggingView2();
	void startLIO();
	void endLIO();

	/***CAN***/
	// void button2_w1s_slot();
	// void button2_w2s_slot();
	// void button2_canSend_slot();
	// void button2_canStop_slot();

private:
	/***CAN***/
	// static void refreshUITimerFun(void *);
	// sr::TimerId refresh_ui_timer;
	// MotorDriveControl motor_1_drive_control, motor_2_drive_control;

	int main_argc;
	char** main_argv;
	Ui::MainWindowDesign ui;
	QNode qnode;
	LIONode lioNode;
};

}  // namespace class1_ros_qt_demo

