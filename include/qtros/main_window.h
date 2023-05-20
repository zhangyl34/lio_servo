#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.h"
#include "lio/qnode_lio.h"

namespace class1_ros_qt_demo {

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
	void button2_w1s_slot();
	void button2_w2s_slot();
	void button2_canSend_slot();
	void button2_canStop_slot();

private:
	int main_argc;
	char** main_argv;
	Ui::MainWindowDesign ui;
	QNode qnode;
	LIONode lioNode;
};

}  // namespace class1_ros_qt_demo

