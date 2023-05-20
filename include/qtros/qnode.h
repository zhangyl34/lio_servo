#pragma once

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

namespace class1_ros_qt_demo {

class QNode : public QThread {
Q_OBJECT

public:
	QNode();
	virtual ~QNode();
	bool init(int argc, char** argv);
	void run();

	enum LogLevel {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
    void loggingUpdated();  // qt 信号
    void rosShutdown();

private:

	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
};

}  // namespace class1_ros_qt_demo

