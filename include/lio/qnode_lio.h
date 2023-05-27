#pragma once

#include <string>
#include <QThread>
#include <QStringListModel>
#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <sstream>
#include <signal.h>
#include <iostream>
#include <vector>
#include <deque>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <condition_variable>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/impl/pcl_base.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <ikd_Tree.h>
#include <file_logger.h>

#include "lio/common_lib.h"
#include "lio/IMU_Processing.h"
#include "lio/preprocess.h"
#include "lio/use-ikfom.h"

namespace class1_ros_qt_demo {

class LIONode : public QThread {
Q_OBJECT

public:
	LIONode();
	virtual ~LIONode();
	bool init(int argc, char** argv);
	void run();

    /* 用于打 log*/
	enum LogLevel {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };
	QStringListModel* loggingModel() {return &logging_model;};
	void log( const LogLevel &level, const std::string &msg);
    void exit() {flg_exit = true;};

    /***用于闭环控制的访问接口***/
    std::vector<double> read3DPose();
    std::vector<double> read7DPose();
    M3D readRWG() const {return p_imu->get_R_W_G();};

Q_SIGNALS:
    void loggingUpdated();  // qt 信号
    void rosShutdown();

private:
    void pointBodyToWorld(PointType const * const pi, PointType * const po);
    void pointBodyToGround(PointType const * const pi, PointType * const po, const M3D& R_W_G);
    void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    bool sync_packages(MeasureGroup &meas);
    void map_incremental(const bool flg_EKF_inited);
    void publish_frame_world(const ros::Publisher & pubLaserCloudFull, const M3D& R_W_G);
    void h_share_model(state_ikfom &st, esekfom::dyn_share_datastruct<double> &ekfom_data);

    QStringListModel logging_model;

    float _LASER_POINT_COV, _INIT_TIME;
    bool scan_pub_en, pcd_save_en;
    bool dense_pub_en;
    double filter_size_map_min;
    double filter_size_surf_min;
    
    std::shared_ptr<Preprocess> p_pre = nullptr;
    std::mutex mtx_buffer;
    double last_timestamp_lidar;
    double last_timestamp_imu;
    std::deque<double> time_buffer;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

    bool flg_exit;
    PointCloudXYZI::Ptr feats_down_body{new PointCloudXYZI()};
    PointCloudXYZI::Ptr feats_down_world{new PointCloudXYZI()};
    std::vector<PointVector>  Nearest_Points;
    std::shared_ptr<KD_TREE<PointType>> ikdtree = nullptr;

    double lidar_end_time;
    PointCloudXYZI::Ptr feats_undistort{new PointCloudXYZI()};
    PointCloudXYZI::Ptr pcl_wait_save{new PointCloudXYZI()};
    state_ikfom state_point;

    std::vector<double> extrinT;
    std::vector<double> extrinR;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    int num_max_iterations;
    std::string lid_topic, imu_topic;

    double param_blind;
    int param_scans;
    int param_filters;
    int param_reflect;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    std::shared_ptr<ImuProcess> p_imu = nullptr;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;

    ros::Subscriber sub_pcl;
    ros::Subscriber sub_imu;
    ros::Publisher pubLaserCloudFull;

    /***用于闭环控制的访问接口***/
    // IMU 坐标系相对于 Ground 坐标系的位姿：G^xt, G^yt, G^orit
    void updatePose();
    std::mutex mtx_3DPose;
    std::vector<double> pose3D;
    std::mutex mtx_7DPose;
    std::vector<double> pose7D;
};

}  // namespace class1_ros_qt_demo