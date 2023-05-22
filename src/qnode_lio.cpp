#include "lio/qnode_lio.h"

namespace class1_ros_qt_demo {

LIONode::LIONode() :
	_LASER_POINT_COV(0.001), _INIT_TIME(0.1),
    scan_pub_en(true), pcd_save_en(true), dense_pub_en(true), flg_exit(false),
    filter_size_map_min(0.05), filter_size_surf_min(0.5),
    last_timestamp_lidar(0.0), last_timestamp_imu(-1.0), lidar_end_time(0.0), 
    gyr_cov(0.5), acc_cov(0.5), b_gyr_cov(0.0005), b_acc_cov(0.0005), num_max_iterations(4),
    lid_topic("/livox/lidar"), imu_topic("/imu"),
    param_blind(0.5), param_scans(1), param_filters(2), param_reflect(10) {

    p_pre.reset(new Preprocess());
    p_imu.reset(new ImuProcess());
    ikdtree.reset(new KD_TREE<PointType>());
    extrinT = {0.0078, 0.13, 0.0509};
    extrinR = {1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0};
    p_pre->set_blind(param_blind);
    p_pre->set_N_SCANS(param_scans);
    p_pre->set_point_filter_num(param_filters);
    p_pre->set_reflect_thresh(param_reflect);

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    V3D Lidar_T_wrt_IMU(V3D(0.0,0.0,0.0));
    M3D Lidar_R_wrt_IMU(M3D::Identity());
    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw,
        [this](state_ikfom &st, esekfom::dyn_share_datastruct<double> &ekfom_data) { h_share_model(st, ekfom_data); },
        num_max_iterations, epsi);

    pose3D.resize(3);
}

LIONode::~LIONode() {
    if(ros::isStarted()) {
        ros::shutdown();  // explicitly needed since we use ros::start()
        ros::waitForShutdown();
    }
	wait();
}

bool LIONode::init(int argc, char** argv) {
    ros::init(argc, argv, "laserMapping");
    if (!ros::master::check()) {  // 检查 master 节点是否启动
        std::cout << "Master node is not activated..." << std::endl;
		return false;
	}
    ros::start();  // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle nh;
	sub_pcl = nh.subscribe<livox_ros_driver::CustomMsg>(lid_topic, 200000, [this](const livox_ros_driver::CustomMsg::ConstPtr &msg) { livox_pcl_cbk(msg); });
    sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200000, [this](const sensor_msgs::Imu::ConstPtr &msg) { imu_cbk(msg); });
    pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    log(Info, std::string("***LIO START***"));

    start();  // 启动 QThread，执行 run() 函数
	return true;
}

void LIONode::run() {
	ros::Rate rate(5000);
    bool status = ros::ok();
    bool flg_first_scan = true;
    double first_lidar_time = 0.0;
    MeasureGroup measures;
    while (status) {
        if (flg_exit) {  // 有中断产生
            break;
        }
        ros::spinOnce();
        if(!sync_packages(measures)) {
            status = ros::ok();
            rate.sleep();
            continue;
        }
        if (flg_first_scan) {
            first_lidar_time = measures.lidar_beg_time;
            flg_first_scan = false;
            continue;
        }
        p_imu->Process(measures, kf, feats_undistort);
        if (feats_undistort->empty() || (feats_undistort == NULL)) {
            log(Warn, std::string("No point, skip this scan!(1)"));
            continue;
        }
        state_point = kf.get_x();

        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        int feats_down_size = feats_down_body->points.size();
        if (feats_down_size <= 5) {
            log(Warn, std::string("No point, skip this scan!(2)"));
            continue;
        }

        if(ikdtree->Root_Node == nullptr) {
            ikdtree->set_downsample_param(filter_size_map_min);
            feats_down_world->resize(feats_down_size);
            for(int i = 0; i < feats_down_size; i++) {
                pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            }
            ikdtree->Build(feats_down_world->points);
            log(Info, std::string("ikd-Tree initialized!"));
            continue;
        }
        log(Info, "receiving Livox message.");
        feats_down_world->resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);
        kf.update_iterated_dyn_share_modified(_LASER_POINT_COV);

        state_point = kf.get_x();
        update3DPose();

        bool flg_EKF_inited = (measures.lidar_beg_time - first_lidar_time) < _INIT_TIME ? false : true;
        map_incremental(flg_EKF_inited);

        if (scan_pub_en || pcd_save_en) {
            publish_frame_world(pubLaserCloudFull, p_imu->get_R_W_G());
        }

        status = ros::ok();
        rate.sleep();
    }

    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        V3D gravity = p_imu->get_mean_acc();
        std::string strout;
        strout = "gravity: x " + std::to_string(gravity(0)) +
            ", y " + std::to_string(gravity(1)) + ", z " + std::to_string(gravity(2));
        log(Info, strout);
        std::string file_name = std::string("scans.ply");
        std::string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PLYWriter writer;
        log(Info, "current scan saved to /PCD/"+file_name);
        writer.write(all_points_dir, *pcl_wait_save);
    }
    log(Info, "***LIO END***");
	Q_EMIT rosShutdown();
}

void LIONode::log(const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
    switch (level) {
		case(Debug) : {
            ROS_DEBUG_STREAM(msg);
            logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
            break;
		}
		case(Info) : {
            ROS_INFO_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
		}
		case(Warn) : {
            ROS_WARN_STREAM(msg);
            logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
            break;
		}
		case(Error) : {
            ROS_ERROR_STREAM(msg);
            logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
            break;
		}
		case(Fatal) : {
            ROS_FATAL_STREAM(msg);
            logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
            break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

void LIONode::pointBodyToWorld(PointType const * const pi, PointType * const po) {

    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LIONode::pointBodyToGround(PointType const * const pi, PointType * const po, const M3D& R_W_G) {
    
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_ground(R_W_G * (state_point.rot * (state_point.offset_R_L_I*p_body+state_point.offset_T_L_I)
        + state_point.pos));

    po->x = p_ground(0);
    po->y = p_ground(1);
    po->z = p_ground(2);
    po->intensity = pi->intensity;
}

void LIONode::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    std::unique_lock<std::mutex> locker(mtx_buffer, std::defer_lock);

    locker.lock();
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        neal::logger(neal::LOG_ERROR, "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    
    if (fabs(last_timestamp_lidar - last_timestamp_imu) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty()) {
        std::string strout;
        strout = "IMU and LiDAR not Synced, IMU time: " + std::to_string(last_timestamp_imu) +
            ", lidar scan end time: " + std::to_string(last_timestamp_lidar) + '.';
        neal::logger(neal::LOG_ERROR, strout);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    locker.unlock();
}

void LIONode::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
    
    std::unique_lock<std::mutex> locker(mtx_buffer, std::defer_lock);
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    double timestamp = msg->header.stamp.toSec();
    locker.lock();
    if (timestamp < last_timestamp_imu) {
        std::string strout = "imu loop back, clear buffer";
        neal::logger(neal::LOG_ERROR, strout);
        imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    locker.unlock();
}

bool LIONode::sync_packages(MeasureGroup &meas) {

    static int scan_num = 0;
    static double lidar_mean_scantime = 0.0;
    static bool lidar_pushed = false;

    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    if(!lidar_pushed) {
        meas.lidar = lidar_buffer.front();
        lidar_buffer.pop_front();
        meas.lidar_beg_time = time_buffer.front();
        time_buffer.pop_front();
        double duration = static_cast<double>(meas.lidar->points.back().curvature) / 1000.0;
        if (meas.lidar->points.size() <= 1) {
            lidar_end_time = meas.lidar_beg_time + 0.0;
            neal::logger(neal::LOG_WARN, "Too few input point cloud!");
        }
        else if (duration < 0.5 * lidar_mean_scantime) {
            lidar_end_time = meas.lidar_beg_time + duration;
            neal::logger(neal::LOG_WARN, "Too short scan time!");
        }
        else {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + duration;
            lidar_mean_scantime += (duration - lidar_mean_scantime) / scan_num;
        }
        meas.lidar_end_time = lidar_end_time;
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time) {
        return false;
    }

    meas.imu.clear();
    while ((!imu_buffer.empty())) {
        double imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time) {
            break;
        }
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }
    lidar_pushed = false;
    return true;
}

void LIONode::map_incremental(const bool flg_EKF_inited) {

    int feats_down_size = feats_down_body->points.size();

    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++) {
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        if (!Nearest_Points[i].empty() && flg_EKF_inited) {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++) {
                if (points_near.size() < NUM_MATCH_POINTS) {
                    break;
                }
                if (calc_dist(points_near[readd_i], mid_point) < dist) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) {
                PointToAdd.push_back(feats_down_world->points[i]);
            }
        }
        else {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }
    ikdtree->Add_Points(PointToAdd, true);
    ikdtree->Add_Points(PointNoNeedDownsample, false); 
}

void LIONode::publish_frame_world(const ros::Publisher & pubLaserCloudFull, const M3D& R_W_G) {
    
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();

    if(scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++) {
            pointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
    }

    if (pcd_save_en) {
        PointCloudXYZI::Ptr laserCloudGround(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++) {
            pointBodyToGround(&laserCloudFullRes->points[i], &laserCloudGround->points[i], R_W_G);
        }
        *pcl_wait_save += *laserCloudGround;
    }
}

void LIONode::h_share_model(state_ikfom &st, esekfom::dyn_share_datastruct<double> &ekfom_data) {

    int feats_down_size = feats_down_body->points.size();
 
    PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI());
    laserCloudOri->resize(feats_down_size);
    PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI());
    corr_normvect->resize(feats_down_size);
    std::vector<bool> point_selected_surf;
    point_selected_surf.resize(feats_down_size);
    PointCloudXYZI::Ptr normvec(new PointCloudXYZI());
    normvec->resize(feats_down_size);

    for (int i = 0; i < feats_down_size; i++) {
        const PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(st.rot * (st.offset_R_L_I * p_body + st.offset_T_L_I) + st.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        PointVector &points_near = Nearest_Points[i];
        ikdtree->Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);

        point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false :
            (pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5.0 ? false: true);
        if (!point_selected_surf[i]) {
            continue;
        }

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f)) {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
            if (s > 0.9) {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
            }
        }
    }

    int effct_feat_num = 0;
    for (int i = 0; i < feats_down_size; i++) {
        if (point_selected_surf[i]) {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1) {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        neal::logger(neal::LOG_WARN, "No Effective Points!");
        return;
    }

    ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);
    for (int i = 0; i < effct_feat_num; i++) {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = st.offset_R_L_I * point_this_be + st.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
        V3D C(st.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        V3D B(point_be_crossmat * st.offset_R_L_I.conjugate() * C);
        ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        ekfom_data.h(i) = -norm_p.intensity;
    }
}

void LIONode::update3DPose() {
    M3D R_W_G(p_imu->get_R_W_G());  // Ground^R_World
    V3D IMU2DPose(R_W_G*state_point.pos);  // 检查一下，这步转换有必要吗？
    V3D x(1,0,0);
    V3D IMU1DPose(R_W_G*state_point.rot*x);

    std::unique_lock<std::mutex> locker(mtx_3DPose, std::defer_lock);
    locker.lock();
    pose3D[0] = IMU2DPose(0);
    pose3D[1] = IMU2DPose(1);
    pose3D[2] = atan2(IMU1DPose(1),IMU1DPose(0));
    locker.unlock();
}

std::vector<double> LIONode::read3DPose() {
    std::unique_lock<std::mutex> locker(mtx_3DPose, std::defer_lock);
    locker.lock();
    std::vector<double> out = pose3D;
    locker.unlock();
    return out;
}

}  // namespace class1_ros_qt_demo
