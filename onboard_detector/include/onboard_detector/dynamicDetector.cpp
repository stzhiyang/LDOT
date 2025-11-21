/*
    FILE: dynamicDetector.cpp
    ---------------------------------
    function implementation of dynamic osbtacle detector
*/
#include <onboard_detector/dynamicDetector.h>

namespace onboardDetector{
    // 默认构造函数
    dynamicDetector::dynamicDetector(){
        this->ns_ = "onboard_detector";
        this->hint_ = "[onboardDetector]";
    }

    // 带节点句柄的构造函数
    dynamicDetector::dynamicDetector(const ros::NodeHandle& nh){
        this->ns_ = "onboard_detector";
        this->hint_ = "[onboardDetector]";
        this->nh_ = nh;
        this->initParam();
        this->registerPub();
        this->registerCallback();
    }

    // 初始化检测器
    void dynamicDetector::initDetector(const ros::NodeHandle& nh){
        this->nh_ = nh;
        this->initParam();
        this->registerPub();
        this->registerCallback();
    }

    // 初始化参数
    void dynamicDetector::initParam(){
        // ---------------------------------获取ros话题---------------------------------------
        // localization mode
        if (not this->nh_.getParam(this->ns_ + "/localization_mode", this->localizationMode_)){
            this->localizationMode_ = 0;
            cout << this->hint_ << ": No localization mode option. Use default: pose" << endl;
        }
        else{
            cout << this->hint_ << ": Localizaiton mode: pose (0)/odom (1). Your option: " << this->localizationMode_ << endl;
        }   

        // use livox custom msg
        if (not this->nh_.getParam(this->ns_ + "/use_livox_custom_msg", this->useLivoxCustomMsg_)){
            this->useLivoxCustomMsg_ = false;
            cout << this->hint_ << ": No use_livox_custom_msg parameter. Use default: false (PointCloud2)" << endl;
        }
        else{
            cout << this->hint_ << ": Use Livox CustomMsg: " << (this->useLivoxCustomMsg_ ? "true" : "false") << endl;
        }

        // lidar topic name
        if (not this->nh_.getParam(this->ns_ + "/lidar_pointcloud_topic", this->lidarTopicName_)){
            this->lidarTopicName_ = "/cloud_registered";
            cout << this->hint_ << ": No lidar pointcloud topic name. Use default: /cloud_registered" << endl;
        }
        else{
            cout << this->hint_ << ": Lidar pointcloud topic: " << this->lidarTopicName_ << endl;
        }

        if (this->localizationMode_ == 0){
            // odom topic name
            if (not this->nh_.getParam(this->ns_ + "/pose_topic", this->poseTopicName_)){
                this->poseTopicName_ = "/CERLAB/quadcopter/pose";
                cout << this->hint_ << ": No pose topic name. Use default: /CERLAB/quadcopter/pose" << endl;
            }
            else{
                cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << endl;
            }           
        }

        if (this->localizationMode_ == 1){
            // pose topic name
            if (not this->nh_.getParam(this->ns_ + "/odom_topic", this->odomTopicName_)){
                this->odomTopicName_ = "/CERLAB/quadcopter/odom";
                cout << this->hint_ << ": No odom topic name. Use default: /CERLAB/quadcopter/odom" << endl;
            }
            else{
                cout << this->hint_ << ": Odom topic: " << this->odomTopicName_ << endl;
            }
        }

        // --------------------------------------坐标系转换参数（外参）--------------------------------------------
        // transform matrix: body to lidar
        std::vector<double> body2LidarVec (16);
        if (not this->nh_.getParam(this->ns_ + "/body_to_lidar", body2LidarVec)){
            ROS_ERROR("[dynamicDetector]: Please check body to lidar matrix!");
        }
        else{
            for (int i=0; i<4; ++i){
                for (int j=0; j<4; ++j){
                    this->body2Lidar_(i, j) = body2LidarVec[i * 4 + j];
                }
            }
        }

        // --------------------------------------系统运行的频率（时间步长）-----------------------------------------
        if (not this->nh_.getParam(this->ns_ + "/time_step", this->dt_)){
            this->dt_ = 0.033;
            std::cout << this->hint_ << ": No time step parameter found. Use default: 0.033." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Time step for the system is set to: " << this->dt_ << std::endl;
        }  

        // --------------------------------------DBSCAN通用参数--------------------------------------------------
        // 地面高度
        if (not this->nh_.getParam(this->ns_ + "/ground_height", this->groundHeight_)){
            this->groundHeight_ = 0.1;
            std::cout << this->hint_ << ": No ground height parameter. Use default: 0.1m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Ground height is set to: " << this->groundHeight_ << std::endl;
        }

        // roof height  天花板高度
        if (not this->nh_.getParam(this->ns_ + "/roof_height", this->roofHeight_)){
            this->roofHeight_ = 2.0;
            std::cout << this->hint_ << ": No roof height parameter. Use default: 2.0m." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Roof height is set to: " << this->roofHeight_ << std::endl;
        }

        // lidar detection range
        std::vector<double> detectionRange;
        if (not this->nh_.getParam(this->ns_ + "/lidar_detection_range", detectionRange)){
            this->localLidarRange_ = Eigen::Vector3d(10.0, 10.0, 3.0);
            std::cout << this->hint_ << ": No lidar detection range parameter. Use default: [10.0, 10.0, 3.0]." << std::endl;
        }
        else{
            if (detectionRange.size() == 3){
                this->localLidarRange_ = Eigen::Vector3d(detectionRange[0], detectionRange[1], detectionRange[2]);
                std::cout << this->hint_ << ": Lidar detection range is set to: [" << this->localLidarRange_.x() << ", " 
                     << this->localLidarRange_.y() << ", " << this->localLidarRange_.z() << "]" << std::endl;
            }
            else{
                this->localLidarRange_ = Eigen::Vector3d(10.0, 10.0, 3.0);
                std::cout << this->hint_ << ": Invalid lidar detection range size. Use default: [10.0, 10.0, 3.0]." << std::endl;
            }
        }

        // --------------------------------------激光DBSCAN聚类参数---------------------------------------------------------
        // lidar dbscan min points
        if (not this->nh_.getParam(this->ns_ + "/lidar_DBSCAN_min_points", this->lidarDBMinPoints_)){
            this->lidarDBMinPoints_ = 10;
            cout << this->hint_ << ": No lidar DBSCAN minimum point in each cluster parameter. Use default: 10." << endl;
        }
        else{
            cout << this->hint_ << ": Lidar DBSCAN Minimum point in each cluster is set to: " << this->lidarDBMinPoints_ << endl;
        }

        // lidar dbscan search range
        if (not this->nh_.getParam(this->ns_ + "/lidar_DBSCAN_epsilon", this->lidarDBEpsilon_)){
            this->lidarDBEpsilon_ = 0.2;
            cout << this->hint_ << ": No lidar DBSCAN epsilon parameter. Use default: 0.5." << endl;
        }
        else{
            cout << this->hint_ << ": Lidar DBSCAN epsilon is set to: " << this->lidarDBEpsilon_ << endl;
        }

        // lidar dbscan use adaptive epsilon
        if (not this->nh_.getParam(this->ns_ + "/lidar_DBSCAN_use_adaptive", this->lidarDBUseAdaptive_)){
            this->lidarDBUseAdaptive_ = false;
            cout << this->hint_ << ": No lidar DBSCAN use adaptive parameter. Use default: false." << endl;
        }
        else{
            cout << this->hint_ << ": Lidar DBSCAN use adaptive is set to: " << (this->lidarDBUseAdaptive_ ? "true" : "false") << endl;
        }

        // lidar dbscan distance scale for adaptive epsilon
        if (not this->nh_.getParam(this->ns_ + "/lidar_DBSCAN_distance_scale", this->lidarDBDistanceScale_)){
            this->lidarDBDistanceScale_ = 0.05;
            cout << this->hint_ << ": No lidar DBSCAN distance scale parameter. Use default: 0.05." << endl;
        }
        else{
            cout << this->hint_ << ": Lidar DBSCAN distance scale is set to: " << this->lidarDBDistanceScale_ << endl;
        }
        
        // lidar points downsample threshold
        if(not this->nh_.getParam(this->ns_ + "/downsample_threshold", this->downSampleThresh_)){
            this->downSampleThresh_ = 4000;
            cout << this->hint_ << ": No downsample threshold parameter found. Use default: 4000." << endl;
        }
        else{
            cout << this->hint_ << ": Downsample threshold is set to: " << this->downSampleThresh_ << endl;
        }

        // gaussian downsample rate
        if (not this->nh_.getParam(this->ns_ + "/gaussian_downsample_rate", this->gaussianDownSampleRate_)){
            this->gaussianDownSampleRate_ = 2;
            std::cout << this->hint_ << ": No gaussian downsample rate parameter found. Use default: 2." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Gaussian downsample rate is set to: " << this->gaussianDownSampleRate_ << std::endl;
        }

        // -------------------------------------------目标跟踪与数据关联参数--------------------------------------------------
        // 数据关联门限阈值（基于卡方分布）
        if (not this->nh_.getParam(this->ns_ + "/association_gate_threshold", this->associationGateThresh_)){
            this->associationGateThresh_ = 9.21; // 自由度为3的卡方分布，置信度99%
            cout << this->hint_ << ": No association gate threshold parameter found. Use default: 9.21." << endl;
        }
        else{
            cout << this->hint_ << ": Association gate threshold is set to: " << this->associationGateThresh_ << "." << endl;
        }

        // 位置代价权重
        if (not this->nh_.getParam(this->ns_ + "/association_pos_cost_weight", this->associationPosCostWeight_)){
            this->associationPosCostWeight_ = 1.0;
            cout << this->hint_ << ": No position cost weight parameter found. Use default: 1.0." << endl;
        }
        else{
            cout << this->hint_ << ": Position cost weight is set to: " << this->associationPosCostWeight_ << "." << endl;
        }

        // 尺寸代价权重
        if (not this->nh_.getParam(this->ns_ + "/association_size_cost_weight", this->associationSizeCostWeight_)){
            this->associationSizeCostWeight_ = 0.5;
            cout << this->hint_ << ": No size cost weight parameter found. Use default: 0.5." << endl;
        }
        else{
            cout << this->hint_ << ": Size cost weight is set to: " << this->associationSizeCostWeight_ << "." << endl;
        }

        // 点云标准差代价权重
        if (not this->nh_.getParam(this->ns_ + "/association_std_cost_weight", this->associationStdCostWeight_)){
            this->associationStdCostWeight_ = 0.3;
            cout << this->hint_ << ": No std cost weight parameter found. Use default: 0.3." << endl;
        }
        else{
            cout << this->hint_ << ": Std cost weight is set to: " << this->associationStdCostWeight_ << "." << endl;
        }

        // tracking history size
        if (not this->nh_.getParam(this->ns_ + "/history_size", this->histSize_)){
            this->histSize_ = 5;
            std::cout << this->hint_ << ": No tracking history size parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": History for tracking is set to: " << this->histSize_ << std::endl;
        }  

        // history threshold for fixing box size
        if (not this->nh_.getParam(this->ns_ + "/fix_size_history_threshold", this->fixSizeHistThresh_)){
            this->fixSizeHistThresh_ = 10;
            std::cout << this->hint_ << ": No history threshold for fixing size parameter found. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": History threshold for fixing size parameter is set to: " << this->fixSizeHistThresh_ << std::endl;
        }  

        // dimension threshold for fixing box size
        if (not this->nh_.getParam(this->ns_ + "/fix_size_dimension_threshold", this->fixSizeDimThresh_)){
            this->fixSizeDimThresh_ = 0.4;
            std::cout << this->hint_ << ": No dimension threshold for fixing size parameter found. Use default: 0.4." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Dimension threshold for fixing size parameter is set to: " << this->fixSizeDimThresh_ << std::endl;
        } 

        // kalman filter parameters
        std::vector<double> kalmanFilterParams;
        if (not this->nh_.getParam(this->ns_ + "/kalman_filter_param", kalmanFilterParams)){
            this->eP_ = 0.5;
            this->eQPos_ = 0.5; // pos prediction noise
            this->eQVel_ = 0.5; // vel prediction noise
            this->eQAcc_ = 0.5; // acc prediction noise
            this->eRPos_ = 0.5; // pos measurement noise
            this->eRVel_ = 0.5; // vel measurement noise
            this->eRAcc_ = 0.5; // acc measurement noise
            std::cout << this->hint_ << ": No kalman filter parameter found. Use default: 0.5." << std::endl;
        }
        else{
            this->eP_ = kalmanFilterParams[0];
            this->eQPos_ = kalmanFilterParams[1]; // pos prediction noise
            this->eQVel_ = kalmanFilterParams[2]; // vel prediction noise
            this->eQAcc_ = kalmanFilterParams[3]; // acc prediction noise
            this->eRPos_ = kalmanFilterParams[4]; // pos measurement noise
            this->eRVel_ = kalmanFilterParams[5]; // vel measurement noise
            this->eRAcc_ = kalmanFilterParams[6]; // acc measurement noise
            std::cout << this->hint_ << ": Kalman filter parameter is set to: [";
            for (int i=0; i<int(kalmanFilterParams.size()); ++i){
                double param = kalmanFilterParams[i];
                if (i != int(kalmanFilterParams.size())-1){
                    std::cout << param << ", ";
                }
                else{
                    std::cout << param;
                }
            }
            std::cout << "]." << std::endl;
        }  

        // num of frames used in KF for observation
        if (not this->nh_.getParam(this->ns_ + "/kalman_filter_averaging_frames", this->kfAvgFrames_)){
            this->kfAvgFrames_ = 10;
            std::cout << this->hint_ << ": No number of frames used in KF for observation parameter found. Use default: 10." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Number of frames used in KF for observation is set to: " << this->kfAvgFrames_ << std::endl;
        } 

        //-------------------------------------动态/静态分类参数----------------------------------------------------
        // skip frame for classification
        if (not this->nh_.getParam(this->ns_ + "/frame_skip", this->skipFrame_)){
            this->skipFrame_ = 5;
            std::cout << this->hint_ << ": No skip frame parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Frames skiped in classification when comparing two point cloud is set to: " << this->skipFrame_ << std::endl;
        }  

        // velocity threshold for dynamic classification
        if (not this->nh_.getParam(this->ns_ + "/dynamic_velocity_threshold", this->dynaVelThresh_)){
            this->dynaVelThresh_ = 0.35;
            std::cout << this->hint_ << ": No dynamic velocity threshold parameter found. Use default: 0.35." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Velocity threshold for dynamic classification is set to: " << this->dynaVelThresh_ << std::endl;
        }  

        // voting threshold for dynamic classification
        if (not this->nh_.getParam(this->ns_ + "/dynamic_voting_threshold", this->dynaVoteThresh_)){
            this->dynaVoteThresh_ = 0.8;
            std::cout << this->hint_ << ": No dynamic velocity threshold parameter found. Use default: 0.8." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Voting threshold for dynamic classification is set to: " << this->dynaVoteThresh_ << std::endl;
        }  

        // frames to force dynamic
        if (not this->nh_.getParam(this->ns_ + "/frames_force_dynamic", this->forceDynaFrames_)){
            this->forceDynaFrames_ = 20;
            std::cout << this->hint_ << ": No range of searching dynamic obstacles in box history found. Use default: 20." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Range of searching dynamic obstacles in box history is set to: " << this->forceDynaFrames_ << std::endl;
        }  

        if (not this->nh_.getParam(this->ns_ + "/frames_force_dynamic_check_range", this->forceDynaCheckRange_)){
            this->forceDynaCheckRange_ = 30;
            std::cout << this->hint_ << ": No threshold for forcing dynamic obstacles found. Use default: 30." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for forcing dynamic obstacles is set to: " << this->forceDynaCheckRange_ << std::endl;
        }  

        // dynamic consistency check
        if (not this->nh_.getParam(this->ns_ + "/dynamic_consistency_threshold", this->dynamicConsistThresh_)){
            this->dynamicConsistThresh_ = 3;
            std::cout << this->hint_ << ": No threshold for dynamic-consistency check found. Use default: 3." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Threshold for dynamic consistency check is set to: " << this->dynamicConsistThresh_ << std::endl;
        }  

        if ( this->histSize_ < this->forceDynaCheckRange_+1){
            ROS_ERROR("history length is too short to perform force-dynamic");
        }

        //-----------------------------------------尺寸约束参数--------------------------------------------------------------
        // constrain target object size
        if (not this->nh_.getParam(this->ns_ + "/target_constrain_size", this->constrainSize_)){
            this->constrainSize_ = false;
            std::cout << this->hint_ << ": No target object constrain size param found. Use default: false." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Target object constrain is set to: " << this->constrainSize_ << std::endl;
        }  

        // target object  sizes
        std::vector<double> targetObjectSizeTemp;
        if (not this->nh_.getParam(this->ns_ + "/target_object_size", targetObjectSizeTemp)){
            std::cout << this->hint_ << ": No target object size found. Do not apply target object size." << std::endl;
        }
        else{
            for (size_t i=0; i<targetObjectSizeTemp.size(); i+=3){
                Eigen::Vector3d targetSize (targetObjectSizeTemp[i+0], targetObjectSizeTemp[i+1], targetObjectSizeTemp[i+2]);
                this->targetObjectSize_.push_back(targetSize);
                std::cout << this->hint_ << ": target object size is set to: [" << targetObjectSizeTemp[i+0]  << ", " 
                << targetObjectSizeTemp[i+1] << ", " <<  targetObjectSizeTemp[i+2] << "]." << std::endl;
            }
            
        }

        // max object size
        std::vector<double> maxObjectSizeTemp;
        if(not this->nh_.getParam(this->ns_ + "/max_object_size", maxObjectSizeTemp)){
            this->maxObjectSize_ = Eigen::Vector3d (2.0, 2.0, 2.0);
            std::cout << this->hint_ << ": No max object size threshold parameter found. Use default: [2.0, 2.0, 2.0]." << endl;
        }
        else{
            this->maxObjectSize_(0) = maxObjectSizeTemp[0];
            this->maxObjectSize_(1) = maxObjectSizeTemp[1];
            this->maxObjectSize_(2) = maxObjectSizeTemp[2];
            std::cout <<  this->hint_ << ": Max object size threshold is set to: [";
            for (size_t i = 0; i < maxObjectSizeTemp.size(); ++i) {
                std::cout << maxObjectSizeTemp[i];
                if (i != maxObjectSizeTemp.size()-1){
                    std::cout << ", ";
                }
            }
            std::cout << "]." << std::endl;
        }

        //-----------------------------------------物体分类参数--------------------------------------------------------------
        // 人的分类阈值
        std::vector<double> classifyHumanThresh;
        if (not this->nh_.getParam(this->ns_ + "/classify_human_threshold", classifyHumanThresh)){
            this->classifyHumanZWidthRatio_ = 2.0;
            this->classifyHumanCentroidZRatio_ = 0.5;
            ROS_WARN_STREAM(this->hint_ << " No classify_human_threshold param. Use default: [2.0, 0.5]");
        }
        else{
            this->classifyHumanZWidthRatio_ = classifyHumanThresh[0];
            this->classifyHumanCentroidZRatio_ = classifyHumanThresh[1];
            ROS_INFO_STREAM(this->hint_ << " classify_human_threshold: [" << this->classifyHumanZWidthRatio_ 
                           << ", " << this->classifyHumanCentroidZRatio_ << "]");
        }

        // 车的分类阈值
        std::vector<double> classifyVehicleThresh;
        if (not this->nh_.getParam(this->ns_ + "/classify_vehicle_threshold", classifyVehicleThresh)){
            this->classifyVehicleXYWidthRatio_ = 1.5;
            this->classifyVehicleCentroidZRatio_ = 0.8;
            ROS_WARN_STREAM(this->hint_ << " No classify_vehicle_threshold param. Use default: [1.5, 0.8]");
        }
        else{
            this->classifyVehicleXYWidthRatio_ = classifyVehicleThresh[0];
            this->classifyVehicleCentroidZRatio_ = classifyVehicleThresh[1];
            ROS_INFO_STREAM(this->hint_ << " classify_vehicle_threshold: [" << this->classifyVehicleXYWidthRatio_ 
                           << ", " << this->classifyVehicleCentroidZRatio_ << "]");
        }

        // 无人机的分类阈值
        std::vector<double> classifyUAVThresh;
        if (not this->nh_.getParam(this->ns_ + "/classify_uav_threshold", classifyUAVThresh)){
            this->classifyUAVMaxSize_ = 0.6;
            this->classifyUAVCentroidZRatio_ = 1.2;
            ROS_WARN_STREAM(this->hint_ << " No classify_uav_threshold param. Use default: [0.6, 1.2]");
        }
        else{
            this->classifyUAVMaxSize_ = classifyUAVThresh[0];
            this->classifyUAVCentroidZRatio_ = classifyUAVThresh[1];
            ROS_INFO_STREAM(this->hint_ << " classify_uav_threshold: [" << this->classifyUAVMaxSize_ 
                           << ", " << this->classifyUAVCentroidZRatio_ << "]");
        }
    }

    void dynamicDetector::registerPub(){
        // 激光雷达边界框发布
        this->lidarBBoxesPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/lidar_bboxes", 10);

        // 过滤后的边界框发布
        this->filteredBBoxesPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/filtered_bboxes", 10);

        // 跟踪的边界框发布
        this->trackedBBoxesPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/tracked_bboxes", 10);

        // 动态边界框发布
        this->dynamicBBoxesPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/dynamic_bboxes", 10);

        // 激光雷达聚类发布 
        this->lidarClustersPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/lidar_clusters", 10);

        // 过滤后的点云发布 
        this->filteredPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/filtered_point_cloud", 10);

        // 动态点云发布
        this->dynamicPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/dynamic_point_cloud", 10);

        // 原始动态点云发布
        this->rawDynamicPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/raw_dynamic_point_cloud", 10);

        // 降采样点可视化发布
        this->downSamplePointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/downsampled_point_cloud", 10);

        // 原始激光雷达点可视化发布
        this->rawLidarPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/raw_lidar_point_cloud", 10);

        // 历史轨迹发布
        this->historyTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/history_trajectories", 10);

        // 速度可视化发布
        this->velVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/velocity_visualizaton", 10);
    }   

    void dynamicDetector::registerCallback(){
        //message_filters和正常的ros订阅区别是，message_filters不会直接调用回调函数，而是满足过滤器的条件才调用
        // 深度图像和位姿回调。reset表示释放旧的对象，处理当前的新对象。new动态内存分配，如果分配的是对象，new会调用该对象的构造函数来初始化它，也分配内存
        
        if (this->useLivoxCustomMsg_){
            // 使用Livox CustomMsg格式
            this->lidarCustomMsgSub_.reset(new message_filters::Subscriber<livox_ros_driver2::CustomMsg>(this->nh_, this->lidarTopicName_, 50));
            
            if (this->localizationMode_ == 0){
                this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
                this->lidarCustomPoseSync_.reset(new message_filters::Synchronizer<lidarCustomPoseSync>(lidarCustomPoseSync(100), *this->lidarCustomMsgSub_, *this->poseSub_));
                this->lidarCustomPoseSync_->registerCallback(boost::bind(&dynamicDetector::lidarCustomPoseCB, this, _1, _2));
            }
            else if (this->localizationMode_ == 1){
                this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
                this->lidarCustomOdomSync_.reset(new message_filters::Synchronizer<lidarCustomOdomSync>(lidarCustomOdomSync(100), *this->lidarCustomMsgSub_, *this->odomSub_));
                this->lidarCustomOdomSync_->registerCallback(boost::bind(&dynamicDetector::lidarCustomOdomCB, this, _1, _2));
            }
            else{
                ROS_ERROR("[dynamicDetector]: Invalid localization mode!");
                exit(0);
            }
        }
        else{
            // 使用标准PointCloud2格式
            this->lidarCloudSub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(this->nh_, this->lidarTopicName_, 50));
            
            if (this->localizationMode_ == 0){
                this->poseSub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(this->nh_, this->poseTopicName_, 25));
                this->lidarPoseSync_.reset(new message_filters::Synchronizer<lidarPoseSync>(lidarPoseSync(100), *this->lidarCloudSub_, *this->poseSub_));
                this->lidarPoseSync_->registerCallback(boost::bind(&dynamicDetector::lidarPoseCB, this, _1, _2));
            }
            else if (this->localizationMode_ == 1){
                this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(this->nh_, this->odomTopicName_, 25));
                this->lidarOdomSync_.reset(new message_filters::Synchronizer<lidarOdomSync>(lidarOdomSync(100), *this->lidarCloudSub_, *this->odomSub_));
                this->lidarOdomSync_->registerCallback(boost::bind(&dynamicDetector::lidarOdomCB, this, _1, _2));
            }
            else{
                ROS_ERROR("[dynamicDetector]: Invalid localization mode!");
                exit(0);
            }
        }

        // 激光雷达检测定时器
        this->lidarDetectionTimer_ = this->nh_.createTimer(ros::Duration(this->dt_), &dynamicDetector::lidarDetectionCB, this);

        // 跟踪定时器
        this->trackingTimer_ = this->nh_.createTimer(ros::Duration(this->dt_), &dynamicDetector::trackingCB, this);

        // 分类定时器
        this->classificationTimer_ = this->nh_.createTimer(ros::Duration(this->dt_), &dynamicDetector::classificationCB, this);
    
        // 可视化定时器
        this->visTimer_ = this->nh_.createTimer(ros::Duration(this->dt_), &dynamicDetector::visCB, this);
        
		// 获取动态障碍物服务
		this->getDynamicObstacleServer_ = this->nh_.advertiseService("onboard_detector/get_dynamic_obstacles", &dynamicDetector::getDynamicObstacles, this);
    }


    // 获取动态障碍物的服务回调函数。对获取的障碍物按与机器人的距离从小到大排序
    bool dynamicDetector::getDynamicObstacles(onboard_detector::GetDynamicObstacles::Request& req, 
                                              onboard_detector::GetDynamicObstacles::Response& res) {
        // 从服务请求中获取机器人当前的位置
        Eigen::Vector3d currPos = Eigen::Vector3d (req.current_position.x, req.current_position.y, req.current_position.z);

        // 创建一个向量，用于存储障碍物id及与机器人距离的键值对，方便后续排序
        std::vector<std::pair<double, onboardDetector::box3D>> obstaclesWithDistances;

        // 遍历当前所有已检测到的动态障碍物
        for (const onboardDetector::box3D& bbox : this->dynamicBBoxes_) {
            Eigen::Vector3d obsPos(bbox.x, bbox.y, bbox.z);
            Eigen::Vector3d diff = currPos - obsPos;
            diff(2) = 0.; // 忽略Z轴差异，计算2D平面距离
            double distance = diff.norm();

            // 如果障碍物在请求的范围之内，则将其添加到列表中
            if (distance <= req.range) {
                obstaclesWithDistances.push_back(std::make_pair(distance, bbox));
            }
        }

        // 按距离从小到大对障碍物进行排序
        std::sort(obstaclesWithDistances.begin(), obstaclesWithDistances.end(), 
                [](const std::pair<double, onboardDetector::box3D>& a, const std::pair<double, onboardDetector::box3D>& b) {
                    return a.first < b.first;
                });

        // 将排序后的障碍物信息填充到服务响应中
        for (const auto& item : obstaclesWithDistances) {
            const onboardDetector::box3D& bbox = item.second;

            geometry_msgs::Vector3 pos;
            geometry_msgs::Vector3 vel;
            geometry_msgs::Vector3 size;

            // 填充位置
            pos.x = bbox.x;
            pos.y = bbox.y;
            pos.z = bbox.z;

            // 填充速度（Z轴速度设为0）
            vel.x = bbox.Vx;
            vel.y = bbox.Vy;
            vel.z = 0.;

            // 填充尺寸
            size.x = bbox.x_width;
            size.y = bbox.y_width;
            size.z = bbox.z_width;

            res.position.push_back(pos);
            res.velocity.push_back(vel);
            res.size.push_back(size);
        }

        return true; // 表示服务成功完成
    }

    //转换点云格式，滤波一定范围内的点
    void dynamicDetector::lidarPoseCB(const sensor_msgs::PointCloud2ConstPtr& cloudMsg, const geometry_msgs::PoseStampedConstPtr& pose){
        // [Performance Timing] 测量回调函数耗时
        // auto start_time = std::chrono::high_resolution_clock::now();
        
        // 仅用于可视化，存储最新的原始点云消息
        this->hasSensorPose_ = true ;
        this->latestCloud_ = cloudMsg;

        // --- 更新位姿信息（提前更新，避免后续重复计算） ---
        Eigen::Matrix4d lidarPoseMatrix;
        this->getLidarPose(pose, lidarPoseMatrix);

        // 更新机器人主体的位姿
        this->position_(0) = pose->pose.position.x;
        this->position_(1) = pose->pose.position.y;
        this->position_(2) = pose->pose.position.z;
        Eigen::Quaterniond quat(pose->pose.orientation.w, pose->pose.orientation.x, 
                                pose->pose.orientation.y, pose->pose.orientation.z);
        this->orientation_ = quat.toRotationMatrix();

        // 更新激光雷达的位姿
        this->positionLidar_(0) = lidarPoseMatrix(0, 3);
        this->positionLidar_(1) = lidarPoseMatrix(1, 3);
        this->positionLidar_(2) = lidarPoseMatrix(2, 3);
        this->orientationLidar_ = lidarPoseMatrix.block<3, 3>(0, 0);

        // 将ROS点云消息转换为PCL点云格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloudMsg, *tempCloud);

        // --- 优化：一次性滤波（X、Y范围）并进行均匀密度降采样 ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr preTransformCloud(new pcl::PointCloud<pcl::PointXYZ>());
        preTransformCloud->reserve(tempCloud->size()); // 预分配内存，估计保留1/3的点

        double x_max = this->localLidarRange_.x();
        double y_max = this->localLidarRange_.y();
        
        for (const pcl::PointXYZ &pt : tempCloud->points) {
            // 先做范围检查（最快的操作）
            if (std::abs(pt.x) > x_max || std::abs(pt.y) > y_max) {
                continue;
            }
            preTransformCloud->push_back(pt);
            
            //均匀密度降采样
            // // 计算点到传感器的距离
            // double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            // // 使用平滑的S型曲线计算采样概率
            // // 采用 tanh 函数实现平滑过渡
            // double normalized_dist = (dist - characteristic_dist) / characteristic_dist;
            // // tanh函数：从-1平滑过渡到+1
            // // 映射到采样率：从0.3平滑增长到0.8
            // double tanh_value = std::tanh(normalized_dist);
            // double sampling_prob = 0.50 + 0.25 * tanh_value;
            // // dist=0时 prob≈0.35, dist=characteristic_dist时 prob=0.55, dist→∞时 prob≈0.80
            // // 确保概率在合理范围内
            // sampling_prob = std::max(0.20, std::min(sampling_prob, 0.85));
            // double r = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            // if (r < sampling_prob) {
            //     preTransformCloud->push_back(pt);
            // }
        }

        // --- 坐标变换 ---
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.linear() = this->orientationLidar_;
        transform.translation() = this->positionLidar_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*preTransformCloud, *transformedCloud, transform);

        // --- 过滤地面和天花板（在变换后进行，避免重复操作） ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr groundRoofFilterCloud (new pcl::PointCloud<pcl::PointXYZ>());
        groundRoofFilterCloud->reserve(transformedCloud->size());
        
        for (const pcl::PointXYZ &pt : transformedCloud->points) {
            if (pt.z >= this->groundHeight_ && pt.z <= this->roofHeight_) {
                groundRoofFilterCloud->push_back(pt);
            }
        }

        // // --- 空间自适应体素网格降采样 ---
        // pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud (new pcl::PointCloud<pcl::PointXYZ>());
        
        // if (groundRoofFilterCloud->size() > this->downSampleThresh_) {
        //     // 根据点云密度自适应调整体素大小
        //     double density_ratio = static_cast<double>(groundRoofFilterCloud->size()) / static_cast<double>(this->downSampleThresh_);
        //     float adaptive_leaf_size = 0.1f * std::sqrt(density_ratio);
        //     adaptive_leaf_size = std::max(0.08f, std::min(adaptive_leaf_size, 0.3f)); // 限制在合理范围内
            
        //     pcl::VoxelGrid<pcl::PointXYZ> sor;
        //     sor.setInputCloud(groundRoofFilterCloud);
        //     sor.setLeafSize(adaptive_leaf_size, adaptive_leaf_size, adaptive_leaf_size);
        //     sor.filter(*downsampledCloud);
        // } else {
        //     downsampledCloud = groundRoofFilterCloud;
        // }

        // 存储处理后的激光雷达点云
        this->lidarCloud_ = groundRoofFilterCloud;
        
        // 将处理后的点云发布出去，用于可视化
        sensor_msgs::PointCloud2 outputCloud;
        pcl::toROSMsg(*this->lidarCloud_, outputCloud);
        outputCloud.header.frame_id = "map";
        outputCloud.header.stamp = cloudMsg->header.stamp;
        this->downSamplePointsPub_.publish(outputCloud);
        
        // [Performance Timing] 输出耗时
        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // ROS_INFO_THROTTLE(1.0, "%s: lidarPoseCB took %.3f ms, points: %lu -> %lu", 
        //                  this->hint_.c_str(), duration.count() / 1000.0, 
        //                  tempCloud->size(), this->lidarCloud_->size());
    }

    // 里程计回调函数，处理点云和里程计数据
    void dynamicDetector::lidarOdomCB(const sensor_msgs::PointCloud2ConstPtr& cloudMsg, const nav_msgs::OdometryConstPtr& odom){
        // [Performance Timing] 测量回调函数耗时
        // auto start_time = std::chrono::high_resolution_clock::now();
        
        // 用于可视化
        this->hasSensorPose_ = true ;
        this->latestCloud_ = cloudMsg;

        // --- 提前更新位姿信息 ---
        Eigen::Matrix4d lidarPoseMatrix;
        this->getLidarPose(odom, lidarPoseMatrix);

        this->position_(0) = odom->pose.pose.position.x;
        this->position_(1) = odom->pose.pose.position.y;
        this->position_(2) = odom->pose.pose.position.z;
        Eigen::Quaterniond quat(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, 
                                odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
        this->orientation_ = quat.toRotationMatrix();

        this->positionLidar_(0) = lidarPoseMatrix(0, 3);
        this->positionLidar_(1) = lidarPoseMatrix(1, 3);
        this->positionLidar_(2) = lidarPoseMatrix(2, 3);
        this->orientationLidar_ = lidarPoseMatrix.block<3, 3>(0, 0);

        // 局部点云转换
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloudMsg, *tempCloud);

        // --- 优化：一次性滤波（X、Y范围）并进行均匀密度降采样 ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr preTransformCloud(new pcl::PointCloud<pcl::PointXYZ>());
        // 范围过滤后保留的点数不确定，预分配为原始点云大小以避免多次重新分配
        preTransformCloud->reserve(tempCloud->size());

        double x_max = this->localLidarRange_.x();
        double y_max = this->localLidarRange_.y();
        
        for (const pcl::PointXYZ &pt : tempCloud->points) {
            // 先做范围检查（最快的操作）
            if (std::abs(pt.x) > x_max || std::abs(pt.y) > y_max) {
                continue;
            }
            preTransformCloud->push_back(pt);
            
            // // 计算点到传感器的距离
            // double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            // // 使用平滑的S型曲线计算采样概率
            // // 采用 tanh 函数实现平滑过渡
            // double normalized_dist = (dist - characteristic_dist) / characteristic_dist;
            // // tanh函数：从-1平滑过渡到+1
            // // 映射到采样率：从0.3平滑增长到0.8
            // double tanh_value = std::tanh(normalized_dist);
            // double sampling_prob = 0.50 + 0.25 * tanh_value;
            // // dist=0时 prob≈0.35, dist=characteristic_dist时 prob=0.55, dist→∞时 prob≈0.80
            // // 确保概率在合理范围内
            // sampling_prob = std::max(0.20, std::min(sampling_prob, 0.85));
            // double r = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
            // if (r < sampling_prob) {
            //     preTransformCloud->push_back(pt);
            // }
        }

        // --- 坐标变换 ---
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.linear() = this->orientationLidar_;
        transform.translation() = this->positionLidar_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*preTransformCloud, *transformedCloud, transform);

        // --- 过滤地面和天花板（在变换后进行，避免重复操作） ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr groundRoofFilterCloud (new pcl::PointCloud<pcl::PointXYZ>());
        groundRoofFilterCloud->reserve(transformedCloud->size());
        
        for (const pcl::PointXYZ &pt : transformedCloud->points) {
            if (pt.z >= this->groundHeight_ && pt.z <= this->roofHeight_) {
                groundRoofFilterCloud->push_back(pt);
            }
        }

        // // --- 空间自适应体素网格降采样 ---
        // pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud (new pcl::PointCloud<pcl::PointXYZ>());
        // if (groundRoofFilterCloud->size() > this->downSampleThresh_) {
        //     // 根据点云密度自适应调整体素大小
        //     double density_ratio = static_cast<double>(groundRoofFilterCloud->size()) / static_cast<double>(this->downSampleThresh_);
        //     float adaptive_leaf_size = 0.1f * std::sqrt(density_ratio);
        //     adaptive_leaf_size = std::max(0.08f, std::min(adaptive_leaf_size, 0.3f)); // 限制在合理范围内
            
        //     pcl::VoxelGrid<pcl::PointXYZ> sor;
        //     sor.setInputCloud(groundRoofFilterCloud);
        //     sor.setLeafSize(adaptive_leaf_size, adaptive_leaf_size, adaptive_leaf_size);
        //     sor.filter(*downsampledCloud);
        // } else {
        //     downsampledCloud = groundRoofFilterCloud;
        // }

        // 存储处理后的点云
        this->lidarCloud_ = groundRoofFilterCloud;
        
        // 发布降采样后的点云
        sensor_msgs::PointCloud2 outputCloud;
        pcl::toROSMsg(*this->lidarCloud_, outputCloud);
        outputCloud.header.frame_id = "map";
        outputCloud.header.stamp = cloudMsg->header.stamp;
        this->downSamplePointsPub_.publish(outputCloud);
        
        // [Performance Timing] 输出耗时
        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // ROS_INFO_THROTTLE(1.0, "%s: lidarOdomCB took %.3f ms, points: %lu -> %lu", 
        //                  this->hint_.c_str(), duration.count() / 1000.0,
        //                  tempCloud->size(), this->lidarCloud_->size());
    }

    // 激光雷达检测定时器回调函数
    void dynamicDetector::lidarDetectionCB(const ros::TimerEvent&){
        // // [Performance Timing] 测量回调函数耗时
        // auto start_time = std::chrono::high_resolution_clock::now();
        
        this->lidarDetect();
        this->newDetectFlag_ = true; // get a new detection
        
        // // [Performance Timing] 输出耗时
        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // ROS_INFO_THROTTLE(1.0, "%s: lidarDetectionCB took %.3f ms", this->hint_.c_str(), duration.count() / 1000.0);
    }

    // 跟踪定时器回调函数,有个问题，匹配时，多出的轨迹直接丢掉
    void dynamicDetector::trackingCB(const ros::TimerEvent&){
        // // [Performance Timing] 测量回调函数耗时
        // auto start_time = std::chrono::high_resolution_clock::now();
        
        // 数据关联线程
        std::vector<int> bestMatch; // 存储当前检测与历史障碍物的匹配索引。
        this->boxAssociation(bestMatch); // 执行边界框关联。
        
        // kalman filter tracking
        // 卡尔曼滤波跟踪
        if (bestMatch.size()){ // 如果找到匹配。
            this->kalmanFilterAndUpdateHist(bestMatch); // 更新卡尔曼滤波器和历史记录。
        }
        else { // 如果没有匹配。
            // 清空历史记录。
            this->boxHist_.clear();
            this->pcHist_.clear();
            this->pcCenterHist_.clear();
            this->pcStdHist_.clear();
        }
        
        // // [Performance Timing] 输出耗时
        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // ROS_INFO_THROTTLE(1.0, "%s: trackingCB took %.3f ms", this->hint_.c_str(), duration.count() / 1000.0);
    }

    // 分类定时器回调函数
    void dynamicDetector::classificationCB(const ros::TimerEvent&){
        // // [Performance Timing] 测量回调函数耗时
        // auto start_time = std::chrono::high_resolution_clock::now();
        
        // 创建一个临时向量来存储当前帧检测到的动态边界框
        std::vector<onboardDetector::box3D> dynamicBBoxesTemp;

        // 遍历所有被跟踪目标的点云/边界框历史
        // 注意：在某些情况下，我们不需要执行动态障碍物识别
        for (size_t i=0; i<this->pcHist_.size() ; ++i){
            // ===================================================================================
            // 情况一：如果目标已被YOLO等外部检测器识别为人类，则直接视为动态（此功能当前被注释掉）
            // if (this->boxHist_[i][0].is_human){
            //     dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
            //     continue;
            // }
            // ===================================================================================


            // ===================================================================================
            // 情况二：历史记录长度不足以进行分类
            // 确定用于比较的当前帧与历史帧之间的时间间隔（帧数）
            int curFrameGap;
            if (int(this->pcHist_[i].size()) < this->skipFrame_+1){
                // 如果历史记录不够长，就用现有的最远一帧进行比较
                curFrameGap = this->pcHist_[i].size() - 1;
            }
            else{
                // 否则，使用参数设定的帧间隔
                curFrameGap = this->skipFrame_;
            }
            // ===================================================================================


            // ==================================================================================
            // 情况三：强制动态（如果一个障碍物在过去一段时间内被频繁分类为动态，则强制认定其为动态）
            int dynaFrames = 0;
            if (int(this->boxHist_[i].size()) > this->forceDynaCheckRange_){
                for (int j=1 ; j<this->forceDynaCheckRange_+1 ; ++j){
                    if (this->boxHist_[i][j].is_dynamic){
                        ++dynaFrames;
                    }
                }
            }

            if (dynaFrames >= this->forceDynaFrames_){
                this->boxHist_[i][0].is_dynamic = true;
                dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
                continue;
            }
            // ===================================================================================

            // 获取当前帧和历史帧的点云
            std::vector<Eigen::Vector3d> currPc = this->pcHist_[i][0];
            std::vector<Eigen::Vector3d> prevPc = this->pcHist_[i][curFrameGap];
            
            // 初始化速度向量
            Eigen::Vector3d Vcur(0.,0.,0.); // 单个点的速度
            Eigen::Vector3d Vbox(0.,0.,0.); // 整个边界框的平均速度
            Eigen::Vector3d Vkf(0.,0.,0.);  // 卡尔曼滤波器估计的速度
            
            int numPoints = currPc.size(); // 点云中的总点数，用于计算投票率
            int votes = 0; // “动态”票数

            // 计算边界框中心点的速度
            Vbox(0) = (this->boxHist_[i][0].x - this->boxHist_[i][curFrameGap].x)/(this->dt_*curFrameGap);
            Vbox(1) = (this->boxHist_[i][0].y - this->boxHist_[i][curFrameGap].y)/(this->dt_*curFrameGap);
            Vbox(2) = (this->boxHist_[i][0].z - this->boxHist_[i][curFrameGap].z)/(this->dt_*curFrameGap);
            
            // 获取卡尔曼滤波器估计的速度
            Vkf(0) = this->boxHist_[i][0].Vx;
            Vkf(1) = this->boxHist_[i][0].Vy;

            // 遍历当前点云中的每一个点，通过与历史点云比较来“投票”
            for (size_t j=0 ; j<currPc.size() ; ++j){
                double minDist = 2; // 初始化一个较大的最小距离
                Eigen::Vector3d nearestVect;
                // 在历史点云中为当前点寻找最近邻点
                for (size_t k=0 ; k<prevPc.size() ; k++){ 
                    double dist = (currPc[j]-prevPc[k]).norm();
                    if (abs(dist) < minDist){
                        minDist = dist;
                        nearestVect = currPc[j]-prevPc[k]; // 记录位移向量
                    }
                }
                // 计算该点的速度，并忽略Z轴
                Vcur = nearestVect/(this->dt_*curFrameGap); Vcur(2) = 0;
                // 计算点的速度向量与边界框整体速度向量的余弦相似度
                double velSim = Vcur.dot(Vbox)/(Vcur.norm()*Vbox.norm());

                // 如果速度方向相反，则认为该点是噪声或匹配错误，不计入总点数
                if (velSim < 0){
                    --numPoints;
                }
                else{
                    // 如果点的速度超过动态阈值，则投一票“动态”
                    if (Vcur.norm()>this->dynaVelThresh_){
                        ++votes;
                    }
                }
            }
            
            
            // --- 根据投票结果和速度阈值判断是否为动态 ---
            // 计算动态票的比例
            double voteRatio = (numPoints>0)?double(votes)/double(numPoints):0;
            // 获取卡尔曼滤波器估计的速度大小
            double velNorm = Vkf.norm();

            // 综合两个条件进行判断:
            // 1. 点云投票率是否足够高
            // 2. 卡尔曼滤波器估计的速度是否足够快
            if (voteRatio>=this->dynaVoteThresh_ && velNorm>=this->dynaVelThresh_){
                // 如果满足条件，首先标记为“动态候选”
                this->boxHist_[i][0].is_dynamic_candidate = true;
                
                // --- 动态一致性检查 ---
                // 检查过去几帧是否也一直被认为是动态的，以增加鲁棒性
                int dynaConsistCount = 0;
                if (int(this->boxHist_[i].size()) >= this->dynamicConsistThresh_){
                    for (int j=0 ; j<this->dynamicConsistThresh_; ++j){
                        // 如果是动态候选、被识别为人或已经是动态，则计数
                        if (this->boxHist_[i][j].is_dynamic_candidate or this->boxHist_[i][j].is_human or this->boxHist_[i][j].is_dynamic){
                            ++dynaConsistCount;
                        }
                    }
                }            
                // 如果连续几帧都满足条件
                if (dynaConsistCount == this->dynamicConsistThresh_){
                    // 则正式标记为动态，并添加到本轮的动态障碍物列表中
                    this->boxHist_[i][0].is_dynamic = true;
                    dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);    
                }
            }
        }

        // --- 可选步骤：根据目标尺寸过滤动态障碍物 ---
        if (this->constrainSize_){
            std::vector<onboardDetector::box3D> dynamicBBoxesBeforeConstrain = dynamicBBoxesTemp;
            dynamicBBoxesTemp.clear();

            // 遍历所有初步认定的动态障碍物
            for (onboardDetector::box3D ob : dynamicBBoxesBeforeConstrain){
                bool findMatch = false;
                // 检查其尺寸是否与预设的“目标尺寸”之一匹配
                for (Eigen::Vector3d targetSize : this->targetObjectSize_){
                    double xdiff = std::abs(ob.x_width - targetSize(0));
                    double ydiff = std::abs(ob.y_width - targetSize(1));
                    double zdiff = std::abs(ob.z_width - targetSize(2)); 
                    // 如果尺寸差异在容忍范围内
                    if (xdiff < 0.8 and ydiff < 0.8 and zdiff < 1.0){
                        findMatch = true;
                    }
                }

                // 如果尺寸匹配，则保留该障碍物
                if (findMatch){
                    dynamicBBoxesTemp.push_back(ob);
                }
            }
        }

        // 更新最终的动态障碍物列表
        this->dynamicBBoxes_ = dynamicBBoxesTemp;
        
        // // [Performance Timing] 输出耗时
        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // ROS_INFO_THROTTLE(1.0, "%s: classificationCB took %.3f ms", this->hint_.c_str(), duration.count() / 1000.0);
    }

    // 可视化定时器回调函数
    void dynamicDetector::visCB(const ros::TimerEvent&){
        // // [Performance Timing] 测量回调函数耗时
        // auto start_time = std::chrono::high_resolution_clock::now();
        
        //----------------------------障碍物检测阶段----------------------------------------
        // 从原始（未降采样）的激光雷达数据中提取并发布动态点云，以获得更密集的视觉效果
        this->publishRawDynamicPoints();
        // 发布过滤后的点云（通常是灰色的），检测完障碍物的所有点云簇
        this->publishFilteredPoints();
        // 发布带颜色的激光雷达聚类点云，每个聚类一个随机颜色
        this->publishLidarClusters(); 
        // 发布雷达检测器的聚类边界框（灰色）
        this->publish3dBox(this->lidarBBoxes_, this->lidarBBoxesPub_, 0.5, 0.5, 0.5); 
        // 等于同上
        this->publish3dBox(this->filteredBBoxes_, this->filteredBBoxesPub_, 0, 1, 1);

        //----------------------------障碍物关联和跟踪阶段-------------------------------------
        // 发布经过卡尔曼滤波跟踪后的边界框（黄色）
        this->publish3dBox(this->trackedBBoxes_, this->trackedBBoxesPub_, 1, 1, 0);
        // 发布被跟踪物体的历史轨迹线
        this->publishHistoryTraj();
        // 将被跟踪物体的速度作为文本发布到Rviz中
        this->publishVelVis();
        
        //-----------------------------动态障碍物识别阶段--------------------------------------
        // 发布最终被分类为动态的边界框（蓝色）
        this->publish3dBox(this->dynamicBBoxes_, this->dynamicBBoxesPub_, 0, 0, 1);
        // 提取并发布属于动态障碍物的点云
        std::vector<Eigen::Vector3d> dynamicPoints;
        this->getDynamicPc(dynamicPoints);
        this->publishPoints(dynamicPoints, this->dynamicPointsPub_);
        
        // // [Performance Timing] 输出耗时
        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // ROS_INFO_THROTTLE(1.0, "%s: visCB took %.3f ms", this->hint_.c_str(), duration.count() / 1000.0);
    }

    /*!
     * @brief 对单个边界框进行物体分类
     * @param bbox 待分类的边界框（引用传递，会修改其分类标志）
     * @param centroid 点云质心坐标 [x, y, z, 1]
     * 分类规则：
     * - 人：z轴宽度 >= x/y轴最小值的阈值倍数 且 质心z < z轴宽度的阈值倍数
     * - 车：x/y轴最大宽度 >= z轴的阈值倍数 且 质心z < z轴宽度的阈值倍数
     * - 无人机：x/y/z轴宽度都 < 阈值 且 质心z > z轴宽度的阈值倍数
     * - 其他：不满足以上条件
     */
    void dynamicDetector::classifyBox(onboardDetector::box3D& bbox, const Eigen::Vector4f& centroid){
        double x_width = bbox.x_width;
        double y_width = bbox.y_width;
        double z_width = bbox.z_width;
        double centroid_z = centroid(2);
        
        // 计算x、y轴的最小值和最大值
        double xy_min = std::min(x_width, y_width);
        double xy_max = std::max(x_width, y_width);

        // 分类为人：z轴宽度大，质心靠下（站立的人形）
        if (z_width >= xy_min * this->classifyHumanZWidthRatio_ && 
            centroid_z < z_width * this->classifyHumanCentroidZRatio_) {
            bbox.is_human = true;
        }
        // 分类为车：x/y轴宽度大，质心靠下（扁平的车形）
        else if (xy_max >= z_width * this->classifyVehicleXYWidthRatio_ && 
                 centroid_z < z_width * this->classifyVehicleCentroidZRatio_) {
            bbox.is_che = true;
        }
        // 分类为无人机：体积小，质心靠上（悬浮在空中）
        else if (x_width < this->classifyUAVMaxSize_ && 
                 y_width < this->classifyUAVMaxSize_ && 
                 z_width < this->classifyUAVMaxSize_ && 
                 centroid_z > z_width * this->classifyUAVCentroidZRatio_) {
            bbox.is_uav = true;
        }
        // 其他情况
        else {
            bbox.is_else = true;
        }
    }

    /*!
     * 该函数通过激光雷达点云数据检测环境中的障碍物。它会初始化激光雷达检测器（如果尚未初始化），
     * 执行DBSCAN聚类算法来识别点云中的不同对象，并过滤掉尺寸过大的边界框。
     * 最终结果保存在lidarBBoxes_和lidarClusters_成员变量中。
     */
    void dynamicDetector::lidarDetect(){
        // 检查激光雷达检测器是否已初始化，如果没有则创建并设置参数
        if (this->lidarDetector_ == NULL){
            this->lidarDetector_.reset(new lidarDetector());
            this->lidarDetector_->setParams(this->lidarDBEpsilon_, this->lidarDBMinPoints_, this->lidarDBUseAdaptive_, this->lidarDBDistanceScale_);
        }

        // 检查是否有激光雷达点云数据
        if (this->lidarCloud_ != NULL){
            // 将点云数据传递给检测器并执行DBSCAN聚类
            this->lidarDetector_->getPointcloud(this->lidarCloud_);
            this->lidarDetector_->lidarDBSCAN();

            // 获取聚类结果和对应的边界框
            std::vector<onboardDetector::Cluster> lidarClustersRaw = this->lidarDetector_->getClusters();
            std::vector<onboardDetector::Cluster> lidarClustersFiltered;
            std::vector<onboardDetector::box3D> lidarBBoxesRaw = this->lidarDetector_->getBBoxes();
            std::vector<onboardDetector::box3D> lidarBBoxesFiltered;
            
            // 遍历所有边界框，过滤掉尺寸过大的对象并进行分类
            for (int i=0; i<int(lidarBBoxesRaw.size()); ++i){
                onboardDetector::box3D lidarBBox = lidarBBoxesRaw[i];
                // 过滤掉尺寸超过阈值的边界框
                if(lidarBBox.x_width > this->maxObjectSize_(0) || lidarBBox.y_width > this->maxObjectSize_(1) || lidarBBox.z_width > this->maxObjectSize_(2)){
                    continue;
                }
                
                // 对边界框进行分类
                this->classifyBox(lidarBBox, lidarClustersRaw[i].centroid);
                
                lidarBBoxesFiltered.push_back(lidarBBox);
                lidarClustersFiltered.push_back(lidarClustersRaw[i]);            
            }
            
            // 保存过滤后的结果
            this->lidarBBoxes_ = lidarBBoxesFiltered;
            this->lidarClusters_ = lidarClustersFiltered;
        }

        // 临时存储来自激光雷达的边界框及其点云特征
        std::vector<onboardDetector::box3D> lidarBBoxesTemp;
        std::vector<std::vector<Eigen::Vector3d>> lidarPcClustersTemp;
        std::vector<Eigen::Vector3d> lidarPcClusterCentersTemp;
        std::vector<Eigen::Vector3d> lidarPcClusterStdsTemp; // 存储激光雷达输出

        //获取激光雷达边界框及其对应的点云簇和特征
        for (size_t i = 0; i < this->lidarBBoxes_.size(); ++i) {
            onboardDetector::box3D lidarBBox = this->lidarBBoxes_[i];
            onboardDetector::Cluster cluster = this->lidarClusters_[i];

            std::vector<Eigen::Vector3d> pcCluster;
            for (const pcl::PointXYZ& point : cluster.points->points) {
                pcCluster.emplace_back(point.x, point.y, point.z);
            }

            // 提取点云簇的质心
            Eigen::Vector3d clusterCenter(cluster.centroid[0], cluster.centroid[1], cluster.centroid[2]);

            // 计算点云簇的标准差
            Eigen::Vector3d clusterStd = cluster.eigen_values.cwiseSqrt().cast<double>();

            // 存入临时变量
            lidarBBoxesTemp.push_back(lidarBBox);
            lidarPcClustersTemp.push_back(pcCluster);
            lidarPcClusterCentersTemp.push_back(clusterCenter);
            lidarPcClusterStdsTemp.push_back(clusterStd);
        }

        // 更新最终的过滤结果
        this->filteredBBoxes_ = lidarBBoxesTemp;
        this->filteredPcClusters_ = lidarPcClustersTemp;
        this->filteredPcClusterCenters_ = lidarPcClusterCentersTemp;
        this->filteredPcClusterStds_ = lidarPcClusterStdsTemp;
    }

    /*!
     * @brief 使用马氏距离和匈牙利算法进行数据关联
     * @param[out] bestMatch 最佳匹配结果，bestMatch[i]表示第i个当前检测对应的历史轨迹索引（-1表示新目标）
     * 
     * 关联流程：
     * 1. 首次检测：初始化历史记录和卡尔曼滤波器
     * 2. 后续检测：
     *    - 构建代价矩阵（基于马氏距离、尺寸差异、点云标准差差异）
     *    - 使用匈牙利算法求解最优匹配
     *    - 应用关联门限，拒绝不可靠匹配
     */
    void dynamicDetector::boxAssociation(std::vector<int>& bestMatch){
        int numCurrObjs = int(this->filteredBBoxes_.size());
        
        // 第一次检测：初始化所有目标
        if (this->boxHist_.size() == 0){
            this->boxHist_.resize(numCurrObjs);
            this->pcHist_.resize(numCurrObjs);
            this->pcCenterHist_.resize(numCurrObjs);
            this->pcStdHist_.resize(numCurrObjs);
            bestMatch.resize(numCurrObjs, -1);
            
            for (int i = 0; i < numCurrObjs; ++i){
                this->boxHist_[i].push_back(this->filteredBBoxes_[i]);
                this->pcHist_[i].push_back(this->filteredPcClusters_[i]);
                this->pcCenterHist_[i].push_back(this->filteredPcClusterCenters_[i]);
                this->pcStdHist_[i].push_back(this->filteredPcClusterStds_[i]);
                
                // 初始化卡尔曼滤波器
                MatrixXd states, A, B, H, P, Q, R;
                this->kalmanFilterMatrixAcc(this->filteredBBoxes_[i], states, A, B, H, P, Q, R);
                onboardDetector::kalman_filter newFilter;
                newFilter.setup(states, A, B, H, P, Q, R);
                this->filters_.push_back(newFilter);
            }
        }
        else if (this->newDetectFlag_){
            // 后续检测：使用匈牙利算法进行关联
            int numHistObjs = int(this->boxHist_.size());
            bestMatch.resize(numCurrObjs, -1);
            
            // 构建代价矩阵
            std::vector<std::vector<double>> costMatrix(numCurrObjs, std::vector<double>(numHistObjs, 1e9));
            
            //  遍历当前障碍物与所有历史轨迹的代价，即矩阵的行，为当前检测到的障碍物，列为按顺序排好的每个历史轨迹
            for (int i = 0; i < numCurrObjs; ++i){
                const onboardDetector::box3D& currBox = this->filteredBBoxes_[i];
                const Eigen::Vector3d& currStd = this->filteredPcClusterStds_[i];
                
                for (int j = 0; j < numHistObjs; ++j){
                    // 获取历史轨迹的最新状态
                    const onboardDetector::box3D& histBox = this->boxHist_[j][0];
                    const Eigen::Vector3d& histStd = this->pcStdHist_[j][0];
                    
                    // 使用卡尔曼滤波器预测的位置
                    onboardDetector::box3D predBox = histBox;
                    const MatrixXd& filterStates = this->filters_[j].getStates();
                    predBox.x = filterStates(0, 0);
                    predBox.y = filterStates(2, 0);
                    
                    // 获取预测位置的协方差
                    const MatrixXd& P = this->filters_[j].getCovariance();
                    Eigen::Matrix2d covariance;
                    covariance(0, 0) = P(0, 0); // x的方差
                    covariance(0, 1) = P(0, 2); // x-y协方差
                    covariance(1, 0) = P(2, 0); // y-x协方差
                    covariance(1, 1) = P(2, 2); // y的方差
                    
                    // 计算关联代价
                    double cost = this->computeAssociationCost(predBox, histStd, currBox, currStd, covariance);
                    
                    // 应用关联门限
                    if (cost < this->associationGateThresh_){
                        costMatrix[i][j] = cost;
                    }
                }
            }
            
            // 使用匈牙利算法求解最优匹配
            this->hungarianAlgorithm(costMatrix, bestMatch);
        }
        
        this->newDetectFlag_ = false;
    }

    /*!
     * @brief 计算马氏距离
     * @param posDiff 位置差异向量 [dx, dy]
     * @param covariance 协方差矩阵 2x2
     * @return 马氏距离的平方
     * 
     * 马氏距离公式: d^2 = (x - μ)^T * Σ^(-1) * (x - μ)
     * 其中 Σ 是协方差矩阵
     */
    double dynamicDetector::computeMahalanobisDistance(const Eigen::Vector2d& posDiff, const Eigen::Matrix2d& covariance){
        // 计算协方差矩阵的逆
        Eigen::Matrix2d covInv = covariance.inverse();
        
        // 计算马氏距离的平方
        double mahalDist = posDiff.transpose() * covInv * posDiff;
        
        return mahalDist;
    }

    /*!
     * @brief 计算数据关联的总代价
     * @param predBox 预测的边界框（来自卡尔曼滤波器）
     * @param predStd 预测时刻的点云标准差
     * @param measBox 当前测量的边界框
     * @param measStd 当前测量的点云标准差
     * @param covariance 预测位置的协方差矩阵
     * @return 总关联代价（越小越好）
     * 
     * 代价函数组成：
     * 1. 位置代价：马氏距离（考虑预测不确定性）
     * 2. 尺寸代价：归一化的尺寸差异
     * 3. 形状代价：点云标准差的变化
     */
    double dynamicDetector::computeAssociationCost(const onboardDetector::box3D& predBox, const Eigen::Vector3d& predStd,
                                                   const onboardDetector::box3D& measBox, const Eigen::Vector3d& measStd,
                                                   const Eigen::Matrix2d& covariance){
        // 1. 计算位置代价（马氏距离）
        Eigen::Vector2d posDiff;
        posDiff << (measBox.x - predBox.x), (measBox.y - predBox.y);
        double posCost = this->computeMahalanobisDistance(posDiff, covariance);
        
        // 2. 计算尺寸代价（归一化差异）
        double sizeDiffX = std::abs(measBox.x_width - predBox.x_width);
        double sizeDiffY = std::abs(measBox.y_width - predBox.y_width);
        double sizeDiffZ = std::abs(measBox.z_width - predBox.z_width);
        
        // 归一化尺寸差异（使用平均尺寸）
        double avgSizeX = (measBox.x_width + predBox.x_width) / 2.0;
        double avgSizeY = (measBox.y_width + predBox.y_width) / 2.0;
        double avgSizeZ = (measBox.z_width + predBox.z_width) / 2.0;
        
        double sizeCost = 0.0;
        if (avgSizeX > 1e-6) sizeCost += (sizeDiffX / avgSizeX) * (sizeDiffX / avgSizeX);
        if (avgSizeY > 1e-6) sizeCost += (sizeDiffY / avgSizeY) * (sizeDiffY / avgSizeY);
        if (avgSizeZ > 1e-6) sizeCost += (sizeDiffZ / avgSizeZ) * (sizeDiffZ / avgSizeZ);
        
        // 3. 计算点云标准差代价（反映形状变化）
        Eigen::Vector3d stdDiff = measStd - predStd;
        double stdCost = stdDiff.squaredNorm();
        
        // 加权组合所有代价
        double totalCost = this->associationPosCostWeight_ * posCost +
                          this->associationSizeCostWeight_ * sizeCost +
                          this->associationStdCostWeight_ * stdCost;
        
        return totalCost;
    }

    /*!
     * @brief 匈牙利算法求解最优分配问题
     * @param costMatrix 代价矩阵 [numCurr x numHist]
     * @param assignment 输出匹配结果，assignment[i] = j 表示第i个当前检测匹配到第j个历史轨迹，-1表示未匹配
     * 
     * 算法步骤：
     * 1. 行归约：每行减去该行最小值
     * 2. 列归约：每列减去该列最小值
     * 3. 贪婪匹配：优先选择代价小的匹配
     * 
     * 注意：这是简化版匈牙利算法，适用于大多数情况
     */
    void dynamicDetector::hungarianAlgorithm(const std::vector<std::vector<double>>& costMatrix, std::vector<int>& assignment){
        if (costMatrix.empty()){
            assignment.clear();
            return;
        }
        
        int numRows = costMatrix.size();
        int numCols = costMatrix[0].size();
        assignment.resize(numRows, -1);
        
        // 创建代价矩阵的副本用于修改
        std::vector<std::vector<double>> cost = costMatrix;
        
        // 1. 行归约，找到每行的最小值，即当前每个障碍物找到与其最合适的历史轨迹
        for (int i = 0; i < numRows; ++i){
            double minVal = *std::min_element(cost[i].begin(), cost[i].end()); 
            if (minVal < 1e8){ // 只处理有效代价，找0元素代价
                for (int j = 0; j < numCols; ++j){
                    cost[i][j] -= minVal;
                }
            }
        }
        
        // 2. 列归约，找到每列的最小值，即历史轨迹找到与其最合适的当前障碍物
        // 这两步下来确保行列都有最合适的"零"
        for (int j = 0; j < numCols; ++j){
            double minVal = 1e9;
            for (int i = 0; i < numRows; ++i){
                minVal = std::min(minVal, cost[i][j]);
            }
            if (minVal < 1e8){
                for (int i = 0; i < numRows; ++i){
                    cost[i][j] -= minVal;
                }
            }
        }
        
        // 3. 贪婪匹配（简化版）
        std::vector<bool> colUsed(numCols, false);
        std::vector<std::pair<double, std::pair<int, int>>> candidates;
        
        // 收集所有零代价的候选匹配
        for (int i = 0; i < numRows; ++i){
            for (int j = 0; j < numCols; ++j){
                if (cost[i][j] < 1e-6 && costMatrix[i][j] < 1e8){
                    candidates.push_back({costMatrix[i][j], {i, j}});
                }
            }
        }
        
        // 按原始代价排序
        std::sort(candidates.begin(), candidates.end());
        
        // 贪婪分配，遍历排序后的候选列表。如果某一对 (row, col) 对应的行和列都还没有被占用，就锁定这个匹配。
        std::vector<bool> rowUsed(numRows, false);
        for (const auto& candidate : candidates){
            int row = candidate.second.first;
            int col = candidate.second.second;
            
            if (!rowUsed[row] && !colUsed[col]){
                assignment[row] = col;
                rowUsed[row] = true;
                colUsed[col] = true;
            }
        }
    }


    // 使用卡尔曼滤波器并更新历史记录
    void dynamicDetector::kalmanFilterAndUpdateHist(const std::vector<int>& bestMatch){
        // --- 初始化临时容器 ---
        // 这些临时容器用于构建下一帧的跟踪历史和滤波器列表
        // 这样做可以避免在迭代过程中直接修改正在使用的成员变量
        std::vector<std::deque<onboardDetector::box3D>> boxHistTemp; 
        std::vector<std::deque<std::vector<Eigen::Vector3d>>> pcHistTemp;
        std::vector<std::deque<Eigen::Vector3d>> pcCenterHistTemp;
        std::vector<std::deque<Eigen::Vector3d>> pcStdHistTemp;
        std::vector<onboardDetector::kalman_filter> filtersTemp;
        
        // 为新出现的目标准备的空历史记录模板
        std::deque<onboardDetector::box3D> newSingleBoxHist;
        std::deque<std::vector<Eigen::Vector3d>> newSinglePcHist; 
        std::deque<Eigen::Vector3d> newSinglePcCenterHist;
        std::deque<Eigen::Vector3d> newSinglePcStdHist;
        onboardDetector::kalman_filter newFilter; // 为新目标准备的卡尔曼滤波器实例
        
        std::vector<onboardDetector::box3D> trackedBBoxesTemp; // 存储当前帧滤波后的所有目标框

        newSingleBoxHist.resize(0);
        newSinglePcHist.resize(0);
        newSinglePcCenterHist.resize(0);
        newSinglePcStdHist.resize(0);
        int numObjs = this->filteredBBoxes_.size(); // 当前帧检测到的目标数量

        // --- 遍历当前帧检测到的每一个目标 ---
        for (int i=0 ; i<numObjs ; i++){
            onboardDetector::box3D newEstimatedBBox; // 用于存储卡尔曼滤波后的状态

            // bestMatch[i] 存储的是当前第 i 个检测框所匹配到的历史轨迹的索引
            // 如果 bestMatch[i] >= 0，说明匹配成功
            if (bestMatch[i]>=0){
                // --- 情况1：目标匹配成功 (老目标) ---
                // 继承该目标之前的历史记录和滤波器
                boxHistTemp.push_back(this->boxHist_[bestMatch[i]]);
                pcHistTemp.push_back(this->pcHist_[bestMatch[i]]);
                pcCenterHistTemp.push_back(this->pcCenterHist_[bestMatch[i]]);
                pcStdHistTemp.push_back(this->pcStdHist_[bestMatch[i]]);
                filtersTemp.push_back(this->filters_[bestMatch[i]]);

                // 使用当前检测到的边界框作为测量值，来更新卡尔曼滤波器
                onboardDetector::box3D currDetectedBBox = this->filteredBBoxes_[i];

                // 根据当前测量值和历史信息，生成卡尔曼滤波器的观测向量 Z
                Eigen::MatrixXd Z;
                this->getKalmanObservationAcc(currDetectedBBox, bestMatch[i], Z);
                // 滤波器中最后一个放入的KF，执行的 estimate 步骤（预测+更新），控制输入 u 设为0
                filtersTemp.back().estimate(Z, MatrixXd::Zero(6,1));
                
                // 从滤波器中提取更新后的状态（位置、速度、加速度）
                newEstimatedBBox.x = filtersTemp.back().output(0);
                newEstimatedBBox.y = filtersTemp.back().output(1);
                newEstimatedBBox.z = currDetectedBBox.z; // Z轴位置直接使用测量值，不通过滤波
                newEstimatedBBox.Vx = filtersTemp.back().output(2);
                newEstimatedBBox.Vy = filtersTemp.back().output(3);
                newEstimatedBBox.Ax = filtersTemp.back().output(4);
                newEstimatedBBox.Ay = filtersTemp.back().output(5);   
                          
                // 边界框的尺寸直接使用当前测量值
                newEstimatedBBox.x_width = currDetectedBBox.x_width;
                newEstimatedBBox.y_width = currDetectedBBox.y_width;
                newEstimatedBBox.z_width = currDetectedBBox.z_width;
                // 继承其他标志位
                newEstimatedBBox.is_dynamic = currDetectedBBox.is_dynamic;
                newEstimatedBBox.is_human = currDetectedBBox.is_human;
            }
            else{
                // --- 情况2：目标未匹配 (新目标) ---
                // 为这个新目标创建全新的、空的轨迹历史
                boxHistTemp.push_back(newSingleBoxHist);
                pcHistTemp.push_back(newSinglePcHist);
                pcCenterHistTemp.push_back(newSinglePcCenterHist);
                pcStdHistTemp.push_back(newSinglePcStdHist);

                // 为这个新目标创建一个全新的卡尔曼滤波器
                onboardDetector::box3D currDetectedBBox = this->filteredBBoxes_[i];
                MatrixXd states, A, B, H, P, Q, R;    
                this->kalmanFilterMatrixAcc(currDetectedBBox, states, A, B, H, P, Q, R);
                
                newFilter.setup(states, A, B, H, P, Q, R);
                filtersTemp.push_back(newFilter);
                // 对于新目标，其初始估计状态就是它的第一次测量值
                newEstimatedBBox = currDetectedBBox;
            }

            // --- 更新历史记录队列 ---
            // 如果历史记录的长度达到了设定的最大值
            if (int(boxHistTemp[i].size()) == this->histSize_){
                // 从队列尾部移除最老的数据
                boxHistTemp[i].pop_back();
                pcHistTemp[i].pop_back();
                pcCenterHistTemp[i].pop_back();
                pcStdHistTemp[i].pop_back();
            }

            // 将当前帧的最新估计状态和信息从队列头部推入
            boxHistTemp[i].push_front(newEstimatedBBox); 
            pcHistTemp[i].push_front(this->filteredPcClusters_[i]);
            pcCenterHistTemp[i].push_front(this->filteredPcClusterCenters_[i]);
            pcStdHistTemp[i].push_front(this->filteredPcClusterStds_[i]);

            // 将当前帧的最终跟踪结果存入 trackedBBoxesTemp
            trackedBBoxesTemp.push_back(newEstimatedBBox);
        }
  
        // --- 后处理：稳定边界框尺寸 ---
        // 如果已经有跟踪历史
        if (boxHistTemp.size()){
            // 再次遍历所有当前帧的目标
            for (size_t i=0; i<trackedBBoxesTemp.size(); ++i){ 
                // 如果一个目标的跟踪历史足够长
                if (int(boxHistTemp[i].size()) >= this->fixSizeHistThresh_){
                    // 并且当前帧与上一帧的尺寸变化在阈值范围内（尺寸趋于稳定）
                    if ((abs(trackedBBoxesTemp[i].x_width-boxHistTemp[i][1].x_width)/boxHistTemp[i][1].x_width) <= this->fixSizeDimThresh_ &&
                        (abs(trackedBBoxesTemp[i].y_width-boxHistTemp[i][1].y_width)/boxHistTemp[i][1].y_width) <= this->fixSizeDimThresh_&&
                        (abs(trackedBBoxesTemp[i].z_width-boxHistTemp[i][1].z_width)/boxHistTemp[i][1].z_width) <= this->fixSizeDimThresh_){
                        // 则强制将当前尺寸设为上一帧的尺寸，以防止尺寸抖动
                        trackedBBoxesTemp[i].x_width = boxHistTemp[i][1].x_width;
                        trackedBBoxesTemp[i].y_width = boxHistTemp[i][1].y_width;
                        trackedBBoxesTemp[i].z_width = boxHistTemp[i][1].z_width;
                        // 同时更新历史记录中的最新值
                        boxHistTemp[i][0].x_width = trackedBBoxesTemp[i].x_width;
                        boxHistTemp[i][0].y_width = trackedBBoxesTemp[i].y_width;
                        boxHistTemp[i][0].z_width = trackedBBoxesTemp[i].z_width;
                    }

                }
            }
        }
        
        // --- 更新类的成员变量 ---
        // 用新构建的临时历史记录和滤波器列表，替换掉旧的成员变量
        this->boxHist_ = boxHistTemp;
        // std::cout << "this->boxHist_[1].size() = "  << this->boxHist_[1].size() << std::endl;
        this->pcHist_ = pcHistTemp;
        this->pcCenterHist_ = pcCenterHistTemp;
        this->pcStdHist_ = pcStdHistTemp;
        this->filters_ = filtersTemp;

        // 更新当前帧的最终跟踪结果
        this->trackedBBoxes_=  trackedBBoxesTemp;
    }

    // 设置速度模型的卡尔曼滤波器矩阵
    void dynamicDetector::kalmanFilterMatrixVel(const onboardDetector::box3D& currDetectedBBox, MatrixXd& states, MatrixXd& A, MatrixXd& B, MatrixXd& H, MatrixXd& P, MatrixXd& Q, MatrixXd& R){
        states.resize(4,1);
        states(0) = currDetectedBBox.x;
        states(1) = currDetectedBBox.y;
        // 将速度和加速度初始化为零
        states(2) = 0.;
        states(3) = 0.;

        MatrixXd ATemp;
        ATemp.resize(4, 4);
        ATemp <<  0, 0, 1, 0,
                  0, 0, 0, 1,
                  0, 0, 0, 0,
                  0 ,0, 0, 0;
        A = MatrixXd::Identity(4,4) + this->dt_*ATemp;
        B = MatrixXd::Zero(4, 4);
        H = MatrixXd::Identity(4, 4);
        P = MatrixXd::Identity(4, 4) * this->eP_;
        Q = MatrixXd::Identity(4, 4);
        Q(0,0) *= this->eQPos_; Q(1,1) *= this->eQPos_; Q(2,2) *= this->eQVel_; Q(3,3) *= this->eQVel_; 
        R = MatrixXd::Identity(4, 4);
        R(0,0) *= this->eRPos_; R(1,1) *= this->eRPos_; R(2,2) *= this->eRVel_; R(3,3) *= this->eRVel_;

    }

    // 设置加速度模型的卡尔曼滤波器矩阵
    void dynamicDetector::kalmanFilterMatrixAcc(const onboardDetector::box3D& currDetectedBBox, MatrixXd& states, MatrixXd& A, MatrixXd& B, MatrixXd& H, MatrixXd& P, MatrixXd& Q, MatrixXd& R){
        states.resize(6,1);
        states(0) = currDetectedBBox.x;
        states(1) = currDetectedBBox.y;
        // 将速度和加速度初始化为零
        states(2) = 0.;
        states(3) = 0.;
        states(4) = 0.;
        states(5) = 0.;

        MatrixXd ATemp;
        ATemp.resize(6, 6);

        ATemp <<  1, 0, this->dt_, 0, 0.5*pow(this->dt_, 2), 0,
                  0, 1, 0, this->dt_, 0, 0.5*pow(this->dt_, 2),
                  0, 0, 1, 0, this->dt_, 0,
                  0 ,0, 0, 1, 0, this->dt_,
                  0, 0, 0, 0, 1, 0,
                  0, 0, 0, 0, 0, 1;
        A = ATemp;
        B = MatrixXd::Zero(6, 6);
        H = MatrixXd::Identity(6, 6);
        P = MatrixXd::Identity(6, 6) * this->eP_;
        Q = MatrixXd::Identity(6, 6);
        Q(0,0) *= this->eQPos_; Q(1,1) *= this->eQPos_; Q(2,2) *= this->eQVel_; Q(3,3) *= this->eQVel_; Q(4,4) *= this->eQAcc_; Q(5,5) *= this->eQAcc_;
        R = MatrixXd::Identity(6, 6);
        R(0,0) *= this->eRPos_; R(1,1) *= this->eRPos_; R(2,2) *= this->eRVel_; R(3,3) *= this->eRVel_; R(4,4) *= this->eRAcc_; R(5,5) *= this->eRAcc_;
    }

    // 获取速度模型的卡尔曼滤波器观测值
    void dynamicDetector::getKalmanObservationVel(const onboardDetector::box3D& currDetectedBBox, int bestMatchIdx, MatrixXd& Z){
        Z.resize(4,1);
        Z(0) = currDetectedBBox.x; 
        Z(1) = currDetectedBBox.y;

        // 使用前k帧进行速度估计
        int k = this->kfAvgFrames_;
        int historySize = this->boxHist_[bestMatchIdx].size();
        if (historySize < k){
            k = historySize;
        }
        onboardDetector::box3D prevMatchBBox = this->boxHist_[bestMatchIdx][k-1];

        Z(2) = (currDetectedBBox.x-prevMatchBBox.x)/(this->dt_*k);
        Z(3) = (currDetectedBBox.y-prevMatchBBox.y)/(this->dt_*k);
    }

    // 获取加速度模型的卡尔曼滤波器观测值
    void dynamicDetector::getKalmanObservationAcc(const onboardDetector::box3D& currDetectedBBox, int bestMatchIdx, MatrixXd& Z){
        Z.resize(6, 1);
        Z(0) = currDetectedBBox.x;
        Z(1) = currDetectedBBox.y;

        // 计算用于速度估计的历史帧数，不能超过预设的平均帧数和实际历史大小的最小值
        int k = this->kfAvgFrames_;
        int historySize = this->boxHist_[bestMatchIdx].size();
        if (historySize < k){
            k = historySize;
        }
        onboardDetector::box3D prevMatchBBox = this->boxHist_[bestMatchIdx][k-1];

        Z(2) = (currDetectedBBox.x - prevMatchBBox.x)/(this->dt_*k);
        Z(3) = (currDetectedBBox.y - prevMatchBBox.y)/(this->dt_*k);
        Z(4) = (Z(2) - prevMatchBBox.Vx)/(this->dt_*k);
        Z(5) = (Z(3) - prevMatchBBox.Vy)/(this->dt_*k);
    }
 
    // 获取动态点云
    void dynamicDetector::getDynamicPc(std::vector<Eigen::Vector3d>& dynamicPc){
        Eigen::Vector3d curPoint;
        for (size_t i=0; i<this->filteredPcClusters_.size(); ++i){
            for (size_t j=0; j<this->filteredPcClusters_[i].size(); ++j){
                curPoint = this->filteredPcClusters_[i][j];
                for (size_t k=0; k<this->dynamicBBoxes_.size(); ++k){
                    if (abs(curPoint(0)-this->dynamicBBoxes_[k].x)<=this->dynamicBBoxes_[k].x_width/2 and 
                        abs(curPoint(1)-this->dynamicBBoxes_[k].y)<=this->dynamicBBoxes_[k].y_width/2 and 
                        abs(curPoint(2)-this->dynamicBBoxes_[k].z)<=this->dynamicBBoxes_[k].z_width/2) {
                        dynamicPc.push_back(curPoint);
                        break;
                    }
                }
            }
        }
    } 
    

    // 发布点云
    void dynamicDetector::publishPoints(const std::vector<Eigen::Vector3d>& points, const ros::Publisher& publisher){
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;        
        for (size_t i=0; i<points.size(); ++i){
            pt.x = points[i](0);
            pt.y = points[i](1);
            pt.z = points[i](2);
            cloud.push_back(pt);
        }    
        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = "map";

        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg(cloud, cloudMsg);
        publisher.publish(cloudMsg);
    }


    // 发布3D边界框
    void dynamicDetector::publish3dBox(const std::vector<box3D>& boxes,
                                   const ros::Publisher& publisher,
                                   double r, double g, double b){
        // 创建一个MarkerArray消息，用于批量发布多个Marker
        visualization_msgs::MarkerArray markers;

        // 遍历所有传入的边界框
        for (size_t i = 0; i < boxes.size(); i++)
        {
            // 为每个边界框创建一个LINE_LIST类型的Marker
            visualization_msgs::Marker line;
            line.header.frame_id = "map"; // 设置Marker的坐标系为"map"
            line.ns = "box3D"; // 设置Marker的命名空间
            line.id = i; // 为Marker设置唯一的ID
            line.type = visualization_msgs::Marker::LINE_LIST; // Marker类型为线列表，用于绘制立方体的边
            line.action = visualization_msgs::Marker::ADD; // 操作类型为添加或修改
            line.scale.x = 0.06; // 设置线的宽度
            
            // 设置线的颜色和透明度
            line.color.r = r;
            line.color.g = g;
            line.color.b = b;
            line.color.a = 1.0;

            line.lifetime = ros::Duration(0.05); // Marker的生命周期，0.05秒后会自动消失
            
            // 设置Marker的姿态，这里表示无旋转
            line.pose.orientation.x = 0.0;
            line.pose.orientation.y = 0.0;
            line.pose.orientation.z = 0.0;
            line.pose.orientation.w = 1.0;

            // 设置Marker的中心位置
            line.pose.position.x = boxes[i].x;
            line.pose.position.y = boxes[i].y;
            
            // 获取边界框的宽度和长度
            double x_width = boxes[i].x_width;
            double y_width = boxes[i].y_width;

            // --- 计算Marker在Z轴上的位置和高度 ---
            // 这里的计算方式似乎是为了让边界框的底部接触地面（z=0）
            double top = boxes[i].z + boxes[i].z_width / 2.0; // 计算边界框的最高点Z值
            double z_width = top / 2.0; // 将可视化Marker的高度设为最高点的一半
            line.pose.position.z = z_width; // 将可视化Marker的中心Z坐标设为该值

            // 定义立方体的8个顶点
            geometry_msgs::Point corner[8];
            corner[0].x = -x_width / 2.0; corner[0].y = -y_width / 2.0; corner[0].z = -z_width;
            corner[1].x = -x_width / 2.0; corner[1].y =  y_width / 2.0; corner[1].z = -z_width;
            corner[2].x =  x_width / 2.0; corner[2].y =  y_width / 2.0; corner[2].z = -z_width;
            corner[3].x =  x_width / 2.0; corner[3].y = -y_width / 2.0; corner[3].z = -z_width;

            corner[4].x = -x_width / 2.0; corner[4].y = -y_width / 2.0; corner[4].z =  z_width;
            corner[5].x = -x_width / 2.0; corner[5].y =  y_width / 2.0; corner[5].z =  z_width;
            corner[6].x =  x_width / 2.0; corner[6].y =  y_width / 2.0; corner[6].z =  z_width;
            corner[7].x =  x_width / 2.0; corner[7].y = -y_width / 2.0; corner[7].z =  z_width;

            // 定义连接8个顶点的12条边
            int edgeIdx[12][2] = {
                {0,1}, {1,2}, {2,3}, {3,0},  // 底部四条边
                {4,5}, {5,6}, {6,7}, {7,4},  // 顶部四条边
                {0,4}, {1,5}, {2,6}, {3,7}   // 连接上下面的四条垂直边
            };

            // 将12条边的端点添加到Marker的点列表中
            for (int e = 0; e < 12; e++)
            {
                line.points.push_back(corner[edgeIdx[e][0]]);
                line.points.push_back(corner[edgeIdx[e][1]]);
            }

            // 将配置好的Marker添加到MarkerArray中
            markers.markers.push_back(line);
        }

        // 通过发布器将整个MarkerArray发布出去
        publisher.publish(markers);
    }


    // 发布历史轨迹
    void dynamicDetector::publishHistoryTraj(){
        visualization_msgs::MarkerArray trajMsg;
        int countMarker = 0;
        for (size_t i=0; i<this->boxHist_.size(); ++i){
            // std::cout << "this->boxHist_[i].size() = "  << this->boxHist_[i].size() << std::endl;
            if (this->boxHist_[i].size() > 1){
                visualization_msgs::Marker traj;
                traj.header.frame_id = "map";
                traj.header.stamp = ros::Time::now();
                traj.ns = "dynamic_detector";
                traj.id = countMarker;
                traj.type = visualization_msgs::Marker::LINE_LIST;
                traj.scale.x = 0.03;
                traj.scale.y = 0.03;
                traj.scale.z = 0.03;
                traj.color.a = 1.0; // Don't forget to set the alpha!
                traj.color.r = 0.0;
                traj.color.g = 1.0;
                traj.color.b = 0.0;
                traj.pose.orientation.w = 1.0;
                traj.pose.orientation.x = 0.0;
                traj.pose.orientation.y = 0.0;
                traj.pose.orientation.z = 0.0;
                for (size_t j=0; j<this->boxHist_[i].size()-1; ++j){
                    geometry_msgs::Point p1, p2;
                    onboardDetector::box3D box1 = this->boxHist_[i][j];
                    onboardDetector::box3D box2 = this->boxHist_[i][j+1];
                    p1.x = box1.x; p1.y = box1.y; p1.z = box1.z;
                    p2.x = box2.x; p2.y = box2.y; p2.z = box2.z;
                    traj.points.push_back(p1);
                    traj.points.push_back(p2);
                }

                ++countMarker;
                trajMsg.markers.push_back(traj);
            }
        }
        this->historyTrajPub_.publish(trajMsg);
    }

    // 发布所有被跟踪对象的速度可视化信息
    void dynamicDetector::publishVelVis(){ 
        visualization_msgs::MarkerArray velVisMsg;
        int countMarker = 0;
        for (size_t i=0; i<this->trackedBBoxes_.size(); ++i){
            visualization_msgs::Marker velMarker;
            velMarker.header.frame_id = "map";
            velMarker.header.stamp = ros::Time::now();
            velMarker.ns = "dynamic_detector";
            velMarker.id =  countMarker;
            velMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            velMarker.pose.position.x = this->trackedBBoxes_[i].x;
            velMarker.pose.position.y = this->trackedBBoxes_[i].y;
            velMarker.pose.position.z = this->trackedBBoxes_[i].z + this->trackedBBoxes_[i].z_width/2. + 0.3;
            velMarker.scale.x = 0.15;
            velMarker.scale.y = 0.15;
            velMarker.scale.z = 0.15;
            velMarker.color.a = 1.0;
            velMarker.color.r = 1.0;
            velMarker.color.g = 0.0;
            velMarker.color.b = 0.0;
            velMarker.lifetime = ros::Duration(0.1);
            double vx = this->trackedBBoxes_[i].Vx;
            double vy = this->trackedBBoxes_[i].Vy;
            double vNorm = sqrt(vx*vx+vy*vy);
            std::string velText = "Vx=" + std::to_string(vx) + ", Vy=" + std::to_string(vy) + ", |V|=" + std::to_string(vNorm);
            velMarker.text = velText;
            velVisMsg.markers.push_back(velMarker);
            ++countMarker;
        }
        this->velVisPub_.publish(velVisMsg);
    }

    // 发布激光雷达聚类
    void dynamicDetector::publishLidarClusters(){
        sensor_msgs::PointCloud2 lidarClustersMsg;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (size_t i=0; i<this->lidarClusters_.size(); ++i){
            onboardDetector::Cluster & cluster = this->lidarClusters_[i];

            std_msgs::ColorRGBA color;
            srand(cluster.cluster_id);
            color.r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            color.g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            color.b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            // color.r = 0.5;
            // color.g = 0.5;
            // color.b = 0.5;
            // color.a = 1.0;

            for (size_t j=0; j<cluster.points->size(); ++j){
                pcl::PointXYZRGB point;
                const pcl::PointXYZ & pt = cluster.points->at(j);
                point.x = pt.x;
                point.y = pt.y;
                point.z = pt.z;
                point.r = color.r * 255;
                point.g = color.g * 255;
                point.b = color.b * 255;
                colored_cloud->push_back(point);
            }
        }
        pcl::toROSMsg(*colored_cloud, lidarClustersMsg);
        lidarClustersMsg.header.frame_id = "map";
        lidarClustersMsg.header.stamp = ros::Time::now();
        this->lidarClustersPub_.publish(lidarClustersMsg);
    }

    // 发布过滤后的点
    void dynamicDetector::publishFilteredPoints(){
        sensor_msgs::PointCloud2 filteredPointsMsg;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (size_t i=0; i<this->filteredPcClusters_.size(); ++i){
            std_msgs::ColorRGBA color;
            color.r = 0.5;
            color.g = 0.5;
            color.b = 0.5;
            color.a = 1.0;

            for (size_t j=0; j<this->filteredPcClusters_[i].size(); ++j){
                pcl::PointXYZRGB point;
                point.x = this->filteredPcClusters_[i][j](0);
                point.y = this->filteredPcClusters_[i][j](1);
                point.z = this->filteredPcClusters_[i][j](2);
                point.r = color.r * 255;
                point.g = color.g * 255;
                point.b = color.b * 255;
                colored_cloud->push_back(point);
            }
        }
        pcl::toROSMsg(*colored_cloud, filteredPointsMsg);
        filteredPointsMsg.header.frame_id = "map";
        filteredPointsMsg.header.stamp = ros::Time::now();
        this->filteredPointsPub_.publish(filteredPointsMsg);
    }

    // 发布原始动态点
    void dynamicDetector::publishRawDynamicPoints(){
        // 检查是否有最新的点云数据，如果没有则直接返回
        if (not this->latestCloud_){
            return;
        }
        try {
            // 创建一个PCL点云指针，用于存储全局坐标系下的点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            // 检查是否有传感器位姿信息
            if (this->hasSensorPose_) {
                // 如果有位姿，则将ROS消息格式的原始点云转换到PCL格式
                pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::fromROSMsg(*this->latestCloud_, *tempCloud);
                
                // 创建一个仿射变换矩阵，用于将点云从激光雷达坐标系转换到全局（map）坐标系
                Eigen::Affine3d transform = Eigen::Affine3d::Identity();
                transform.linear() = this->orientationLidar_; // 设置旋转部分
                transform.translation() = this->positionLidar_; // 设置平移部分
                
                // 对点云应用变换
                pcl::transformPointCloud(*tempCloud, *globalCloud, transform);

                // 创建一个ROS点云消息用于发布可视化
                sensor_msgs::PointCloud2 cloudMsg;
                pcl::toROSMsg(*globalCloud, cloudMsg);
                cloudMsg.header.frame_id = "map"; // 设置坐标系为 "map"
                cloudMsg.header.stamp = ros::Time::now(); // 设置时间戳
                this->rawLidarPointsPub_.publish(cloudMsg); // 发布转换到全局坐标系的原始点云
            
                // 终端输出测试：打印发布点云信息到ROS日志/终端
                // std::cout << ": No time step parameter found. Use default: 0.033." << std::endl;
                // ROS_INFO_STREAM(this->hint_ << " publishRawDynamicPoints: transformed pointcloud published. "
                //                 << "hasSensorPose=" << (this->hasSensorPose_ ? "true" : "false")
                //                 << ", points=" << globalCloud->points.size());
            }
            else {
                // 如果没有位姿信息，直接将ROS消息转换为PCL点云（假设其已在全局坐标系）
                pcl::fromROSMsg(*this->latestCloud_, *globalCloud);
            }
            
            // 创建一个向量，用于存储属于动态障碍物的点的坐标
            std::vector<Eigen::Vector3d> dynamicEigenPoints;
            
            // 遍历所有已识别的动态边界框
            for (const auto& box : this->dynamicBBoxes_) {
                // 如果该边界框未被标记为动态，则跳过
                if (!box.is_dynamic)
                    continue;
                
                // 计算边界框的最小和最大坐标
                double xmin = box.x - box.x_width / 2.0;
                double xmax = box.x + box.x_width / 2.0;
                double ymin = box.y - box.y_width / 2.0;
                double ymax = box.y + box.y_width / 2.0;
                double zmin = box.z - box.z_width / 2.0;
                double zmax = box.z + box.z_width / 2.0;
                
                // 遍历全局点云中的每一个点
                for (const auto& point : globalCloud->points) {
                    // 检查点是否在当前动态边界框内部
                    if (point.x >= xmin && point.x <= xmax &&
                        point.y >= ymin && point.y <= ymax &&
                        point.z >= zmin && point.z <= zmax)
                    {
                        // 如果点在框内，则将其添加到动态点向量中
                        dynamicEigenPoints.push_back(Eigen::Vector3d(point.x, point.y, point.z));
                    }
                }
            }
            
            // 如果没有找到任何动态点，则直接返回
            if (dynamicEigenPoints.empty()) {
                return;
            }
            
            // 调用publishPoints函数，将提取出的动态点云发布出去
            this->publishPoints(dynamicEigenPoints, this->rawDynamicPointsPub_);
        }
        // 捕获并报告PCL库可能抛出的异常
        catch (const pcl::PCLException& e) {
            ROS_ERROR("PCL Exception during dynamic point extraction: %s", e.what());
        }
        // 捕获并报告标准C++库可能抛出的异常
        catch (const std::exception& e) {
            ROS_ERROR("Standard Exception during dynamic point extraction: %s", e.what());
        }
        // 捕获所有其他类型的未知异常
        catch (...) {
            ROS_ERROR("Unknown error during dynamic point extraction.");
        }
    }

    // 用户函数：获取动态障碍物
    void dynamicDetector::getDynamicObstacles(std::vector<onboardDetector::box3D>& incomeDynamicBBoxes, const Eigen::Vector3d &robotSize){
        incomeDynamicBBoxes.clear();
        for (int i=0; i<int(this->dynamicBBoxes_.size()); i++){
            onboardDetector::box3D box = this->dynamicBBoxes_[i];
            box.x_width += robotSize(0);
            box.y_width += robotSize(1);
            box.z_width += robotSize(2);
            incomeDynamicBBoxes.push_back(box);
        }
    }

    // 用户函数：获取动态障碍物历史
    void dynamicDetector::getDynamicObstaclesHist(std::vector<std::vector<Eigen::Vector3d>>& posHist, std::vector<std::vector<Eigen::Vector3d>>& velHist, std::vector<std::vector<Eigen::Vector3d>>& sizeHist, const Eigen::Vector3d &robotSize){
		posHist.clear();
        velHist.clear();
        sizeHist.clear();

        if (this->boxHist_.size()){
            for (size_t i=0 ; i<this->boxHist_.size() ; ++i){
                if (this->boxHist_[i][0].is_dynamic or this->boxHist_[i][0].is_human){   
                    bool findMatch = false;     
                    if (this->constrainSize_){
                        for (Eigen::Vector3d targetSize : this->targetObjectSize_){
                            double xdiff = std::abs(this->boxHist_[i][0].x_width - targetSize(0));
                            double ydiff = std::abs(this->boxHist_[i][0].y_width - targetSize(1));
                            double zdiff = std::abs(this->boxHist_[i][0].z_width - targetSize(2)); 
                            if (xdiff < 0.8 and ydiff < 0.8 and zdiff < 1.0){
                                findMatch = true;
                            }
                        }
                    }
                    else{
                        findMatch = true;
                    }
                    if (findMatch){
                        std::vector<Eigen::Vector3d> obPosHist, obVelHist, obSizeHist;
                        for (size_t j=0; j<this->boxHist_[i].size() ; ++j){
                            Eigen::Vector3d pos(this->boxHist_[i][j].x, this->boxHist_[i][j].y, this->boxHist_[i][j].z);
                            Eigen::Vector3d vel(this->boxHist_[i][j].Vx, this->boxHist_[i][j].Vy, 0);
                            Eigen::Vector3d size(this->boxHist_[i][j].x_width, this->boxHist_[i][j].y_width, this->boxHist_[i][j].z_width);
                            size += robotSize;
                            obPosHist.push_back(pos);
                            obVelHist.push_back(vel);
                            obSizeHist.push_back(size);
                        }
                        posHist.push_back(obPosHist);
                        velHist.push_back(obVelHist);
                        sizeHist.push_back(obSizeHist);
                    }
                }
            }
        }
	}

    // 将Livox CustomMsg格式转换为PointCloud2格式
    void dynamicDetector::convertCustomMsgToPointCloud2(const livox_ros_driver2::CustomMsgConstPtr& customMsg, sensor_msgs::PointCloud2& cloud){
        // auto start_time = std::chrono::high_resolution_clock::now();
        
        // 设置PointCloud2的基本信息
        cloud.header = customMsg->header;
        cloud.height = 1;
        cloud.width = customMsg->point_num;
        cloud.is_bigendian = false;
        cloud.is_dense = false;
        
        // 定义PointCloud2的字段
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(customMsg->point_num);
        
        // 创建迭代器访问xyz字段
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        
        // 将CustomMsg中的点转换到PointCloud2
        for (size_t i = 0; i < customMsg->points.size(); ++i){
            const auto& point = customMsg->points[i];
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
        
        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // ROS_INFO_THROTTLE(1.0, "%s: CustomMsg to PointCloud2 conversion took %.3f ms for %u points", 
        //                   this->hint_.c_str(), duration.count() / 1000.0, customMsg->point_num);
    }

    // Livox CustomMsg + Pose 回调函数
    void dynamicDetector::lidarCustomPoseCB(const livox_ros_driver2::CustomMsgConstPtr& customMsg, const geometry_msgs::PoseStampedConstPtr& pose){
        // // [Performance Timing] 测量回调函数耗时
        // auto start_time = std::chrono::high_resolution_clock::now();
        
        // 将CustomMsg转换为PointCloud2
        sensor_msgs::PointCloud2 cloudMsg;
        this->convertCustomMsgToPointCloud2(customMsg, cloudMsg);
        
        // 转换为ConstPtr并调用原有的处理函数
        sensor_msgs::PointCloud2ConstPtr cloudMsgPtr = boost::make_shared<sensor_msgs::PointCloud2>(cloudMsg);
        this->lidarPoseCB(cloudMsgPtr, pose);
        
        // // [Performance Timing] 输出耗时
        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // ROS_INFO_THROTTLE(1.0, "%s: lidarCustomPoseCB took %.3f ms", this->hint_.c_str(), duration.count() / 1000.0);
    }

    // Livox CustomMsg + Odometry 回调函数
    void dynamicDetector::lidarCustomOdomCB(const livox_ros_driver2::CustomMsgConstPtr& customMsg, const nav_msgs::OdometryConstPtr& odom){
        // // [Performance Timing] 测量回调函数耗时
        // auto start_time = std::chrono::high_resolution_clock::now();
        
        // 将CustomMsg转换为PointCloud2
        sensor_msgs::PointCloud2 cloudMsg;
        this->convertCustomMsgToPointCloud2(customMsg, cloudMsg);
        
        // 转换为ConstPtr并调用原有的处理函数
        sensor_msgs::PointCloud2ConstPtr cloudMsgPtr = boost::make_shared<sensor_msgs::PointCloud2>(cloudMsg);
        this->lidarOdomCB(cloudMsgPtr, odom);
        
        // // [Performance Timing] 输出耗时
        // auto end_time = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // ROS_INFO_THROTTLE(1.0, "%s: lidarCustomOdomCB took %.3f ms", this->hint_.c_str(), duration.count() / 1000.0);
    }
}