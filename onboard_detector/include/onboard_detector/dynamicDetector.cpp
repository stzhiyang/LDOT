/*
    FILE: dynamicDetector.cpp
    ---------------------------------
    function implementation of dynamic osbtacle detector
*/
#include <cmath> // for std::isfinite
#include <onboard_detector/dynamicDetector.h>

namespace onboardDetector {
// 默认构造函数
dynamicDetector::dynamicDetector() {
  this->ns_ = "onboard_detector";
  this->hint_ = "[onboardDetector]";
}

// 带节点句柄的构造函数
dynamicDetector::dynamicDetector(const ros::NodeHandle &nh) {
  this->ns_ = "onboard_detector";
  this->hint_ = "[onboardDetector]";
  this->nh_ = nh;
  this->initParam();
  this->registerPub();
  this->registerCallback();
}

// 初始化检测器
void dynamicDetector::initDetector(const ros::NodeHandle &nh) {
  this->nh_ = nh;
  this->initParam();
  this->registerPub();
  this->registerCallback();
}

// 初始化参数
void dynamicDetector::initParam() {
  // ---------------------------------获取ros话题---------------------------------------
  // localization mode
  if (not this->nh_.getParam(this->ns_ + "/localization_mode",
                             this->localizationMode_)) {
    this->localizationMode_ = 0;
    cout << this->hint_ << ": No localization mode option. Use default: pose"
         << endl;
  } else {
    cout << this->hint_
         << ": Localizaiton mode: pose (0)/odom (1). Your option: "
         << this->localizationMode_ << endl;
  }

  // use livox custom msg
  if (not this->nh_.getParam(this->ns_ + "/use_livox_custom_msg",
                             this->useLivoxCustomMsg_)) {
    this->useLivoxCustomMsg_ = false;
    cout << this->hint_
         << ": No use_livox_custom_msg parameter. Use default: false "
            "(PointCloud2)"
         << endl;
  } else {
    cout << this->hint_ << ": Use Livox CustomMsg: "
         << (this->useLivoxCustomMsg_ ? "true" : "false") << endl;
  }

  // lidar topic name
  if (not this->nh_.getParam(this->ns_ + "/lidar_pointcloud_topic",
                             this->lidarTopicName_)) {
    this->lidarTopicName_ = "/cloud_registered";
    cout << this->hint_
         << ": No lidar pointcloud topic name. Use default: /cloud_registered"
         << endl;
  } else {
    cout << this->hint_ << ": Lidar pointcloud topic: " << this->lidarTopicName_
         << endl;
  }

  if (this->localizationMode_ == 0) {
    // odom topic name
    if (not this->nh_.getParam(this->ns_ + "/pose_topic",
                               this->poseTopicName_)) {
      this->poseTopicName_ = "/CERLAB/quadcopter/pose";
      cout << this->hint_
           << ": No pose topic name. Use default: /CERLAB/quadcopter/pose"
           << endl;
    } else {
      cout << this->hint_ << ": Pose topic: " << this->poseTopicName_ << endl;
    }
  }

  if (this->localizationMode_ == 1) {
    // pose topic name
    if (not this->nh_.getParam(this->ns_ + "/odom_topic",
                               this->odomTopicName_)) {
      this->odomTopicName_ = "/CERLAB/quadcopter/odom";
      cout << this->hint_
           << ": No odom topic name. Use default: /CERLAB/quadcopter/odom"
           << endl;
    } else {
      cout << this->hint_ << ": Odom topic: " << this->odomTopicName_ << endl;
    }
  }

  // --------------------------------------坐标系转换参数（外参）--------------------------------------------
  // transform matrix: body to lidar
  std::vector<double> body2LidarVec(16);
  if (not this->nh_.getParam(this->ns_ + "/body_to_lidar", body2LidarVec)) {
    ROS_ERROR("[dynamicDetector]: Please check body to lidar matrix!");
  } else {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        this->body2Lidar_(i, j) = body2LidarVec[i * 4 + j];
      }
    }
  }

  // --------------------------------------系统运行的频率（时间步长）-----------------------------------------
  if (not this->nh_.getParam(this->ns_ + "/time_step", this->dt_)) {
    this->dt_ = 0.033;
    std::cout << this->hint_
              << ": No time step parameter found. Use default: 0.033."
              << std::endl;
  } else {
    std::cout << this->hint_
              << ": Time step for the system is set to: " << this->dt_
              << std::endl;
  }

  // --------------------------------------DBSCAN通用参数--------------------------------------------------
  // 地面高度
  if (not this->nh_.getParam(this->ns_ + "/ground_height",
                             this->groundHeight_)) {
    this->groundHeight_ = 0.1;
    std::cout << this->hint_
              << ": No ground height parameter. Use default: 0.1m."
              << std::endl;
  } else {
    std::cout << this->hint_
              << ": Ground height is set to: " << this->groundHeight_
              << std::endl;
  }

  // roof height  天花板高度
  if (not this->nh_.getParam(this->ns_ + "/roof_height", this->roofHeight_)) {
    this->roofHeight_ = 2.0;
    std::cout << this->hint_ << ": No roof height parameter. Use default: 2.0m."
              << std::endl;
  } else {
    std::cout << this->hint_ << ": Roof height is set to: " << this->roofHeight_
              << std::endl;
  }

  // lidar detection range
  std::vector<double> detectionRange;
  if (not this->nh_.getParam(this->ns_ + "/lidar_detection_range",
                             detectionRange)) {
    this->localLidarRange_ = Eigen::Vector3d(10.0, 10.0, 3.0);
    std::cout << this->hint_
              << ": No lidar detection range parameter. Use default: [10.0, "
                 "10.0, 3.0]."
              << std::endl;
  } else {
    if (detectionRange.size() == 3) {
      this->localLidarRange_ = Eigen::Vector3d(
          detectionRange[0], detectionRange[1], detectionRange[2]);
      std::cout << this->hint_ << ": Lidar detection range is set to: ["
                << this->localLidarRange_.x() << ", "
                << this->localLidarRange_.y() << ", "
                << this->localLidarRange_.z() << "]" << std::endl;
    } else {
      this->localLidarRange_ = Eigen::Vector3d(10.0, 10.0, 3.0);
      std::cout << this->hint_
                << ": Invalid lidar detection range size. Use default: [10.0, "
                   "10.0, 3.0]."
                << std::endl;
    }
  }

  // --------------------------------------激光DBSCAN聚类参数---------------------------------------------------------
  // lidar dbscan min points
  if (not this->nh_.getParam(this->ns_ + "/lidar_DBSCAN_min_points",
                             this->lidarDBMinPoints_)) {
    this->lidarDBMinPoints_ = 10;
    cout << this->hint_
         << ": No lidar DBSCAN minimum point in each cluster parameter. Use "
            "default: 10."
         << endl;
  } else {
    cout << this->hint_
         << ": Lidar DBSCAN Minimum point in each cluster is set to: "
         << this->lidarDBMinPoints_ << endl;
  }

  // lidar dbscan search range
  if (not this->nh_.getParam(this->ns_ + "/lidar_DBSCAN_epsilon",
                             this->lidarDBEpsilon_)) {
    this->lidarDBEpsilon_ = 0.2;
    cout << this->hint_
         << ": No lidar DBSCAN epsilon parameter. Use default: 0.5." << endl;
  } else {
    cout << this->hint_
         << ": Lidar DBSCAN epsilon is set to: " << this->lidarDBEpsilon_
         << endl;
  }

  // lidar dbscan use adaptive epsilon
  if (not this->nh_.getParam(this->ns_ + "/lidar_DBSCAN_use_adaptive",
                             this->lidarDBUseAdaptive_)) {
    this->lidarDBUseAdaptive_ = false;
    cout << this->hint_
         << ": No lidar DBSCAN use adaptive parameter. Use default: false."
         << endl;
  } else {
    cout << this->hint_ << ": Lidar DBSCAN use adaptive is set to: "
         << (this->lidarDBUseAdaptive_ ? "true" : "false") << endl;
  }

  // lidar dbscan distance scale for adaptive epsilon
  if (not this->nh_.getParam(this->ns_ + "/lidar_DBSCAN_distance_scale",
                             this->lidarDBDistanceScale_)) {
    this->lidarDBDistanceScale_ = 0.05;
    cout << this->hint_
         << ": No lidar DBSCAN distance scale parameter. Use default: 0.05."
         << endl;
  } else {
    cout << this->hint_ << ": Lidar DBSCAN distance scale is set to: "
         << this->lidarDBDistanceScale_ << endl;
  }

  // -------------------------------------------目标跟踪与数据关联参数--------------------------------------------------
  // 读取关联置信度（默认 0.99）
  if (not this->nh_.getParam(this->ns_ + "/association_gate_confidence",
                             this->associationGateConfidence_)) {
    this->associationGateConfidence_ = 0.99;
    cout << this->hint_
         << ": No association_gate_confidence param, use default 0.99." << endl;
  } else {
    cout << this->hint_ << ": Association gate confidence set to: "
         << this->associationGateConfidence_ << endl;
  }
  // 根据置信度计算 3D 门限（卡方分布）
  {
    boost::math::chi_squared_distribution<double> chi2_3d(3);
    this->gateThreshold3D_ =
        boost::math::quantile(chi2_3d, this->associationGateConfidence_);
    cout << this->hint_ << ": gateThreshold3D = " << this->gateThreshold3D_
         << endl;
  }

  // 位置代价权重
  if (not this->nh_.getParam(this->ns_ + "/association_pos_cost_weight",
                             this->associationPosCostWeight_)) {
    this->associationPosCostWeight_ = 1.0;
    cout << this->hint_
         << ": No position cost weight parameter found. Use default: 1.0."
         << endl;
  } else {
    cout << this->hint_ << ": Position cost weight is set to: "
         << this->associationPosCostWeight_ << "." << endl;
  }

  // IoU代价权重
  if (not this->nh_.getParam(this->ns_ + "/association_iou_cost_weight",
                             this->associationIoUCostWeight_)) {
    this->associationIoUCostWeight_ = 0.4;
    cout << this->hint_
         << ": No IoU cost weight parameter found. Use default: 0.4." << endl;
  } else {
    cout << this->hint_
         << ": IoU cost weight is set to: " << this->associationIoUCostWeight_
         << "." << endl;
  }

  // tracking history size
  if (not this->nh_.getParam(this->ns_ + "/history_size", this->histSize_)) {
    this->histSize_ = 5;
    std::cout << this->hint_
              << ": No tracking history size parameter found. Use default: 5."
              << std::endl;
  } else {
    std::cout << this->hint_
              << ": History for tracking is set to: " << this->histSize_
              << std::endl;
  }

  // max missed frames
  if (not this->nh_.getParam(this->ns_ + "/max_missed_frames",
                             this->maxMissedFrames_)) {
    this->maxMissedFrames_ = 5;
    std::cout << this->hint_ << ": No max_missed_frames param. Use default: 5."
              << std::endl;
  } else {
    std::cout << this->hint_
              << ": Max missed frames is set to: " << this->maxMissedFrames_
              << std::endl;
  }

  // duplicate track IoU threshold
  if (not this->nh_.getParam(this->ns_ + "/duplicate_track_iou_threshold",
                             this->duplicateTrackIoUThreshold_)) {
    this->duplicateTrackIoUThreshold_ = 0.5;
    std::cout << this->hint_
              << ": No duplicate_track_iou_threshold param. Use default: 0.5"
              << std::endl;
  } else {
    std::cout << this->hint_ << ": Duplicate track IoU threshold is set to: "
              << this->duplicateTrackIoUThreshold_ << std::endl;
  }

  if (not this->nh_.getParam(this->ns_ + "/classification_interval_sec",
                             this->classificationIntervalSec_)) {
    this->classificationIntervalSec_ = 3.0; // 设置默认值，比如1秒
    ROS_WARN_STREAM(
        this->hint_
        << " No classification_interval_sec param. Use default: 1.0");
  } else {
    ROS_INFO_STREAM(this->hint_ << " classification_interval_sec: "
                                << this->classificationIntervalSec_);
  }

  //-------------------------------------动态/静态分类参数----------------------------------------------------
  // skip frame for classification
  if (not this->nh_.getParam(this->ns_ + "/frame_skip", this->skipFrame_)) {
    this->skipFrame_ = 5;
    std::cout << this->hint_
              << ": No skip frame parameter found. Use default: 5."
              << std::endl;
  } else {
    std::cout << this->hint_
              << ": Frames skiped in classification when comparing two point "
                 "cloud is set to: "
              << this->skipFrame_ << std::endl;
  }

  // velocity threshold for dynamic classification
  if (not this->nh_.getParam(this->ns_ + "/dynamic_velocity_threshold",
                             this->dynaVelThresh_)) {
    this->dynaVelThresh_ = 0.35;
    std::cout
        << this->hint_
        << ": No dynamic velocity threshold parameter found. Use default: 0.35."
        << std::endl;
  } else {
    std::cout << this->hint_
              << ": Velocity threshold for dynamic classification is set to: "
              << this->dynaVelThresh_ << std::endl;
  }

  // voting threshold for dynamic classification
  if (not this->nh_.getParam(this->ns_ + "/dynamic_voting_threshold",
                             this->dynaVoteThresh_)) {
    this->dynaVoteThresh_ = 0.8;
    std::cout
        << this->hint_
        << ": No dynamic velocity threshold parameter found. Use default: 0.8."
        << std::endl;
  } else {
    std::cout << this->hint_
              << ": Voting threshold for dynamic classification is set to: "
              << this->dynaVoteThresh_ << std::endl;
  }

  // frames to force dynamic
  if (not this->nh_.getParam(this->ns_ + "/frames_force_dynamic",
                             this->forceDynaFrames_)) {
    this->forceDynaFrames_ = 20;
    std::cout << this->hint_
              << ": No range of searching dynamic obstacles in box history "
                 "found. Use default: 20."
              << std::endl;
  } else {
    std::cout
        << this->hint_
        << ": Range of searching dynamic obstacles in box history is set to: "
        << this->forceDynaFrames_ << std::endl;
  }

  if (not this->nh_.getParam(this->ns_ + "/frames_force_dynamic_check_range",
                             this->forceDynaCheckRange_)) {
    this->forceDynaCheckRange_ = 30;
    std::cout << this->hint_
              << ": No threshold for forcing dynamic obstacles found. Use "
                 "default: 30."
              << std::endl;
  } else {
    std::cout << this->hint_
              << ": Threshold for forcing dynamic obstacles is set to: "
              << this->forceDynaCheckRange_ << std::endl;
  }

  // dynamic consistency check
  if (not this->nh_.getParam(this->ns_ + "/dynamic_consistency_threshold",
                             this->dynamicConsistThresh_)) {
    this->dynamicConsistThresh_ = 3;
    std::cout
        << this->hint_
        << ": No threshold for dynamic-consistency check found. Use default: 3."
        << std::endl;
  } else {
    std::cout << this->hint_
              << ": Threshold for dynamic consistency check is set to: "
              << this->dynamicConsistThresh_ << std::endl;
  }

  if (this->histSize_ < this->forceDynaCheckRange_ + 1) {
    ROS_ERROR("history length is too short to perform force-dynamic");
  }

  //-----------------------------------------尺寸约束参数--------------------------------------------------------------
  // max object size
  std::vector<double> maxObjectSizeTemp;
  if (not this->nh_.getParam(this->ns_ + "/max_object_size",
                             maxObjectSizeTemp)) {
    this->maxObjectSize_ = Eigen::Vector3d(2.0, 2.0, 2.0);
    std::cout << this->hint_
              << ": No max object size threshold parameter found. Use default: "
                 "[2.0, 2.0, 2.0]."
              << endl;
  } else {
    this->maxObjectSize_(0) = maxObjectSizeTemp[0];
    this->maxObjectSize_(1) = maxObjectSizeTemp[1];
    this->maxObjectSize_(2) = maxObjectSizeTemp[2];
    std::cout << this->hint_ << ": Max object size threshold is set to: [";
    for (size_t i = 0; i < maxObjectSizeTemp.size(); ++i) {
      std::cout << maxObjectSizeTemp[i];
      if (i != maxObjectSizeTemp.size() - 1) {
        std::cout << ", ";
      }
    }
    std::cout << "]." << std::endl;
  }

  //-----------------------------------------物体分类参数--------------------------------------------------------------
  // 人的分类阈值
  std::vector<double> classifyHumanThresh;
  if (not this->nh_.getParam(this->ns_ + "/classify_human_threshold",
                             classifyHumanThresh)) {
    this->classifyHumanZWidthRatio_ = 2.0;
    this->classifyHumanCentroidZRatio_ = 0.5;
    ROS_WARN_STREAM(
        this->hint_
        << " No classify_human_threshold param. Use default: [2.0, 0.5]");
  } else {
    this->classifyHumanZWidthRatio_ = classifyHumanThresh[0];
    this->classifyHumanCentroidZRatio_ = classifyHumanThresh[1];
    ROS_INFO_STREAM(this->hint_ << " classify_human_threshold: ["
                                << this->classifyHumanZWidthRatio_ << ", "
                                << this->classifyHumanCentroidZRatio_ << "]");
  }

  // 车的分类阈值
  std::vector<double> classifyVehicleThresh;
  if (not this->nh_.getParam(this->ns_ + "/classify_vehicle_threshold",
                             classifyVehicleThresh)) {
    this->classifyVehicleXYWidthRatio_ = 1.5;
    this->classifyVehicleCentroidZRatio_ = 0.8;
    ROS_WARN_STREAM(
        this->hint_
        << " No classify_vehicle_threshold param. Use default: [1.5, 0.8]");
  } else {
    this->classifyVehicleXYWidthRatio_ = classifyVehicleThresh[0];
    this->classifyVehicleCentroidZRatio_ = classifyVehicleThresh[1];
    ROS_INFO_STREAM(this->hint_ << " classify_vehicle_threshold: ["
                                << this->classifyVehicleXYWidthRatio_ << ", "
                                << this->classifyVehicleCentroidZRatio_ << "]");
  }

  // 无人机的分类阈值
  std::vector<double> classifyUAVThresh;
  if (not this->nh_.getParam(this->ns_ + "/classify_uav_threshold",
                             classifyUAVThresh)) {
    this->classifyUAVMaxSize_ = 0.6;
    this->classifyUAVCentroidZRatio_ = 1.2;
    ROS_WARN_STREAM(
        this->hint_
        << " No classify_uav_threshold param. Use default: [0.6, 1.2]");
  } else {
    this->classifyUAVMaxSize_ = classifyUAVThresh[0];
    this->classifyUAVCentroidZRatio_ = classifyUAVThresh[1];
    ROS_INFO_STREAM(this->hint_ << " classify_uav_threshold: ["
                                << this->classifyUAVMaxSize_ << ", "
                                << this->classifyUAVCentroidZRatio_ << "]");
    ROS_INFO_STREAM(this->hint_ << " classify_uav_threshold: ["
                                << this->classifyUAVMaxSize_ << ", "
                                << this->classifyUAVCentroidZRatio_ << "]");
  }

  // 分类与模型切换参数
  if (not this->nh_.getParam(this->ns_ + "/classification_start_frame",
                             this->classificationStartFrame_)) {
    this->classificationStartFrame_ = 10;
    ROS_WARN_STREAM(this->hint_
                    << " No classification_start_frame param. Use default: 10");
  }
  if (not this->nh_.getParam(this->ns_ + "/classify_human_pca_ratio",
                             this->classifyHumanPcaRatio_)) {
    this->classifyHumanPcaRatio_ = 1.5;
    ROS_WARN_STREAM(this->hint_
                    << " No classify_human_pca_ratio param. Use default: 1.5");
  }
  if (not this->nh_.getParam(this->ns_ + "/classify_vehicle_pca_ratio",
                             this->classifyVehiclePcaRatio_)) {
    this->classifyVehiclePcaRatio_ = 1.5;
    ROS_WARN_STREAM(this->hint_ << " No classify_vehicle_pca_ratio param. Use "
                                   "default: 1.5");
  }

  // -----------------------------------------卡尔曼滤波器参数--------------------------------------------------------------
  // CA Model (Human)
  if (not this->nh_.getParam(this->ns_ +
                                 "/kalman_filter/ca_model/human/jerk_sigma",
                             this->kfParams_.ca_human.jerk_sigma)) {
    this->kfParams_.ca_human.jerk_sigma = 1.0;
  }
  if (not this->nh_.getParam(this->ns_ +
                                 "/kalman_filter/ca_model/human/init_cov",
                             this->kfParams_.ca_human.init_cov)) {
    this->kfParams_.ca_human.init_cov = {0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 10.0};
  }
  if (not this->nh_.getParam(this->ns_ +
                                 "/kalman_filter/ca_model/human/meas_noise",
                             this->kfParams_.ca_human.meas_noise)) {
    this->kfParams_.ca_human.meas_noise = {0.1, 0.1, 0.1};
  }
  if (not this->nh_.getParam(
          this->ns_ + "/kalman_filter/ca_model/human/z_process_noise",
          this->kfParams_.ca_human.z_process_noise)) {
    this->kfParams_.ca_human.z_process_noise = 0.01;
  }

  // CA Model (UAV)
  if (not this->nh_.getParam(this->ns_ +
                                 "/kalman_filter/ca_model/uav/jerk_sigma",
                             this->kfParams_.ca_uav.jerk_sigma)) {
    this->kfParams_.ca_uav.jerk_sigma = 1.0;
  }
  if (not this->nh_.getParam(this->ns_ + "/kalman_filter/ca_model/uav/init_cov",
                             this->kfParams_.ca_uav.init_cov)) {
    this->kfParams_.ca_uav.init_cov = {0.1, 0.1,  0.1,  1.0, 1.0,
                                       1.0, 10.0, 10.0, 10.0};
  }
  if (not this->nh_.getParam(this->ns_ +
                                 "/kalman_filter/ca_model/uav/meas_noise",
                             this->kfParams_.ca_uav.meas_noise)) {
    this->kfParams_.ca_uav.meas_noise = {0.1, 0.1, 0.1};
  }

  // CV Model
  if (not this->nh_.getParam(this->ns_ + "/kalman_filter/cv_model/acc_sigma",
                             this->kfParams_.cv.acc_sigma)) {
    this->kfParams_.cv.acc_sigma = 3.0;
  }
  if (not this->nh_.getParam(this->ns_ + "/kalman_filter/cv_model/init_cov",
                             this->kfParams_.cv.init_cov)) {
    this->kfParams_.cv.init_cov = {0.5, 0.5, 0.5, 2.0, 2.0, 2.0};
  }
  if (not this->nh_.getParam(this->ns_ + "/kalman_filter/cv_model/meas_noise",
                             this->kfParams_.cv.meas_noise)) {
    this->kfParams_.cv.meas_noise = {0.3, 0.3, 0.3};
  }

  // CTRA Model
  if (not this->nh_.getParam(this->ns_ + "/kalman_filter/ctra_model/init_cov",
                             this->kfParams_.ctra.init_cov)) {
    this->kfParams_.ctra.init_cov = {0.1, 0.1, 0.1, 1.0, 10.0, 0.5, 1.0};
  }
  if (not this->nh_.getParam(this->ns_ +
                                 "/kalman_filter/ctra_model/process_noise",
                             this->kfParams_.ctra.process_noise)) {
    this->kfParams_.ctra.process_noise = {0.1, 0.1, 0.01, 1.0, 10.0, 0.1, 1.0};
  }
  if (not this->nh_.getParam(this->ns_ + "/kalman_filter/ctra_model/meas_noise",
                             this->kfParams_.ctra.meas_noise)) {
    this->kfParams_.ctra.meas_noise = {0.1, 0.1, 0.1};
  }

  // -----------------------------------------自适应测量噪声R参数--------------------------------------------------------------
  if (not this->nh_.getParam(
          this->ns_ + "/kalman_filter/adaptive_measurement_noise/enable",
          this->adaptiveREnabled_)) {
    this->adaptiveREnabled_ = false;
    ROS_WARN_STREAM(this->hint_
                    << " No adaptive_measurement_noise/enable param. Use "
                       "default: false");
  } else {
    ROS_INFO_STREAM(this->hint_
                    << " Adaptive measurement noise R enabled: "
                    << (this->adaptiveREnabled_ ? "true" : "false"));
  }

  if (not this->nh_.getParam(
          this->ns_ + "/kalman_filter/adaptive_measurement_noise/window_size",
          this->adaptiveRWindowSize_)) {
    this->adaptiveRWindowSize_ = 10;
    ROS_WARN_STREAM(this->hint_ << " No adaptive_measurement_noise/window_size "
                                   "param. Use default: 10");
  } else {
    ROS_INFO_STREAM(this->hint_ << " Adaptive R window size: "
                                << this->adaptiveRWindowSize_);
  }

  if (not this->nh_.getParam(
          this->ns_ + "/kalman_filter/adaptive_measurement_noise/min_ratio",
          this->adaptiveRMinRatio_)) {
    this->adaptiveRMinRatio_ = 0.3;
    ROS_WARN_STREAM(this->hint_ << " No adaptive_measurement_noise/min_ratio "
                                   "param. Use default: 0.3");
  } else {
    ROS_INFO_STREAM(this->hint_ << " Adaptive R min ratio: "
                                << this->adaptiveRMinRatio_);
  }

  if (not this->nh_.getParam(
          this->ns_ +
              "/kalman_filter/adaptive_measurement_noise/adaptive_weight",
          this->adaptiveRWeight_)) {
    this->adaptiveRWeight_ = 0.7;
    ROS_WARN_STREAM(this->hint_
                    << " No adaptive_measurement_noise/adaptive_weight param. "
                       "Use default: 0.7");
  } else {
    ROS_INFO_STREAM(this->hint_ << " Adaptive R weight: "
                                << this->adaptiveRWeight_);
  }

  if (not this->nh_.getParam(
          this->ns_ + "/kalman_filter/adaptive_measurement_noise/human_z_scale",
          this->adaptiveRHumanZScale_)) {
    this->adaptiveRHumanZScale_ = 0.5;
    ROS_WARN_STREAM(this->hint_
                    << " No adaptive_measurement_noise/human_z_scale param. "
                       "Use default: 0.5");
  } else {
    ROS_INFO_STREAM(this->hint_ << " Adaptive R human z scale: "
                                << this->adaptiveRHumanZScale_);
  }

  if (not this->nh_.getParam(
          this->ns_ + "/kalman_filter/adaptive_measurement_noise/uav_z_scale",
          this->adaptiveRUavZScale_)) {
    this->adaptiveRUavZScale_ = 1.5;
    ROS_WARN_STREAM(this->hint_ << " No adaptive_measurement_noise/uav_z_scale "
                                   "param. Use default: 1.5");
  } else {
    ROS_INFO_STREAM(this->hint_ << " Adaptive R UAV z scale: "
                                << this->adaptiveRUavZScale_);
  }

  if (not this->nh_.getParam(
          this->ns_ +
              "/kalman_filter/adaptive_measurement_noise/ctra_turn_scale",
          this->adaptiveRCtraTurnScale_)) {
    this->adaptiveRCtraTurnScale_ = 0.5;
    ROS_WARN_STREAM(this->hint_
                    << " No adaptive_measurement_noise/ctra_turn_scale param. "
                       "Use default: 0.5");
  } else {
    ROS_INFO_STREAM(this->hint_ << " Adaptive R CTRA turn scale: "
                                << this->adaptiveRCtraTurnScale_);
  }

  if (not this->nh_.getParam(
          this->ns_ +
              "/kalman_filter/adaptive_measurement_noise/ctra_turn_threshold",
          this->adaptiveRCtraTurnThreshold_)) {
    this->adaptiveRCtraTurnThreshold_ = 0.1;
    ROS_WARN_STREAM(
        this->hint_
        << " No adaptive_measurement_noise/ctra_turn_threshold param. Use "
           "default: 0.1");
  } else {
    ROS_INFO_STREAM(this->hint_ << " Adaptive R CTRA turn threshold: "
                                << this->adaptiveRCtraTurnThreshold_);
  }

  // 初始化激光雷达检测器（避免每次回调时重复初始化）
  this->lidarDetector_.reset(new lidarDetector());
  this->lidarDetector_->setParams(
      this->lidarDBEpsilon_, this->lidarDBMinPoints_, this->lidarDBUseAdaptive_,
      this->lidarDBDistanceScale_);
  ROS_INFO_STREAM(this->hint_ << " Lidar detector initialized");
}

void dynamicDetector::registerPub() {
  //===========================原始点云过滤与检测器可视化========================================
  // 原始激光雷达点可视化发布
  this->rawLidarPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(
      this->ns_ + "/raw_lidar_point_cloud", 10);

  // 过滤后点云可视化发布
  this->downSamplePointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(
      this->ns_ + "/downsampled_point_cloud", 10);

  // 聚类检测后的点云发布
  this->filteredPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(
      this->ns_ + "/filtered_point_cloud", 10);

  // 聚类检测后的边界框发布
  this->filteredBBoxesPub_ =
      this->nh_.advertise<visualization_msgs::MarkerArray>(
          this->ns_ + "/filtered_bboxes", 10);

  //============================数据跟踪可视化===============================================
  // 跟踪的边界框发布
  this->trackedBBoxesPub_ =
      this->nh_.advertise<visualization_msgs::MarkerArray>(
          this->ns_ + "/tracked_bboxes", 10);

  // 历史轨迹发布
  this->historyTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(
      this->ns_ + "/history_trajectories", 10);

  // 速度可视化发布
  this->velVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(
      this->ns_ + "/velocity_visualizaton", 10);

  //===========================动态检测可视化===============================================
  // 动态点云发布
  this->dynamicPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(
      this->ns_ + "/dynamic_point_cloud", 10);

  // 动态边界框发布
  this->dynamicBBoxesPub_ =
      this->nh_.advertise<visualization_msgs::MarkerArray>(
          this->ns_ + "/dynamic_bboxes", 10);

  // 原始动态点云发布
  this->rawDynamicPointsPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(
      this->ns_ + "/raw_dynamic_point_cloud", 10);
}

void dynamicDetector::registerCallback() {
  // message_filters和正常的ros订阅区别是，message_filters不会直接调用回调函数，而是满足过滤器的条件才调用
  //  深度图像和位姿回调。reset表示释放旧的对象，处理当前的新对象。new动态内存分配，如果分配的是对象，new会调用该对象的构造函数来初始化它，也分配内存

  if (this->useLivoxCustomMsg_) {
    // 使用Livox CustomMsg格式
    this->lidarCustomMsgSub_.reset(
        new message_filters::Subscriber<livox_ros_driver2::CustomMsg>(
            this->nh_, this->lidarTopicName_, 50));

    if (this->localizationMode_ == 0) {
      this->poseSub_.reset(
          new message_filters::Subscriber<geometry_msgs::PoseStamped>(
              this->nh_, this->poseTopicName_, 25));
      this->lidarCustomPoseSync_.reset(
          new message_filters::Synchronizer<lidarCustomPoseSync>(
              lidarCustomPoseSync(100), *this->lidarCustomMsgSub_,
              *this->poseSub_));
      this->lidarCustomPoseSync_->registerCallback(
          boost::bind(&dynamicDetector::lidarCustomPoseCB, this, _1, _2));
    } else if (this->localizationMode_ == 1) {
      this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
          this->nh_, this->odomTopicName_, 25));
      this->lidarCustomOdomSync_.reset(
          new message_filters::Synchronizer<lidarCustomOdomSync>(
              lidarCustomOdomSync(100), *this->lidarCustomMsgSub_,
              *this->odomSub_));
      this->lidarCustomOdomSync_->registerCallback(
          boost::bind(&dynamicDetector::lidarCustomOdomCB, this, _1, _2));
    } else {
      ROS_ERROR("[dynamicDetector]: Invalid localization mode!");
      exit(0);
    }
  } else {
    // 使用标准PointCloud2格式
    this->lidarCloudSub_.reset(
        new message_filters::Subscriber<sensor_msgs::PointCloud2>(
            this->nh_, this->lidarTopicName_, 50));

    if (this->localizationMode_ == 0) {
      this->poseSub_.reset(
          new message_filters::Subscriber<geometry_msgs::PoseStamped>(
              this->nh_, this->poseTopicName_, 25));
      this->lidarPoseSync_.reset(
          new message_filters::Synchronizer<lidarPoseSync>(
              lidarPoseSync(100), *this->lidarCloudSub_, *this->poseSub_));
      this->lidarPoseSync_->registerCallback(
          boost::bind(&dynamicDetector::lidarPoseCB, this, _1, _2));
    } else if (this->localizationMode_ == 1) {
      this->odomSub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
          this->nh_, this->odomTopicName_, 25));
      this->lidarOdomSync_.reset(
          new message_filters::Synchronizer<lidarOdomSync>(
              lidarOdomSync(100), *this->lidarCloudSub_, *this->odomSub_));
      this->lidarOdomSync_->registerCallback(
          boost::bind(&dynamicDetector::lidarOdomCB, this, _1, _2));
    } else {
      ROS_ERROR("[dynamicDetector]: Invalid localization mode!");
      exit(0);
    }
  }

  // 激光雷达检测定时器
  this->lidarDetectionTimer_ = this->nh_.createTimer(
      ros::Duration(this->dt_), &dynamicDetector::lidarDetectionCB, this);

  // 跟踪定时器
  this->trackingTimer_ = this->nh_.createTimer(
      ros::Duration(this->dt_), &dynamicDetector::trackingCB, this);

  // 分类定时器
  this->classificationTimer_ = this->nh_.createTimer(
      ros::Duration(this->dt_), &dynamicDetector::classificationCB, this);

  // 可视化定时器
  this->visTimer_ = this->nh_.createTimer(ros::Duration(this->dt_),
                                          &dynamicDetector::visCB, this);

  // 获取动态障碍物服务
  this->getDynamicObstacleServer_ =
      this->nh_.advertiseService("onboard_detector/get_dynamic_obstacles",
                                 &dynamicDetector::getDynamicObstacles, this);
}

// 获取动态障碍物的服务回调函数。对获取的障碍物按与机器人的距离从小到大排序
bool dynamicDetector::getDynamicObstacles(
    onboard_detector::GetDynamicObstacles::Request &req,
    onboard_detector::GetDynamicObstacles::Response &res) {
  std::lock_guard<std::mutex> lock(bboxMutex_); // 加锁保护动态边界框数据

  // 从服务请求中获取机器人当前的位置
  Eigen::Vector3d currPos = Eigen::Vector3d(
      req.current_position.x, req.current_position.y, req.current_position.z);

  // 创建一个向量，用于存储障碍物id及与机器人距离的键值对，方便后续排序
  std::vector<std::pair<double, onboardDetector::box3D>> obstaclesWithDistances;

  // 遍历当前所有已检测到的动态障碍物
  for (const onboardDetector::box3D &bbox : this->dynamicBBoxes_) {
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
            [](const std::pair<double, onboardDetector::box3D> &a,
               const std::pair<double, onboardDetector::box3D> &b) {
              return a.first < b.first;
            });

  // 将排序后的障碍物信息填充到服务响应中
  for (const auto &item : obstaclesWithDistances) {
    const onboardDetector::box3D &bbox = item.second;

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

// 转换点云格式，滤波一定范围内的点
void dynamicDetector::lidarPoseCB(
    const sensor_msgs::PointCloud2ConstPtr &cloudMsg,
    const geometry_msgs::PoseStampedConstPtr &pose) {
  // [Performance Timing] 测量回调函数耗时
  // auto start_time = std::chrono::high_resolution_clock::now();

  std::lock_guard<std::mutex> lock(cloudMutex_); // 加锁保护共享数据

  // 仅用于可视化，存储最新的原始点云消息
  this->hasSensorPose_ = true;
  this->latestCloud_ = cloudMsg;
  this->lastCloudTime_ = cloudMsg->header.stamp; // 记录时间戳

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
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloudMsg, *tempCloud);

  // --- 优化：一次性滤波（X、Y范围）并进行均匀密度降采样 ---
  pcl::PointCloud<pcl::PointXYZ>::Ptr preTransformCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  preTransformCloud->reserve(tempCloud->size()); // 预分配内存，估计保留1/3的点

  double x_max = this->localLidarRange_.x();
  double y_max = this->localLidarRange_.y();

  for (const pcl::PointXYZ &pt : tempCloud->points) {
    // 先做范围检查（最快的操作）
    if (std::abs(pt.x) > x_max || std::abs(pt.y) > y_max) {
      continue;
    }
    preTransformCloud->push_back(pt);
  }

  // --- 坐标变换 ---
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.linear() = this->orientationLidar_;
  transform.translation() = this->positionLidar_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*preTransformCloud, *transformedCloud, transform);

  // --- 过滤地面和天花板（在变换后进行，避免重复操作） ---
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundRoofFilterCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  groundRoofFilterCloud->reserve(transformedCloud->size());

  for (const pcl::PointXYZ &pt : transformedCloud->points) {
    if (pt.z >= this->groundHeight_ && pt.z <= this->roofHeight_) {
      groundRoofFilterCloud->push_back(pt);
    }
  }

  // 存储处理后的激光雷达点云
  this->lidarCloud_ = groundRoofFilterCloud;
  hasNewCloud_ = true; // 标记有新数据可用

  // 将处理后的点云发布出去，用于可视化
  sensor_msgs::PointCloud2 outputCloud;
  pcl::toROSMsg(*this->lidarCloud_, outputCloud);
  outputCloud.header.frame_id = "map";
  outputCloud.header.stamp = cloudMsg->header.stamp;
  this->downSamplePointsPub_.publish(outputCloud);

  // [Performance Timing] 输出耗时
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end_time -
  // start_time); ROS_INFO_THROTTLE(1.0, "%s: lidarPoseCB took %.3f ms, points:
  // %lu -> %lu",
  //                  this->hint_.c_str(), duration.count() / 1000.0,
  //                  tempCloud->size(), this->lidarCloud_->size());
  // ROS_INFO_THROTTLE(1.0, "new pointCloud%s", this->hasNewCloud_);
}

// 里程计回调函数，处理点云和里程计数据
void dynamicDetector::lidarOdomCB(
    const sensor_msgs::PointCloud2ConstPtr &cloudMsg,
    const nav_msgs::OdometryConstPtr &odom) {
  // [Performance Timing] 测量回调函数耗时
  // auto start_time = std::chrono::high_resolution_clock::now();

  std::lock_guard<std::mutex> lock(cloudMutex_); // 加锁保护共享数据

  // 用于可视化
  this->hasSensorPose_ = true;
  this->latestCloud_ = cloudMsg;
  this->lastCloudTime_ = cloudMsg->header.stamp; // 记录时间戳

  // --- 提前更新位姿信息 ---
  Eigen::Matrix4d lidarPoseMatrix;
  this->getLidarPose(odom, lidarPoseMatrix);

  this->position_(0) = odom->pose.pose.position.x;
  this->position_(1) = odom->pose.pose.position.y;
  this->position_(2) = odom->pose.pose.position.z;
  Eigen::Quaterniond quat(
      odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  this->orientation_ = quat.toRotationMatrix();

  this->positionLidar_(0) = lidarPoseMatrix(0, 3);
  this->positionLidar_(1) = lidarPoseMatrix(1, 3);
  this->positionLidar_(2) = lidarPoseMatrix(2, 3);
  this->orientationLidar_ = lidarPoseMatrix.block<3, 3>(0, 0);

  // 局部点云转换
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloudMsg, *tempCloud);

  // --- 优化：一次性滤波（X、Y范围）并进行均匀密度降采样 ---
  pcl::PointCloud<pcl::PointXYZ>::Ptr preTransformCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
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
  }

  // --- 坐标变换 ---
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.linear() = this->orientationLidar_;
  transform.translation() = this->positionLidar_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*preTransformCloud, *transformedCloud, transform);

  // --- 过滤地面和天花板（在变换后进行，避免重复操作） ---
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundRoofFilterCloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  groundRoofFilterCloud->reserve(transformedCloud->size());

  for (const pcl::PointXYZ &pt : transformedCloud->points) {
    if (pt.z >= this->groundHeight_ && pt.z <= this->roofHeight_) {
      groundRoofFilterCloud->push_back(pt);
    }
  }

  // 存储处理后的点云
  this->lidarCloud_ = groundRoofFilterCloud;
  hasNewCloud_ = true; // 标记有新数据可用

  // 发布降采样后的点云
  sensor_msgs::PointCloud2 outputCloud;
  pcl::toROSMsg(*this->lidarCloud_, outputCloud);
  outputCloud.header.frame_id = "map";
  outputCloud.header.stamp = cloudMsg->header.stamp;
  this->downSamplePointsPub_.publish(outputCloud);

  // [Performance Timing] 输出耗时
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end_time -
  // start_time); ROS_INFO_THROTTLE(1.0, "%s: lidarOdomCB took %.3f ms, points:
  // %lu -> %lu",
  //                  this->hint_.c_str(), duration.count() / 1000.0,
  //                  tempCloud->size(), this->lidarCloud_->size());
  // ROS_INFO_THROTTLE(1.0, "new pointCloud%s", this->hasNewCloud_);
}

// 激光雷达检测定时器回调函数
void dynamicDetector::lidarDetectionCB(const ros::TimerEvent &event) {
  // [Performance Timing] 测量回调函数耗时
  // auto start_time = std::chrono::high_resolution_clock::now();

  // 检查是否有新点云数据
  if (!hasNewCloud_) {
    ROS_WARN_THROTTLE(5.0, "%s: No new cloud data available for detection",
                      this->hint_.c_str());
    return;
  }

  // 检查数据时效性（避免处理过时数据）
  if ((event.current_real - lastCloudTime_).toSec() > 0.5) {
    ROS_WARN_THROTTLE(
        5.0, "%s: Cloud data too old (%.3f s), skipping detection",
        this->hint_.c_str(), (event.current_real - lastCloudTime_).toSec());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(cloudMutex_);
    this->lidarDetect();
    hasNewCloud_ = false; // 标记数据已处理
  }

  this->newDetectFlag_ = true; // get a new detection
  hasNewDetection_ = true;     // 标记有新检测结果

  // [Performance Timing] 输出耗时
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end_time -
  // start_time); ROS_INFO_THROTTLE(1.0, "%s: lidarDetectionCB took %.3f ms",
  // this->hint_.c_str(), duration.count() / 1000.0);
}

// 跟踪定时器回调函数,有个问题，匹配时，多出的轨迹直接丢掉
void dynamicDetector::trackingCB(const ros::TimerEvent &) {
  // [Performance Timing] 测量回调函数耗时
  // auto start_time = std::chrono::high_resolution_clock::now();

  // 检查是否有新检测结果
  if (!hasNewDetection_) {
    ROS_WARN_THROTTLE(5.0, "%s: No new detection available for tracking",
                      this->hint_.c_str());
    return; // 跳过，避免重复处理相同数据
  }

  std::lock_guard<std::mutex> lock(bboxMutex_); // 加锁保护边界框数据

  // 数据关联线程（预测步骤在 boxAssociation 内部执行）
  std::vector<int> bestMatch;      // 存储当前检测与历史障碍物的匹配索引。
  this->boxAssociation(bestMatch); // 执行边界框关联。

  // --- 1. 先进行物体分类(针对匹配成功的旧轨迹) ---
  if (bestMatch.size()) {
    for (int i = 0; i < int(bestMatch.size()); ++i) {
      if (bestMatch[i] >= 0) { // 匹配成功的旧轨迹
        int histIndex = bestMatch[i];

        // 1.1 更新历史最大尺寸
        double curr_x = this->filteredBBoxes_[i].x_width;
        double curr_y = this->filteredBBoxes_[i].y_width;
        double curr_z = this->filteredBBoxes_[i].z_width;
        if (curr_x > this->maxHistorySizes_[histIndex].x())
          this->maxHistorySizes_[histIndex].x() = curr_x;
        if (curr_y > this->maxHistorySizes_[histIndex].y())
          this->maxHistorySizes_[histIndex].y() = curr_y;
        if (curr_z > this->maxHistorySizes_[histIndex].z())
          this->maxHistorySizes_[histIndex].z() = curr_z;

        // 1.2 检查是否需要进行分类
        // 首次分类：达到 classificationStartFrame_ 且从未分类过 (is_else 为
        // true) 后续分类：使用 ROS 时间间隔 classificationIntervalSec_ 判断
        bool needClassify = false;

        // 确保 lastClassifyTime_ 与历史大小匹配
        if (lastClassifyTime_.size() < this->boxHist_.size()) {
          lastClassifyTime_.resize(this->boxHist_.size(), ros::Time(0));
        }

        if (int(this->boxHist_[histIndex].size()) ==
            this->classificationStartFrame_) {
          // 首次达到分类阈值，进行分类
          needClassify = true;
          lastClassifyTime_[histIndex] = ros::Time::now();
        } else if (int(this->boxHist_[histIndex].size()) >
                   this->classificationStartFrame_) {
          // 已经分类过，检查时间间隔
          ros::Duration timeSince =
              ros::Time::now() - lastClassifyTime_[histIndex];
          if (timeSince.toSec() >= this->classificationIntervalSec_) {
            needClassify = true;
            lastClassifyTime_[histIndex] = ros::Time::now();
          }
        }

        if (needClassify) {
          Eigen::Vector4f centroid;
          centroid << this->filteredPcClusterCenters_[i](0),
              this->filteredPcClusterCenters_[i](1),
              this->filteredPcClusterCenters_[i](2), 1.0;

          // 对当前检测框进行分类,结果写入filteredBBoxes_[i]
          this->classifyBox(this->filteredBBoxes_[i], centroid,
                            this->filteredPcClusterStds_[i],
                            this->maxHistorySizes_[histIndex]);

          // 立即切换卡尔曼滤波模型(在更新之前)
          this->switchKalmanModel(histIndex, this->filteredBBoxes_[i]);
        } else {
          // 未达到分类或重新分类条件,继承历史分类结果
          this->filteredBBoxes_[i].is_human =
              this->boxHist_[histIndex][0].is_human;
          this->filteredBBoxes_[i].is_che = this->boxHist_[histIndex][0].is_che;
          this->filteredBBoxes_[i].is_uav = this->boxHist_[histIndex][0].is_uav;
          this->filteredBBoxes_[i].is_else =
              this->boxHist_[histIndex][0].is_else;
        }
      }
    }
  }

  // --- 2. 卡尔曼滤波跟踪(此时filteredBBoxes_已包含最新的分类信息) ---
  if (bestMatch.size()) {
    this->kalmanFilterAndUpdateHist(bestMatch); // 更新卡尔曼滤波器和历史记录

    // --- 3. 移除重复轨迹（解决幽灵轨迹问题）---
    this->removeDuplicateTracks();
  } else { // 如果当前帧没有任何检测结果
    // 清空历史记录。
    this->boxHist_.clear();
    this->pcHist_.clear();
    this->pcCenterHist_.clear();
    this->pcStdHist_.clear();
    this->maxHistorySizes_.clear();
    // this->filters_.clear(); // 同时清空滤波器
  }

  hasNewDetection_ = false; // 标记检测结果已处理
  hasNewTracking_ = true;   // 标记有新跟踪结果

  // [Performance Timing] 输出耗时
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end_time -
  // start_time); ROS_INFO_THROTTLE(1.0, "%s: trackingCB took %.3f ms",
  // this->hint_.c_str(), duration.count() / 1000.0);
}

// 动静态分类定时器回调函数
void dynamicDetector::classificationCB(const ros::TimerEvent &) {
  // // [Performance Timing] 测量回调函数耗时
  // auto start_time = std::chrono::high_resolution_clock::now();

  // 检查是否有新跟踪结果
  if (!hasNewTracking_) {
    return; // 跳过，避免重复处理相同数据
  }

  std::lock_guard<std::mutex> lock(bboxMutex_); // 加锁保护边界框数据

  // 创建一个临时向量来存储当前帧检测到的动态边界框
  std::vector<onboardDetector::box3D> dynamicBBoxesTemp;

  // 遍历所有被跟踪目标的点云/边界框历史，只判断xy平面的动态性
  // 注意：在某些情况下，我们不需要执行动态障碍物识别
  for (size_t i = 0; i < this->pcHist_.size(); ++i) {
    // ===================================================================================
    // 情况一：历史记录长度不足以进行分类
    // 确定用于比较的当前帧与历史帧之间的时间间隔（帧数）
    int curFrameGap;
    if (int(this->pcHist_[i].size()) < this->skipFrame_ + 1) {
      // 如果历史记录不够长，就用现有的最远一帧进行比较
      curFrameGap = this->pcHist_[i].size() - 1;
    } else {
      // 否则，使用参数设定的帧间隔
      curFrameGap = this->skipFrame_;
    }
    // ===================================================================================

    // ==================================================================================
    // 情况二：强制动态（如果一个障碍物在过去一段时间内被频繁分类为动态，则强制认定其为动态）
    int dynaFrames = 0;
    if (int(this->boxHist_[i].size()) > this->forceDynaCheckRange_) {
      for (int j = 1; j < this->forceDynaCheckRange_ + 1; ++j) {
        if (this->boxHist_[i][j].is_dynamic) {
          ++dynaFrames;
        }
      }
    }

    if (dynaFrames >= this->forceDynaFrames_) {
      this->boxHist_[i][0].is_dynamic = true;
      dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
      continue;
    }
    // ===================================================================================

    // 获取当前帧和历史计算帧的点云
    std::vector<Eigen::Vector3d> currPc = this->pcHist_[i][0];
    std::vector<Eigen::Vector3d> prevPc = this->pcHist_[i][curFrameGap];

    // 初始化速度向量
    Eigen::Vector3d Vcur(0., 0., 0.); // 单个点的速度
    Eigen::Vector3d Vbox(0., 0., 0.); // 整个边界框的平均速度
    Eigen::Vector3d Vkf(0., 0., 0.);  // 卡尔曼滤波器估计的速度

    int numPoints = currPc.size(); // 点云中的总点数，用于计算投票率
    int votes = 0;                 // “动态”票数

    // 计算边界框中心点的速度
    Vbox(0) = (this->boxHist_[i][0].x - this->boxHist_[i][curFrameGap].x) /
              (this->dt_ * curFrameGap);
    Vbox(1) = (this->boxHist_[i][0].y - this->boxHist_[i][curFrameGap].y) /
              (this->dt_ * curFrameGap);
    Vbox(2) = (this->boxHist_[i][0].z - this->boxHist_[i][curFrameGap].z) /
              (this->dt_ * curFrameGap);

    // 获取卡尔曼滤波器估计的速度，根据不同模型维度进行计算
    Eigen::VectorXd state = this->filters_[i]->getState();
    int dim = state.size();
    if (dim == 6) {
      // 3D CV: [x, y, z, vx, vy, vz]
      Vkf(0) = state(3);
      Vkf(1) = state(4);
    } else if (dim == 7) {
      // 7维可能是 Human CA 或 Vehicle CTRA，依据历史分类决定
      bool isVehicle = this->boxHist_[i][0].is_che; // Vehicle CTRA
      if (isVehicle) {
        // CTRA: [x, y, z, v, a, yaw, yaw_rate]
        double v = state(3);
        double yaw = state(5);
        Vkf(0) = v * cos(yaw);
        Vkf(1) = v * sin(yaw);
      } else {
        // Human CA: [x, y, z, vx, vy, ax, ay]
        Vkf(0) = state(3);
        Vkf(1) = state(4);
      }
    } else if (dim == 9) {
      // 3D CA: [x, y, z, vx, vy, vz, ax, ay, az]
      Vkf(0) = state(3);
      Vkf(1) = state(4);
    } else {
      // fallback to historical speed
      Vkf(0) = this->boxHist_[i][0].Vx;
      Vkf(1) = this->boxHist_[i][0].Vy;
    }

    // 遍历当前点云中的每一个点，通过与历史点云比较来“投票”
    for (size_t j = 0; j < currPc.size(); ++j) {
      double minDist = 2; // 初始化一个较大的最小距离
      Eigen::Vector3d nearestVect;
      // 在历史点云中为当前点寻找最近邻点
      for (size_t k = 0; k < prevPc.size(); k++) {
        double dist = (currPc[j] - prevPc[k]).norm();
        if (abs(dist) < minDist) {
          minDist = dist;
          nearestVect = currPc[j] - prevPc[k]; // 记录位移向量
        }
      }
      // 计算该点的速度，并忽略Z轴
      Vcur = nearestVect / (this->dt_ * curFrameGap);
      Vcur(2) = 0;
      // 计算点的速度向量与边界框整体速度向量的余弦相似度
      double velSim = Vcur.dot(Vbox) / (Vcur.norm() * Vbox.norm());

      // 如果速度方向相反，则认为该点是噪声或匹配错误，不计入总点数
      if (velSim < 0) {
        --numPoints;
      } else {
        // 如果点的速度超过动态阈值，则投一票“动态”
        if (Vcur.norm() > this->dynaVelThresh_) {
          ++votes;
        }
      }
    }

    // --- 根据投票结果和速度阈值判断是否为动态 ---
    // 计算动态票的比例
    double voteRatio = (numPoints > 0) ? double(votes) / double(numPoints) : 0;
    // 获取卡尔曼滤波器估计的速度大小
    double velNorm = Vkf.norm();

    // 综合两个条件进行判断:
    // 1. 点云投票率是否足够高
    // 2. 卡尔曼滤波器估计的速度是否足够快
    if (voteRatio >= this->dynaVoteThresh_ && velNorm >= this->dynaVelThresh_) {
      // 如果满足条件，首先标记为“动态候选”
      this->boxHist_[i][0].is_dynamic_candidate = true;

      // --- 动态一致性检查 ---
      // 检查过去几帧是否也一直被认为是动态的，以增加鲁棒性
      int dynaConsistCount = 0;
      if (int(this->boxHist_[i].size()) >= this->dynamicConsistThresh_) {
        for (int j = 0; j < this->dynamicConsistThresh_; ++j) {
          // 如果是动态候选、已经是动态，则计数
          if (this->boxHist_[i][j].is_dynamic_candidate or
              this->boxHist_[i][j].is_dynamic) {
            ++dynaConsistCount;
          }
        }
      }
      // 如果连续几帧都满足条件
      if (dynaConsistCount == this->dynamicConsistThresh_) {
        // 则正式标记为动态，并添加到本轮的动态障碍物列表中
        this->boxHist_[i][0].is_dynamic = true;
        dynamicBBoxesTemp.push_back(this->boxHist_[i][0]);
      }
    }
  }

  // 直接更新最终的动态障碍物列表（已移除尺寸过滤）
  this->dynamicBBoxes_ = dynamicBBoxesTemp;

  hasNewTracking_ = false; // 标记跟踪结果已处理

  // // [Performance Timing] 输出耗时
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end_time -
  // start_time); ROS_INFO_THROTTLE(1.0, "%s: classificationCB took %.3f ms",
  // this->hint_.c_str(), duration.count() / 1000.0);
}

// 可视化定时器回调函数
void dynamicDetector::visCB(const ros::TimerEvent &) {
  // // [Performance Timing] 测量回调函数耗时
  // auto start_time = std::chrono::high_resolution_clock::now();

  //----------------------------障碍物检测阶段----------------------------------------
  // 从原始（未降采样）的激光雷达数据中提取并发布动态点云，以获得更密集的视觉效果
  this->publishRawDynamicPoints();
  this->publishFilteredPoints();
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
  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end_time -
  // start_time); ROS_INFO_THROTTLE(1.0, "%s: visCB took %.3f ms",
  // this->hint_.c_str(), duration.count() / 1000.0);
}

/*!
 * @brief 对单个边界框进行物体分类 (重构版)
 * @param bbox 待分类的边界框（引用传递，会修改其分类标志）
 * @param centroid 点云质心坐标 [x, y, z, 1]
 * @param clusterStd 点云PCA标准差 [std_x, std_y, std_z]
 * @param maxHistorySize 历史最大尺寸 [max_x, max_y, max_z]
 */
void dynamicDetector::classifyBox(onboardDetector::box3D &bbox,
                                  const Eigen::Vector4f &centroid,
                                  const Eigen::Vector3d &clusterStd,
                                  const Eigen::Vector3d &maxHistorySize) {
  // 使用历史最大尺寸进行判断，抵抗遮挡和距离衰减
  double x_width = maxHistorySize.x();
  double y_width = maxHistorySize.y();
  double z_width = maxHistorySize.z();
  double centroid_z = centroid(2);

  // 计算x、y轴的最小值和最大值
  double xy_max = std::max(x_width, y_width);

  // PCA 特征提取
  double pca_z = clusterStd(2);
  double pca_xy_max = std::max(clusterStd(0), clusterStd(1));

  // 重置分类标志
  bbox.is_human = false;
  bbox.is_che = false;
  bbox.is_uav = false;
  bbox.is_else = false;

  // 1. 分类为人：
  // - 尺寸：高瘦 (z > xy * ratio)
  // - 形态(PCA)：Z轴离散度主导 (pca_z > pca_xy * ratio)
  // - 质心：靠下
  if (z_width >= xy_max * this->classifyHumanZWidthRatio_ &&
      pca_z > pca_xy_max * this->classifyHumanPcaRatio_ &&
      centroid_z < z_width * this->classifyHumanCentroidZRatio_) {
    bbox.is_human = true;
  }
  // 2. 分类为车：
  // - 尺寸：扁平 (xy > z * ratio)
  // - 形态(PCA)：XY平面离散度主导 (pca_xy > pca_z * ratio)
  // - 质心：靠下
  else if (xy_max >= z_width * this->classifyVehicleXYWidthRatio_ &&
           pca_xy_max > pca_z * this->classifyVehiclePcaRatio_ &&
           centroid_z < z_width * this->classifyVehicleCentroidZRatio_) {
    bbox.is_che = true;
  }
  // 3. 分类为无人机：
  // - 尺寸：小物体 (all < threshold)
  // - 质心：靠上 (悬浮)
  else if (x_width < this->classifyUAVMaxSize_ &&
           y_width < this->classifyUAVMaxSize_ &&
           z_width < this->classifyUAVMaxSize_ &&
           centroid_z > z_width * this->classifyUAVCentroidZRatio_) {
    bbox.is_uav = true;
  }
  // 4. 其他情况
  else {
    bbox.is_else = true;
  }
}

/*!
 * @brief 切换卡尔曼滤波模型
 * @param index 轨迹索引
 * @param bbox 当前边界框（包含最新的分类信息）
 */
void dynamicDetector::switchKalmanModel(int index,
                                        const onboardDetector::box3D &bbox) {
  // 获取当前滤波器
  auto &filter = this->filters_[index];
  Eigen::VectorXd oldState = filter->getState();
  int oldDim = oldState.size();

  // 获取历史轨迹的分类标志(boxHist_[index][0]是上一帧的分类)
  bool oldIsHuman = false;
  bool oldIsChe = false;
  bool oldIsUav = false;
  bool oldIsElse = false;

  if (this->boxHist_[index].size() > 0) {
    oldIsHuman = this->boxHist_[index][0].is_human;
    oldIsChe = this->boxHist_[index][0].is_che;
    oldIsUav = this->boxHist_[index][0].is_uav;
    oldIsElse = this->boxHist_[index][0].is_else;
  }

  // 判断分类是否发生变化
  bool classificationChanged =
      (bbox.is_human != oldIsHuman) || (bbox.is_che != oldIsChe) ||
      (bbox.is_uav != oldIsUav) || (bbox.is_else != oldIsElse);

  // 如果分类没变,无需切换
  if (!classificationChanged) {
    return;
  }

  // 准备新滤波器参数
  std::shared_ptr<KalmanFilterBase> newFilter = nullptr;
  Eigen::VectorXd newState;
  bool needSwitch = false;

  // 提取旧状态的基础信息 (x, y, z, vx, vy, vz)
  double x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0;

  if (oldDim == 6) { // 3D CV [x, y, z, vx, vy, vz]
    x = oldState(0);
    y = oldState(1);
    z = oldState(2);
    vx = oldState(3);
    vy = oldState(4);
    vz = oldState(5);
  } else if (oldDim == 7) {
    // 7维模型：可能是 Human CA 或 Vehicle CTRA
    x = oldState(0);
    y = oldState(1);
    z = oldState(2);

    if (oldIsChe) {
      // 旧模型为 CTRA [x, y, z, v, a, yaw, yaw_rate]
      double v = oldState(3);
      double yaw = oldState(5);
      vx = v * cos(yaw);
      vy = v * sin(yaw);
      vz = 0;
    } else {
      // 旧模型为 Human CA [x, y, z, vx, vy, ax, ay]
      vx = oldState(3);
      vy = oldState(4);
      vz = 0;
    }
  } else if (oldDim == 9) { // 3D CA [x, y, z, vx, vy, vz, ax, ay, az]
    x = oldState(0);
    y = oldState(1);
    z = oldState(2);
    vx = oldState(3);
    vy = oldState(4);
    vz = oldState(5);
  }

  // 根据新的分类结果创建对应的滤波器
  if (bbox.is_human) {
    // 切换到 Human CA (2D CA, 7维)
    newFilter = createKalmanFilter(true, false, false, false, this->kfParams_);
    newState.resize(7);
    // Human State: [x, y, z, vx, vy, ax, ay]
    newState << x, y, z, vx, vy, 0, 0;
    needSwitch = true;
  } else if (bbox.is_che) {
    // 切换到 Vehicle CTRA (7维)
    newFilter = createKalmanFilter(false, true, false, false, this->kfParams_);
    newState.resize(7);
    // CTRA State: [x, y, z, v, a, yaw, yaw_rate]
    double v = sqrt(vx * vx + vy * vy);
    double yaw = atan2(vy, vx);
    newState << x, y, z, v, 0, yaw, 0;
    needSwitch = true;
  } else if (bbox.is_uav) {
    // 切换到 UAV CA (3D CA, 9维)
    newFilter = createKalmanFilter(false, false, true, false, this->kfParams_);
    newState.resize(9);
    // 3D CA State: [x, y, z, vx, vy, vz, ax, ay, az]
    newState << x, y, z, vx, vy, vz, 0, 0, 0;
    needSwitch = true;
  } else if (bbox.is_else) {
    // 切换到 3D CV (6维)
    newFilter = createKalmanFilter(false, false, false, true, this->kfParams_);
    newState.resize(6);
    // 3D CV State: [x, y, z, vx, vy, vz]
    newState << x, y, z, vx, vy, vz;
    needSwitch = true;
  }

  // 执行切换
  if (needSwitch && newFilter) {
    newFilter->setDt(this->dt_);
    // 用旧模型预测后的状态初始化新模型
    // 注意：oldState是在boxAssociation中predict()后的状态
    // 因此这里不需要再predict()，直接initialize即可
    newFilter->initialize(newState);

    this->filters_[index] = newFilter;
    ROS_INFO_STREAM(this->hint_
                    << " Switched model for object " << index << " (Dim "
                    << oldDim << " -> " << newState.size() << ") to "
                    << (bbox.is_human
                            ? "Human"
                            : (bbox.is_che ? "Vehicle"
                                           : (bbox.is_uav ? "UAV" : "Else"))));
  }
}

/*!
 * 该函数通过激光雷达点云数据检测环境中的障碍物。它会初始化激光雷达检测器（如果尚未初始化），
 * 执行DBSCAN聚类算法来识别点云中的不同对象，并过滤掉尺寸过大的边界框。
 * 最终结果保存在lidarBBoxes_和lidarClusters_成员变量中。
 */
void dynamicDetector::lidarDetect() {
  // 检查是否有激光雷达点云数据（提前返回避免不必要的处理）
  if (this->lidarCloud_ == NULL) {
    ROS_WARN_THROTTLE(1.0, "%s: No point cloud available for detection",
                      this->hint_.c_str());
    return;
  }

  // 执行检测（检测器已在initParam中初始化）
  // 将点云数据传递给检测器并执行DBSCAN聚类
  this->lidarDetector_->getPointcloud(this->lidarCloud_);
  this->lidarDetector_->lidarDBSCAN();

  // 获取聚类结果和对应的边界框
  std::vector<onboardDetector::Cluster> lidarClustersRaw =
      this->lidarDetector_->getClusters();
  std::vector<onboardDetector::Cluster> lidarClustersFiltered;
  std::vector<onboardDetector::box3D> lidarBBoxesRaw =
      this->lidarDetector_->getBBoxes();
  std::vector<onboardDetector::box3D> lidarBBoxesFiltered;

  // 遍历所有边界框，过滤掉尺寸过大的对象并进行分类
  for (int i = 0; i < int(lidarBBoxesRaw.size()); ++i) {
    onboardDetector::box3D lidarBBox = lidarBBoxesRaw[i];
    // 过滤掉尺寸超过阈值的边界框
    if (lidarBBox.x_width > this->maxObjectSize_(0) ||
        lidarBBox.y_width > this->maxObjectSize_(1) ||
        lidarBBox.z_width > this->maxObjectSize_(2)) {
      continue;
    }

    lidarBBoxesFiltered.push_back(lidarBBox);
    lidarClustersFiltered.push_back(lidarClustersRaw[i]);
  }

  // 保存过滤后的结果
  this->lidarBBoxes_ = lidarBBoxesFiltered;
  this->lidarClusters_ = lidarClustersFiltered;

  // 临时存储来自激光雷达的边界框及其点云特征
  std::vector<onboardDetector::box3D> lidarBBoxesTemp;
  std::vector<std::vector<Eigen::Vector3d>> lidarPcClustersTemp;
  std::vector<Eigen::Vector3d> lidarPcClusterCentersTemp;
  std::vector<Eigen::Vector3d> lidarPcClusterStdsTemp; // 存储激光雷达输出

  // 获取激光雷达边界框及其对应的点云簇和特征
  for (size_t i = 0; i < this->lidarBBoxes_.size(); ++i) {
    onboardDetector::box3D lidarBBox = this->lidarBBoxes_[i];
    onboardDetector::Cluster cluster = this->lidarClusters_[i];

    std::vector<Eigen::Vector3d> pcCluster;
    for (const pcl::PointXYZ &point : cluster.points->points) {
      pcCluster.emplace_back(point.x, point.y, point.z);
    }

    // 提取点云簇的质心
    Eigen::Vector3d clusterCenter(cluster.centroid[0], cluster.centroid[1],
                                  cluster.centroid[2]);

    // 计算点云簇的标准差
    Eigen::Vector3d clusterStd =
        cluster.eigen_values.cwiseSqrt().cast<double>();

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
 * @param[out] bestMatch
 * 最佳匹配结果，bestMatch[i]表示第i个当前检测对应的历史轨迹索引（-1表示新目标）
 *
 * 关联流程：
 * 1. 首次检测：初始化历史记录和卡尔曼滤波器
 * 2. 后续检测：
 *    - 构建代价矩阵（基于马氏距离、尺寸差异、点云标准差差异）
 *    - 使用匈牙利算法求解最优匹配
 *    - 应用关联门限，拒绝不可靠匹配
 */
void dynamicDetector::boxAssociation(std::vector<int> &bestMatch) {
  int numCurrObjs = int(this->filteredBBoxes_.size());

  // 第一次检测：初始化所有目标
  if (this->boxHist_.size() == 0) {
    this->boxHist_.resize(numCurrObjs);
    this->pcHist_.resize(numCurrObjs);
    this->pcCenterHist_.resize(numCurrObjs);
    this->pcStdHist_.resize(numCurrObjs);
    this->maxHistorySizes_.resize(numCurrObjs);
    bestMatch.resize(numCurrObjs, -1);

    for (int i = 0; i < numCurrObjs; ++i) {
      this->boxHist_[i].push_back(this->filteredBBoxes_[i]);
      this->pcHist_[i].push_back(this->filteredPcClusters_[i]);
      this->pcCenterHist_[i].push_back(this->filteredPcClusterCenters_[i]);
      this->pcStdHist_[i].push_back(this->filteredPcClusterStds_[i]);

      // 初始化历史最大尺寸
      this->maxHistorySizes_[i] = Eigen::Vector3d(
          this->filteredBBoxes_[i].x_width, this->filteredBBoxes_[i].y_width,
          this->filteredBBoxes_[i].z_width);

      // 强制所有目标使用 3D CV 模型
      auto &bbox = this->filteredBBoxes_[i];
      auto newFilter = createKalmanFilter(false, // is_human
                                          false, // is_che
                                          false, // is_uav
                                          true,  // is_else -> 强制使用 3D CV
                                          this->kfParams_);

      newFilter->setDt(this->dt_);

      // 统一使用 3D CV 初始化：[x, y, z, vx, vy, vz]
      Eigen::VectorXd detection(6);
      detection(0) = bbox.x;
      detection(1) = bbox.y;
      detection(2) = bbox.z;
      detection(3) = 0.0; // vx
      detection(4) = 0.0; // vy
      detection(5) = 0.0; // vz

      newFilter->initialize(detection);
      this->filters_.push_back(newFilter);
    }
  } else if (this->newDetectFlag_) {
    // 后续检测：使用匈牙利算法进行关联
    int numHistObjs = int(this->boxHist_.size());
    bestMatch.resize(numCurrObjs, -1);

    // 首先对所有历史轨迹的卡尔曼滤波器执行预测步骤
    for (int j = 0; j < numHistObjs; ++j) {
      this->filters_[j]->setDt(this->dt_);
      this->filters_[j]->predict();
    }

    // 构建代价矩阵
    std::vector<std::vector<double>> costMatrix(
        numCurrObjs, std::vector<double>(numHistObjs, 1e9));

    //  遍历当前障碍物与所有历史轨迹的代价，即矩阵的行，为当前检测到的障碍物，列为按顺序排好的每个历史轨迹
    for (int i = 0; i < numCurrObjs; ++i) {
      const onboardDetector::box3D &currBox = this->filteredBBoxes_[i];
      const Eigen::Vector3d &currStd = this->filteredPcClusterStds_[i];

      for (int j = 0; j < numHistObjs; ++j) {
        // 获取历史轨迹的最新状态
        const onboardDetector::box3D &histBox = this->boxHist_[j][0];
        const Eigen::Vector3d &histStd = this->pcStdHist_[j][0];

        // 使用卡尔曼滤波器预测的位置构建预测bbox
        // 注意：predBox的位置来自卡尔曼滤波预测，尺寸保持历史值（尺寸不参与状态估计）
        onboardDetector::box3D predBox = histBox;
        const Eigen::VectorXd &filterStates = this->filters_[j]->getState();

        // 根据不同的滤波器类型提取预测位置
        // 注意: 所有模型的状态向量中位置都是三维的 [x, y, z, ...]
        predBox.x = filterStates(0);
        predBox.y = filterStates(1);
        predBox.z = filterStates(2);

        // 预测状态使用历史尺寸和点云标准差（这些不参与卡尔曼滤波）
        // predBox的尺寸已经在初始化时从histBox复制，无需额外设置
        const Eigen::Vector3d &predStd = histStd; // 点云标准差使用历史值

        // 计算关联代价：所有物体统一使用3D马氏距离
        // 注意：所有模型的状态向量中位置都是3维的[x,y,z,...]
        // 马氏距离会自动根据协方差矩阵处理不确定性
        // (例如，如果z方向不确定性大，z的差异对距离贡献会被自动降权)
        const Eigen::MatrixXd &P = this->filters_[j]->getCovariance();

        // 提取位置部分的协方差（状态向量的前3x3块）
        Eigen::Matrix3d P_pos = P.block<3, 3>(0, 0);

        double cost = this->computeAssociationCost3D(predBox, predStd, currBox,
                                                     currStd, P_pos);

        // 统一使用3D门限
        double gateThreshold = this->gateThreshold3D_;

        // 应用关联门限(使用动态门限)
        if (cost < gateThreshold) {
          costMatrix[i][j] = cost;
          // 调试:输出通过门限的匹配
          // ROS_DEBUG_THROTTLE(
          //     0.5, "%s: Match [curr:%d->hist:%d] PASS: cost=%.2f <
          //     gate=%.2f", this->hint_.c_str(), i, j, cost, gateThreshold);
        } else {
          // 调试:输出被门限拒绝的匹配
          ROS_WARN_THROTTLE(
              1.0,
              "%s: Match [curr:%d->hist:%d] REJECT: cost=%.2f >= gate=%.2f",
              this->hint_.c_str(), i, j, cost, gateThreshold);
        }
      }
    }

    // 使用匈牙利算法求解最优匹配
    this->hungarianAlgorithm(costMatrix, bestMatch);

    // 统计并输出关联结果
    int numMatched = 0;
    int numNewTargets = 0;
    for (int i = 0; i < numCurrObjs; ++i) {
      if (bestMatch[i] >= 0) {
        numMatched++;
      } else {
        numNewTargets++;
      }
    }
    int numLostTargets = numHistObjs - numMatched;

    // 简洁的日志输出
    ROS_INFO_THROTTLE(
        0.5, "%s: boxAssociation[currBox:%d histBox:%d] -> [o:%d +:%d -:%d]",
        this->hint_.c_str(), numCurrObjs, numHistObjs, numMatched,
        numNewTargets, numLostTargets);
  }

  this->newDetectFlag_ = false;
}

/*!
 * @brief 计算3D马氏距离
 * @param posDiff 位置差异向量 [dx, dy, dz]
 * @param covariance 协方差矩阵 3x3
 * @return 马氏距离的平方
 */
double dynamicDetector::computeMahalanobisDistance3D(
    const Eigen::Vector3d &posDiff, const Eigen::Matrix3d &covariance) {
  // 计算马氏距离，使用协方差矩阵的逆
  double det = covariance.determinant();
  if (std::abs(det) < 1e-10) {
    // 协方差矩阵奇异，退化为欧式距离
    return posDiff.squaredNorm();
  }
  Eigen::Matrix3d covInv = covariance.inverse();
  double mahalDist = posDiff.transpose() * covInv * posDiff;
  if (!std::isfinite(mahalDist)) {
    // 计算异常，退化为欧式距离
    return posDiff.squaredNorm();
  }
  return mahalDist;
}

/*!
 * @brief 计算两个3D边界框的IoU (Intersection over Union)
 * @param box1 第一个边界框
 * @param box2 第二个边界框
 * @return IoU值，范围 [0, 1]，值越大表示重叠度越高
 *
 * IoU计算公式：IoU = Volume(Intersection) / Volume(Union)
 * 其中：
 * - Intersection: 两个边界框的交集体积
 * - Union: 两个边界框的并集体积 = Vol1 + Vol2 - Intersection
 */
double dynamicDetector::compute3DIoU(const onboardDetector::box3D &box1,
                                     const onboardDetector::box3D &box2) {
  // 计算每个边界框在x、y、z轴上的最小值和最大值
  double box1_x_min = box1.x - box1.x_width / 2.0;
  double box1_x_max = box1.x + box1.x_width / 2.0;
  double box1_y_min = box1.y - box1.y_width / 2.0;
  double box1_y_max = box1.y + box1.y_width / 2.0;
  double box1_z_min = box1.z - box1.z_width / 2.0;
  double box1_z_max = box1.z + box1.z_width / 2.0;

  double box2_x_min = box2.x - box2.x_width / 2.0;
  double box2_x_max = box2.x + box2.x_width / 2.0;
  double box2_y_min = box2.y - box2.y_width / 2.0;
  double box2_y_max = box2.y + box2.y_width / 2.0;
  double box2_z_min = box2.z - box2.z_width / 2.0;
  double box2_z_max = box2.z + box2.z_width / 2.0;

  // 计算x、y、z三个维度的重叠长度
  double x_overlap = std::max(0.0, std::min(box1_x_max, box2_x_max) -
                                       std::max(box1_x_min, box2_x_min));
  double y_overlap = std::max(0.0, std::min(box1_y_max, box2_y_max) -
                                       std::max(box1_y_min, box2_y_min));
  double z_overlap = std::max(0.0, std::min(box1_z_max, box2_z_max) -
                                       std::max(box1_z_min, box2_z_min));

  // 计算交集体积
  double intersection = x_overlap * y_overlap * z_overlap;

  // 计算两个边界框的体积
  double vol1 = box1.x_width * box1.y_width * box1.z_width;
  double vol2 = box2.x_width * box2.y_width * box2.z_width;

  // 计算并集体积
  double union_vol = vol1 + vol2 - intersection;

  // 避免除零
  if (union_vol < 1e-10) {
    return 0.0;
  }

  // 计算IoU
  double iou = intersection / union_vol;

  // 确保IoU在[0, 1]范围内
  return std::max(0.0, std::min(1.0, iou));
}

/*!
 * @brief 计算3D物体（无人机和其他类）的数据关联总代价
 * @param predBox 预测的边界框（来自卡尔曼滤波器）
 * @param predStd 预测时刻的点云标准差（未使用）
 * @param measBox 当前测量的边界框
 * @param measStd 当前测量的点云标准差（未使用）
 * @param covariance 预测位置的3D协方差矩阵
 * @return 总关联代价（越小越好）
 *
 * 代价函数组成：
 * 1. 位置代价：3D马氏距离（考虑x, y, z和不确定性）
 * 2. IoU代价：3D边界框重叠度（1-IoU）
 */
double dynamicDetector::computeAssociationCost3D(
    const onboardDetector::box3D &predBox, const Eigen::Vector3d &predStd,
    const onboardDetector::box3D &measBox, const Eigen::Vector3d &measStd,
    const Eigen::Matrix3d &covariance) {
  // 1. 计算3D位置代价（马氏距离）
  Eigen::Vector3d posDiff;
  posDiff << (measBox.x - predBox.x), (measBox.y - predBox.y),
      (measBox.z - predBox.z);
  double posCost = this->computeMahalanobisDistance3D(posDiff, covariance);

  // 2. 计算IoU代价（IoU越大，代价越小）
  double iou = this->compute3DIoU(predBox, measBox);
  double iouCost = 1.0 - iou; // IoU=1时代价为0，IoU=0时代价为1

  // 加权总代价
  double totalCost = this->associationPosCostWeight_ * posCost +
                     this->associationIoUCostWeight_ * iouCost;

  // 调试：输出各项代价的详细信息
  // ROS_DEBUG_THROTTLE(0.5,
  //                   "%s: Cost3D - pos:%.2f iou:%.2f(%.3f) total:%.2f",
  //                   this->hint_.c_str(), posCost, iouCost, iou, totalCost);

  return totalCost;
}

/*!
 * @brief 匈牙利算法求解最优分配问题
 * @param costMatrix 代价矩阵 [numCurr x numHist]
 * @param assignment 输出匹配结果，assignment[i] = j
 * 表示第i个当前检测匹配到第j个历史轨迹，-1表示未匹配
 *
 * 算法步骤：
 * 1. 行归约：每行减去该行最小值
 * 2. 列归约：每列减去该列最小值
 * 3. 贪婪匹配：优先选择代价小的匹配
 *
 * 注意：这是简化版匈牙利算法，适用于大多数情况
 */
void dynamicDetector::hungarianAlgorithm(
    const std::vector<std::vector<double>> &costMatrix,
    std::vector<int> &assignment) {
  if (costMatrix.empty()) {
    assignment.clear();
    return;
  }

  int numRows = costMatrix.size();
  int numCols = costMatrix[0].size();
  assignment.resize(numRows, -1);

  // 创建代价矩阵的副本用于修改
  std::vector<std::vector<double>> cost = costMatrix;

  // 1. 行归约，找到每行的最小值，即当前每个障碍物找到与其最合适的历史轨迹
  for (int i = 0; i < numRows; ++i) {
    double minVal = *std::min_element(cost[i].begin(), cost[i].end());
    if (minVal < 1e8) { // 只处理有效代价，找0元素代价
      for (int j = 0; j < numCols; ++j) {
        cost[i][j] -= minVal;
      }
    }
  }

  // 2. 列归约，找到每列的最小值，即历史轨迹找到与其最合适的当前障碍物
  // 这两步下来确保行列都有最合适的"零"
  for (int j = 0; j < numCols; ++j) {
    double minVal = 1e9;
    for (int i = 0; i < numRows; ++i) {
      minVal = std::min(minVal, cost[i][j]);
    }
    if (minVal < 1e8) {
      for (int i = 0; i < numRows; ++i) {
        cost[i][j] -= minVal;
      }
    }
  }

  // 3. 贪婪匹配（简化版）
  std::vector<bool> colUsed(numCols, false);
  std::vector<std::pair<double, std::pair<int, int>>> candidates;

  // 收集所有零代价的候选匹配
  for (int i = 0; i < numRows; ++i) {
    for (int j = 0; j < numCols; ++j) {
      if (cost[i][j] < 1e-6 && costMatrix[i][j] < 1e8) {
        candidates.push_back({costMatrix[i][j], {i, j}});
      }
    }
  }

  // 按原始代价排序
  std::sort(candidates.begin(), candidates.end());

  // 贪婪分配，遍历排序后的候选列表。如果某一对 (row, col)
  // 对应的行和列都还没有被占用，就锁定这个匹配。
  std::vector<bool> rowUsed(numRows, false);
  for (const auto &candidate : candidates) {
    int row = candidate.second.first;
    int col = candidate.second.second;

    if (!rowUsed[row] && !colUsed[col]) {
      assignment[row] = col;
      rowUsed[row] = true;
      colUsed[col] = true;
    }
  }
}

// 使用卡尔曼滤波器并更新历史记录
void dynamicDetector::kalmanFilterAndUpdateHist(
    const std::vector<int> &bestMatch) {
  // --- 初始化临时容器 ---
  std::vector<std::deque<onboardDetector::box3D>> boxHistTemp;
  std::vector<std::deque<std::vector<Eigen::Vector3d>>> pcHistTemp;
  std::vector<std::deque<Eigen::Vector3d>> pcCenterHistTemp;
  std::vector<std::deque<Eigen::Vector3d>> pcStdHistTemp;
  std::vector<Eigen::Vector3d> maxHistorySizesTemp;
  std::vector<std::shared_ptr<KalmanFilterBase>> filtersTemp;
  std::vector<int> trackMissedFramesTemp;

  // 确保 trackMissedFrames_ 大小与 boxHist_ 一致
  if (this->trackMissedFrames_.size() != this->boxHist_.size()) {
    this->trackMissedFrames_.resize(this->boxHist_.size(), 0);
  }

  // 为新出现的目标准备的空历史记录模板
  std::deque<onboardDetector::box3D> newSingleBoxHist;
  std::deque<std::vector<Eigen::Vector3d>> newSinglePcHist;
  std::deque<Eigen::Vector3d> newSinglePcCenterHist;
  std::deque<Eigen::Vector3d> newSinglePcStdHist;

  std::vector<onboardDetector::box3D>
      trackedBBoxesTemp; // 存储当前帧滤波后的所有目标框

  newSingleBoxHist.resize(0);
  newSinglePcHist.resize(0);
  newSinglePcCenterHist.resize(0);
  newSinglePcStdHist.resize(0);
  int numCurrObjs = this->filteredBBoxes_.size(); // 当前帧检测到的目标数量
  int numHistObjs = this->boxHist_.size();
  std::vector<bool> isHistMatched(numHistObjs, false);

  // --- 1. 处理当前检测到的目标 (匹配的旧目标 + 新目标) ---
  for (int i = 0; i < numCurrObjs; i++) {
    onboardDetector::box3D currDetectedBBox = this->filteredBBoxes_[i];
    onboardDetector::box3D newEstimatedBBox; // 用于存储卡尔曼滤波后的状态

    // bestMatch[i] 存储的是当前第 i 个检测框所匹配到的历史轨迹的索引
    if (bestMatch[i] >= 0) {
      // --- 情况1：目标匹配成功 (老目标) ---
      int h_idx = bestMatch[i];
      isHistMatched[h_idx] = true;
      trackMissedFramesTemp.push_back(0); // 重置丢失计数

      // 继承该目标之前的历史记录和滤波器
      boxHistTemp.push_back(this->boxHist_[h_idx]);
      pcHistTemp.push_back(this->pcHist_[h_idx]);
      pcCenterHistTemp.push_back(this->pcCenterHist_[h_idx]);
      pcStdHistTemp.push_back(this->pcStdHist_[h_idx]);
      maxHistorySizesTemp.push_back(this->maxHistorySizes_[h_idx]);
      filtersTemp.push_back(this->filters_[h_idx]);

      // 构建测量向量：所有模型都测量3D位置 [x, y, z]
      Eigen::VectorXd measurement(3);
      measurement(0) = currDetectedBBox.x;
      measurement(1) = currDetectedBBox.y;
      measurement(2) = currDetectedBBox.z;

      // 如果启用自适应测量噪声R，则计算并设置
      if (this->adaptiveREnabled_) {
        Eigen::MatrixXd R_adaptive = this->computeAdaptiveMeasNoiseR(
            filtersTemp.size() - 1); // 当前滤波器在临时容器中的索引
        filtersTemp.back()->setMeasNoiseR(R_adaptive);
      }

      // 执行更新步骤 (预测已在 trackingCB 中完成)
      filtersTemp.back()->update(measurement);

      // 从滤波器中提取更新后的状态
      const Eigen::VectorXd &state = filtersTemp.back()->getState();

      if (currDetectedBBox.is_human) {
        // Human CA: [x, y, z, vx, vy, ax, ay]
        newEstimatedBBox.x = state(0);
        newEstimatedBBox.y = state(1);
        newEstimatedBBox.z = state(2);
        newEstimatedBBox.Vx = state(3);
        newEstimatedBBox.Vy = state(4);
        newEstimatedBBox.Vz = 0.0;
        newEstimatedBBox.Ax = state(5);
        newEstimatedBBox.Ay = state(6);
        newEstimatedBBox.Az = 0.0;
      } else if (currDetectedBBox.is_che) {
        // Vehicle CTRA: [x, y, z, v, a, yaw, yaw_rate]
        newEstimatedBBox.x = state(0);
        newEstimatedBBox.y = state(1);
        newEstimatedBBox.z = state(2);
        double v = state(3);
        double yaw = state(5);
        newEstimatedBBox.Vx = v * cos(yaw);
        newEstimatedBBox.Vy = v * sin(yaw);
        newEstimatedBBox.Vz = 0.0;
        newEstimatedBBox.Ax = state(4) * cos(yaw); // a * cos(yaw)
        newEstimatedBBox.Ay = state(4) * sin(yaw); // a * sin(yaw)
        newEstimatedBBox.Az = 0.0;
      } else if (currDetectedBBox.is_uav) {
        // UAV CA: [x, y, z, vx, vy, vz, ax, ay, az]
        newEstimatedBBox.x = state(0);
        newEstimatedBBox.y = state(1);
        newEstimatedBBox.z = state(2);
        newEstimatedBBox.Vx = state(3);
        newEstimatedBBox.Vy = state(4);
        newEstimatedBBox.Vz = state(5);
        newEstimatedBBox.Ax = state(6);
        newEstimatedBBox.Ay = state(7);
        newEstimatedBBox.Az = state(8);
      } else { // is_else
        // Else CV: [x, y, z, vx, vy, vz]
        newEstimatedBBox.x = state(0);
        newEstimatedBBox.y = state(1);
        newEstimatedBBox.z = state(2);
        newEstimatedBBox.Vx = state(3);
        newEstimatedBBox.Vy = state(4);
        newEstimatedBBox.Vz = state(5);
        newEstimatedBBox.Ax = 0.0;
        newEstimatedBBox.Ay = 0.0;
        newEstimatedBBox.Az = 0.0;
      }

      // 边界框的尺寸直接使用当前测量值
      newEstimatedBBox.x_width = currDetectedBBox.x_width;
      newEstimatedBBox.y_width = currDetectedBBox.y_width;
      newEstimatedBBox.z_width = currDetectedBBox.z_width;

      // 使用当前检测框中的最新分类结果
      newEstimatedBBox.is_dynamic = currDetectedBBox.is_dynamic;
      newEstimatedBBox.is_human = currDetectedBBox.is_human;
      newEstimatedBBox.is_che = currDetectedBBox.is_che;
      newEstimatedBBox.is_uav = currDetectedBBox.is_uav;
      newEstimatedBBox.is_else = currDetectedBBox.is_else;
    } else {
      // --- 情况2：目标未匹配 (新目标) ---
      trackMissedFramesTemp.push_back(0);

      boxHistTemp.push_back(newSingleBoxHist);
      pcHistTemp.push_back(newSinglePcHist);
      pcCenterHistTemp.push_back(newSinglePcCenterHist);
      pcStdHistTemp.push_back(newSinglePcStdHist);
      maxHistorySizesTemp.push_back(
          Eigen::Vector3d(currDetectedBBox.x_width, currDetectedBBox.y_width,
                          currDetectedBBox.z_width)); // 初始化最大尺寸

      // 强制所有新轨迹使用 3D CV 模型
      auto newFilter = createKalmanFilter(false, // is_human
                                          false, // is_che
                                          false, // is_uav
                                          true,  // is_else -> 强制使用 3D CV
                                          this->kfParams_);

      newFilter->setDt(this->dt_);

      // 统一使用 3D CV 初始化：[x, y, z, vx, vy, vz]
      Eigen::VectorXd detection(6);
      detection(0) = currDetectedBBox.x;
      detection(1) = currDetectedBBox.y;
      detection(2) = currDetectedBBox.z;
      detection(3) = 0.0; // vx
      detection(4) = 0.0; // vy
      detection(5) = 0.0; // vz

      // 初始化滤波器
      newFilter->initialize(detection);
      filtersTemp.push_back(newFilter);

      // 对于新目标，其初始估计状态就是它的第一次测量值
      newEstimatedBBox = currDetectedBBox;
      newEstimatedBBox.Vx = 0.0;
      newEstimatedBBox.Vy = 0.0;
      newEstimatedBBox.Vz = 0.0;
      newEstimatedBBox.Ax = 0.0;
      newEstimatedBBox.Ay = 0.0;
      newEstimatedBBox.Az = 0.0;
    }

    // --- 更新历史记录队列 ---
    if (int(boxHistTemp.back().size()) == this->histSize_) {
      boxHistTemp.back().pop_back();
      pcHistTemp.back().pop_back();
      pcCenterHistTemp.back().pop_back();
      pcStdHistTemp.back().pop_back();
    }

    // 将当前帧的最新估计状态和信息从队列头部推入
    boxHistTemp.back().push_front(newEstimatedBBox);
    pcHistTemp.back().push_front(this->filteredPcClusters_[i]);
    pcCenterHistTemp.back().push_front(this->filteredPcClusterCenters_[i]);
    pcStdHistTemp.back().push_front(this->filteredPcClusterStds_[i]);

    // 将当前帧的最终跟踪结果存入 trackedBBoxesTemp
    trackedBBoxesTemp.push_back(newEstimatedBBox);
  }

  // --- 2. 处理未匹配的历史目标 (Coasting) ---
  for (int j = 0; j < numHistObjs; ++j) {
    if (!isHistMatched[j]) {
      int missed = this->trackMissedFrames_[j] + 1;
      if (missed < this->maxMissedFrames_) {
        // 保留该轨迹 (Coasting)
        trackMissedFramesTemp.push_back(missed);

        boxHistTemp.push_back(this->boxHist_[j]);
        pcHistTemp.push_back(this->pcHist_[j]);
        pcCenterHistTemp.push_back(this->pcCenterHist_[j]);
        pcStdHistTemp.push_back(this->pcStdHist_[j]);
        maxHistorySizesTemp.push_back(this->maxHistorySizes_[j]);
        filtersTemp.push_back(this->filters_[j]);

        // 获取预测状态 (已在 trackingCB 中 predict)
        const Eigen::VectorXd &state = filtersTemp.back()->getState();

        // 构建预测的 BBox
        onboardDetector::box3D predBBox;
        // 使用上一帧的属性作为基础
        if (boxHistTemp.back().size() > 0) {
          predBBox = boxHistTemp.back().front();
        }

        // 根据模型类型提取预测状态
        // 注意：这里我们使用历史轨迹的分类信息
        if (predBBox.is_human) {
          predBBox.x = state(0);
          predBBox.y = state(1);
          predBBox.z = state(2);
          predBBox.Vx = state(3);
          predBBox.Vy = state(4);
          predBBox.Vz = 0.0;
          predBBox.Ax = state(5);
          predBBox.Ay = state(6);
          predBBox.Az = 0.0;
        } else if (predBBox.is_che) {
          predBBox.x = state(0);
          predBBox.y = state(1);
          predBBox.z = state(2);
          double v = state(3);
          double yaw = state(5);
          predBBox.Vx = v * cos(yaw);
          predBBox.Vy = v * sin(yaw);
          predBBox.Vz = 0.0;
          predBBox.Ax = state(4) * cos(yaw);
          predBBox.Ay = state(4) * sin(yaw);
          predBBox.Az = 0.0;
        } else if (predBBox.is_uav) {
          predBBox.x = state(0);
          predBBox.y = state(1);
          predBBox.z = state(2);
          predBBox.Vx = state(3);
          predBBox.Vy = state(4);
          predBBox.Vz = state(5);
          predBBox.Ax = state(6);
          predBBox.Ay = state(7);
          predBBox.Az = state(8);
        } else {
          predBBox.x = state(0);
          predBBox.y = state(1);
          predBBox.z = state(2);
          predBBox.Vx = state(3);
          predBBox.Vy = state(4);
          predBBox.Vz = state(5);
          predBBox.Ax = 0.0;
          predBBox.Ay = 0.0;
          predBBox.Az = 0.0;
        }

        // 尺寸保持不变 (predBBox 已经复制了上一帧的尺寸)

        // 更新历史队列
        if (int(boxHistTemp.back().size()) == this->histSize_) {
          boxHistTemp.back().pop_back();
          pcHistTemp.back().pop_back();
          pcCenterHistTemp.back().pop_back();
          pcStdHistTemp.back().pop_back();
        }

        boxHistTemp.back().push_front(predBBox);
        // 点云数据推入空值
        pcHistTemp.back().push_front(std::vector<Eigen::Vector3d>());
        pcCenterHistTemp.back().push_front(Eigen::Vector3d::Zero());
        pcStdHistTemp.back().push_front(Eigen::Vector3d::Zero());

        trackedBBoxesTemp.push_back(predBBox);
      }
      // else: 丢弃 (missed >= maxMissedFrames_)
    }
  }

  // --- 更新类的成员变量 ---
  this->boxHist_ = boxHistTemp;
  this->pcHist_ = pcHistTemp;
  this->pcCenterHist_ = pcCenterHistTemp;
  this->pcStdHist_ = pcStdHistTemp;
  this->maxHistorySizes_ = maxHistorySizesTemp;
  this->filters_ = filtersTemp;
  this->trackedBBoxes_ = trackedBBoxesTemp;
  this->trackMissedFrames_ = trackMissedFramesTemp;
}

/*!
 * @brief 计算自适应测量噪声矩阵R
 * @param track_id 轨迹索引
 * @return 自适应调整后的测量噪声协方差矩阵R (3x3)
 *
 * 算法原理:
 * 1. 从历史数据中提取最近K帧的位置测量值
 * 2. 计算相邻帧的位置差分
 * 3. 计算差分的方差，除以2得到测量噪声方差（因为diff包含两次测量的噪声）
 * 4. 根据模型类型进行特定调整（人/车/无人机）
 * 5. 与原始R进行加权融合，并设置下限保护
 */
Eigen::MatrixXd dynamicDetector::computeAdaptiveMeasNoiseR(int track_id) {
  // -------------------------------------------------------------------------
  // 第1步: 参数检查和初始化
  // -------------------------------------------------------------------------

  // 检查track_id有效性
  if (track_id < 0 || track_id >= static_cast<int>(this->boxHist_.size())) {
    ROS_WARN_STREAM(this->hint_ << " Invalid track_id " << track_id
                                << " in computeAdaptiveMeasNoiseR");
    return this->filters_[track_id]->getMeasNoiseR();
  }

  int hist_len = this->boxHist_[track_id].size();

  // 如果历史不足2帧，无法计算差分，返回原始R
  if (hist_len < 2) {
    return this->filters_[track_id]->getMeasNoiseR();
  }

  // 实际使用的帧数 (不能超过历史长度-1，因为要计算差分)
  int actual_K = std::min(this->adaptiveRWindowSize_, hist_len - 1);

  // -------------------------------------------------------------------------
  // 第2步: 提取历史位置测量值
  // -------------------------------------------------------------------------

  std::vector<Eigen::Vector3d> measurements;
  measurements.reserve(actual_K + 1);

  for (int k = 0; k < actual_K + 1; k++) {
    Eigen::Vector3d pos(this->boxHist_[track_id][k].x,
                        this->boxHist_[track_id][k].y,
                        this->boxHist_[track_id][k].z);
    measurements.push_back(pos);
  }

  // -------------------------------------------------------------------------
  // 第3步: 计算相邻帧差分
  // -------------------------------------------------------------------------

  std::vector<Eigen::Vector3d> diffs;
  diffs.reserve(actual_K);

  for (int k = 0; k < actual_K; k++) {
    Eigen::Vector3d diff = measurements[k] - measurements[k + 1];
    diffs.push_back(diff);
  }

  // -------------------------------------------------------------------------
  // 第4步: 计算差分的方差
  // -------------------------------------------------------------------------

  // 4.1 计算均值
  Eigen::Vector3d mean_diff = Eigen::Vector3d::Zero();
  for (const auto &diff : diffs) {
    mean_diff += diff;
  }
  mean_diff /= static_cast<double>(actual_K);

  // 4.2 计算方差
  Eigen::Vector3d variance = Eigen::Vector3d::Zero();
  for (const auto &diff : diffs) {
    Eigen::Vector3d deviation = diff - mean_diff;
    variance += deviation.cwiseProduct(deviation); // 逐元素平方
  }
  variance /= static_cast<double>(actual_K);

  // 4.3 理论修正
  // 因为 diff = meas[k] - meas[k+1]，包含两次独立测量的噪声
  // Var(diff) = Var(n_k) + Var(n_{k+1}) = 2 * Var(n)
  // 所以: Var(n) = Var(diff) / 2
  variance /= 2.0;

  // -------------------------------------------------------------------------
  // 第5步: 构建自适应R矩阵
  // -------------------------------------------------------------------------

  Eigen::MatrixXd R_adaptive = Eigen::MatrixXd::Zero(3, 3);
  for (int i = 0; i < 3; i++) {
    R_adaptive(i, i) = std::max(variance(i), 1e-6); // 避免过小（数值稳定性）
  }

  // -------------------------------------------------------------------------
  // 第6步: 模型特定调整
  // -------------------------------------------------------------------------

  // 6.1 判断当前模型类型
  bool is_human = this->boxHist_[track_id][0].is_human;
  bool is_che = this->boxHist_[track_id][0].is_che;
  bool is_uav = this->boxHist_[track_id][0].is_uav;

  // 6.2 根据模型调整
  if (is_human) {
    // 人的z轴几乎不变，差分包含的噪声可能有地面起伏影响
    // 因此对z轴降权
    R_adaptive(2, 2) *= this->adaptiveRHumanZScale_;

    ROS_DEBUG_STREAM_THROTTLE(5.0, this->hint_
                                       << " Track " << track_id
                                       << " (Human): z-axis R scaled by "
                                       << this->adaptiveRHumanZScale_);
  } else if (is_uav) {
    // 无人机z轴运动剧烈，差分包含真实运动
    // 增加z轴容忍度
    R_adaptive(2, 2) *= this->adaptiveRUavZScale_;

    ROS_DEBUG_STREAM_THROTTLE(5.0, this->hint_ << " Track " << track_id
                                               << " (UAV): z-axis R scaled by "
                                               << this->adaptiveRUavZScale_);
  } else if (is_che) {
    // 车辆: 检查是否在转弯
    const Eigen::VectorXd &state = this->filters_[track_id]->getState();
    double yaw_rate = state(6); // yaw_rate是状态向量的第7个元素

    if (std::abs(yaw_rate) > this->adaptiveRCtraTurnThreshold_) {
      // 转弯时DBSCAN聚类中心会抖动，增加xy方向噪声
      double turn_factor =
          1.0 + this->adaptiveRCtraTurnScale_ * std::abs(yaw_rate);
      R_adaptive(0, 0) *= turn_factor; // x方向
      R_adaptive(1, 1) *= turn_factor; // y方向

      ROS_DEBUG_STREAM_THROTTLE(
          5.0, this->hint_ << " Track " << track_id
                           << " (Vehicle turning, yaw_rate=" << yaw_rate
                           << "): xy R scaled by " << turn_factor);
    }
  }

  // -------------------------------------------------------------------------
  // 第7步: 与原始R融合
  // -------------------------------------------------------------------------

  // 7.1 获取原始R（从yaml配置）
  Eigen::MatrixXd R_original = this->filters_[track_id]->getMeasNoiseR();

  // 7.2 计算融合权重（历史越长，越相信自适应）
  double confidence =
      std::min(1.0, static_cast<double>(actual_K) /
                        static_cast<double>(this->adaptiveRWindowSize_));
  double final_weight = this->adaptiveRWeight_ * confidence;

  // 7.3 加权融合
  Eigen::MatrixXd R_final =
      final_weight * R_adaptive + (1.0 - final_weight) * R_original;

  // 7.4 下限保护（不能太小）
  for (int i = 0; i < 3; i++) {
    double min_value = R_original(i, i) * this->adaptiveRMinRatio_;
    R_final(i, i) = std::max(R_final(i, i), min_value);
  }

  // -------------------------------------------------------------------------
  // 第8步: 调试输出
  // -------------------------------------------------------------------------

  ROS_DEBUG_STREAM_THROTTLE(
      5.0, this->hint_ << " Track " << track_id << " adaptive R: ["
                       << R_final(0, 0) << ", " << R_final(1, 1) << ", "
                       << R_final(2, 2) << "], original R: ["
                       << R_original(0, 0) << ", " << R_original(1, 1) << ", "
                       << R_original(2, 2) << "], hist_len=" << hist_len
                       << ", actual_K=" << actual_K);

  return R_final;
}

/*!
 * @brief 移除重复/重叠的轨迹（用于解决幽灵轨迹问题）
 *
 * 检测逻辑：
 * - 如果两条轨迹的 IoU > 阈值（如 0.5）
 * - 保留丢失帧数较少的（更可靠）
 * - 删除丢失帧数较多的（coasting 轨迹）
 */
void dynamicDetector::removeDuplicateTracks() {
  if (this->boxHist_.size() <= 1) {
    return; // 只有一条或零条轨迹，无需去重
  }

  // 使用配置的 IoU 阈值
  std::vector<bool> toRemove(this->boxHist_.size(), false);

  // 检查所有轨迹对
  for (size_t i = 0; i < this->boxHist_.size(); ++i) {
    if (toRemove[i] || this->boxHist_[i].empty())
      continue;

    for (size_t j = i + 1; j < this->boxHist_.size(); ++j) {
      if (toRemove[j] || this->boxHist_[j].empty())
        continue;

      // 计算两条轨迹最新状态的 IoU
      const auto &bbox_i = this->boxHist_[i][0];
      const auto &bbox_j = this->boxHist_[j][0];
      double iou = this->compute3DIoU(bbox_i, bbox_j);

      // 如果重叠度高，删除丢失帧数多的那条
      if (iou > this->duplicateTrackIoUThreshold_) {
        int missed_i = this->trackMissedFrames_[i];
        int missed_j = this->trackMissedFrames_[j];

        if (missed_i > missed_j) {
          toRemove[i] = true;
          ROS_WARN_THROTTLE(1.0,
                            "%s: Removing duplicate track %zu (missed=%d, "
                            "IoU=%.2f with track %zu)",
                            this->hint_.c_str(), i, missed_i, iou, j);
          break; // i 已被标记删除，无需继续比较
        } else {
          toRemove[j] = true;
          ROS_WARN_THROTTLE(1.0,
                            "%s: Removing duplicate track %zu (missed=%d, "
                            "IoU=%.2f with track %zu)",
                            this->hint_.c_str(), j, missed_j, iou, i);
        }
      }
    }
  }

  // 执行删除（倒序删除避免索引偏移）
  for (int i = this->boxHist_.size() - 1; i >= 0; --i) {
    if (toRemove[i]) {
      this->boxHist_.erase(this->boxHist_.begin() + i);
      this->pcHist_.erase(this->pcHist_.begin() + i);
      this->pcCenterHist_.erase(this->pcCenterHist_.begin() + i);
      this->pcStdHist_.erase(this->pcStdHist_.begin() + i);
      this->maxHistorySizes_.erase(this->maxHistorySizes_.begin() + i);
      this->filters_.erase(this->filters_.begin() + i);
      this->trackedBBoxes_.erase(this->trackedBBoxes_.begin() + i);
      this->trackMissedFrames_.erase(this->trackMissedFrames_.begin() + i);
    }
  }
}

// 获取动态点云
void dynamicDetector::getDynamicPc(std::vector<Eigen::Vector3d> &dynamicPc) {
  Eigen::Vector3d curPoint;
  for (size_t i = 0; i < this->filteredPcClusters_.size(); ++i) {
    for (size_t j = 0; j < this->filteredPcClusters_[i].size(); ++j) {
      curPoint = this->filteredPcClusters_[i][j];
      for (size_t k = 0; k < this->dynamicBBoxes_.size(); ++k) {
        if (abs(curPoint(0) - this->dynamicBBoxes_[k].x) <=
                this->dynamicBBoxes_[k].x_width / 2 and
            abs(curPoint(1) - this->dynamicBBoxes_[k].y) <=
                this->dynamicBBoxes_[k].y_width / 2 and
            abs(curPoint(2) - this->dynamicBBoxes_[k].z) <=
                this->dynamicBBoxes_[k].z_width / 2) {
          dynamicPc.push_back(curPoint);
          break;
        }
      }
    }
  }
}

// 发布点云
void dynamicDetector::publishPoints(const std::vector<Eigen::Vector3d> &points,
                                    const ros::Publisher &publisher) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (size_t i = 0; i < points.size(); ++i) {
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
void dynamicDetector::publish3dBox(const std::vector<box3D> &boxes,
                                   const ros::Publisher &publisher, double r,
                                   double g, double b) {
  // 创建一个MarkerArray消息，用于批量发布多个Marker
  visualization_msgs::MarkerArray markers;

  // 遍历所有传入的边界框
  for (size_t i = 0; i < boxes.size(); i++) {
    // 为每个边界框创建一个LINE_LIST类型的Marker
    visualization_msgs::Marker line;
    line.header.frame_id = "map"; // 设置Marker的坐标系为"map"
    line.ns = "box3D";            // 设置Marker的命名空间
    line.id = i;                  // 为Marker设置唯一的ID
    line.type = visualization_msgs::Marker::
        LINE_LIST; // Marker类型为线列表，用于绘制立方体的边
    line.action = visualization_msgs::Marker::ADD; // 操作类型为添加或修改
    line.scale.x = 0.06;                           // 设置线的宽度

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
    double z_width = top / 2.0;     // 将可视化Marker的高度设为最高点的一半
    line.pose.position.z = z_width; // 将可视化Marker的中心Z坐标设为该值

    // 定义立方体的8个顶点
    geometry_msgs::Point corner[8];
    corner[0].x = -x_width / 2.0;
    corner[0].y = -y_width / 2.0;
    corner[0].z = -z_width;
    corner[1].x = -x_width / 2.0;
    corner[1].y = y_width / 2.0;
    corner[1].z = -z_width;
    corner[2].x = x_width / 2.0;
    corner[2].y = y_width / 2.0;
    corner[2].z = -z_width;
    corner[3].x = x_width / 2.0;
    corner[3].y = -y_width / 2.0;
    corner[3].z = -z_width;

    corner[4].x = -x_width / 2.0;
    corner[4].y = -y_width / 2.0;
    corner[4].z = z_width;
    corner[5].x = -x_width / 2.0;
    corner[5].y = y_width / 2.0;
    corner[5].z = z_width;
    corner[6].x = x_width / 2.0;
    corner[6].y = y_width / 2.0;
    corner[6].z = z_width;
    corner[7].x = x_width / 2.0;
    corner[7].y = -y_width / 2.0;
    corner[7].z = z_width;

    // 定义连接8个顶点的12条边
    int edgeIdx[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, // 底部四条边
        {4, 5}, {5, 6}, {6, 7}, {7, 4}, // 顶部四条边
        {0, 4}, {1, 5}, {2, 6}, {3, 7}  // 连接上下面的四条垂直边
    };

    // 将12条边的端点添加到Marker的点列表中
    for (int e = 0; e < 12; e++) {
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
void dynamicDetector::publishHistoryTraj() {
  visualization_msgs::MarkerArray trajMsg;
  int countMarker = 0;
  for (size_t i = 0; i < this->boxHist_.size(); ++i) {
    // std::cout << "this->boxHist_[i].size() = "  << this->boxHist_[i].size()
    // << std::endl;
    if (this->boxHist_[i].size() > 1) {
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
      for (size_t j = 0; j < this->boxHist_[i].size() - 1; ++j) {
        geometry_msgs::Point p1, p2;
        onboardDetector::box3D box1 = this->boxHist_[i][j];
        onboardDetector::box3D box2 = this->boxHist_[i][j + 1];
        p1.x = box1.x;
        p1.y = box1.y;
        p1.z = box1.z;
        p2.x = box2.x;
        p2.y = box2.y;
        p2.z = box2.z;
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
void dynamicDetector::publishVelVis() {
  visualization_msgs::MarkerArray velVisMsg;
  int countMarker = 0;
  for (size_t i = 0; i < this->trackedBBoxes_.size(); ++i) {
    visualization_msgs::Marker velMarker;
    velMarker.header.frame_id = "map";
    velMarker.header.stamp = ros::Time::now();
    velMarker.ns = "dynamic_detector";
    velMarker.id = countMarker;
    velMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    velMarker.pose.position.x = this->trackedBBoxes_[i].x;
    velMarker.pose.position.y = this->trackedBBoxes_[i].y;
    velMarker.pose.position.z =
        this->trackedBBoxes_[i].z + this->trackedBBoxes_[i].z_width / 2. + 0.3;
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
    double vNorm = sqrt(vx * vx + vy * vy);
    std::string velText = "Vx=" + std::to_string(vx) +
                          ", Vy=" + std::to_string(vy) +
                          ", |V|=" + std::to_string(vNorm);
    velMarker.text = velText;
    velVisMsg.markers.push_back(velMarker);
    ++countMarker;
  }
  this->velVisPub_.publish(velVisMsg);
}

// 发布过滤后的点
void dynamicDetector::publishFilteredPoints() {
  sensor_msgs::PointCloud2 filteredPointsMsg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  for (size_t i = 0; i < this->filteredPcClusters_.size(); ++i) {
    std_msgs::ColorRGBA color;
    color.r = 0.5;
    color.g = 0.5;
    color.b = 0.5;
    color.a = 1.0;

    for (size_t j = 0; j < this->filteredPcClusters_[i].size(); ++j) {
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
void dynamicDetector::publishRawDynamicPoints() {
  // 检查是否有最新的点云数据，如果没有则直接返回
  if (not this->latestCloud_) {
    return;
  }
  try {
    // 创建一个PCL点云指针，用于存储全局坐标系下的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // 检查是否有传感器位姿信息
    if (this->hasSensorPose_) {
      // 如果有位姿，则将ROS消息格式的原始点云转换到PCL格式
      pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(
          new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*this->latestCloud_, *tempCloud);

      // 创建一个仿射变换矩阵，用于将点云从激光雷达坐标系转换到全局（map）坐标系
      Eigen::Affine3d transform = Eigen::Affine3d::Identity();
      transform.linear() = this->orientationLidar_;   // 设置旋转部分
      transform.translation() = this->positionLidar_; // 设置平移部分

      // 对点云应用变换
      pcl::transformPointCloud(*tempCloud, *globalCloud, transform);

      // 创建一个ROS点云消息用于发布可视化
      sensor_msgs::PointCloud2 cloudMsg;
      pcl::toROSMsg(*globalCloud, cloudMsg);
      cloudMsg.header.frame_id = "map";         // 设置坐标系为 "map"
      cloudMsg.header.stamp = ros::Time::now(); // 设置时间戳
      this->rawLidarPointsPub_.publish(
          cloudMsg); // 发布转换到全局坐标系的原始点云

      // 终端输出测试：打印发布点云信息到ROS日志/终端
      // std::cout << ": No time step parameter found. Use default: 0.033." <<
      // std::endl; ROS_INFO_STREAM(this->hint_ << " publishRawDynamicPoints:
      // transformed pointcloud published. "
      //                 << "hasSensorPose=" << (this->hasSensorPose_ ? "true"
      //                 : "false")
      //                 << ", points=" << globalCloud->points.size());
    } else {
      // 如果没有位姿信息，直接将ROS消息转换为PCL点云（假设其已在全局坐标系）
      pcl::fromROSMsg(*this->latestCloud_, *globalCloud);
    }

    // 创建一个向量，用于存储属于动态障碍物的点的坐标
    std::vector<Eigen::Vector3d> dynamicEigenPoints;

    // 遍历所有已识别的动态边界框
    for (const auto &box : this->dynamicBBoxes_) {
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
      for (const auto &point : globalCloud->points) {
        // 检查点是否在当前动态边界框内部
        if (point.x >= xmin && point.x <= xmax && point.y >= ymin &&
            point.y <= ymax && point.z >= zmin && point.z <= zmax) {
          // 如果点在框内，则将其添加到动态点向量中
          dynamicEigenPoints.push_back(
              Eigen::Vector3d(point.x, point.y, point.z));
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
  catch (const pcl::PCLException &e) {
    ROS_ERROR("PCL Exception during dynamic point extraction: %s", e.what());
  }
  // 捕获并报告标准C++库可能抛出的异常
  catch (const std::exception &e) {
    ROS_ERROR("Standard Exception during dynamic point extraction: %s",
              e.what());
  }
  // 捕获所有其他类型的未知异常
  catch (...) {
    ROS_ERROR("Unknown error during dynamic point extraction.");
  }
}

// 用户函数：获取动态障碍物
void dynamicDetector::getDynamicObstacles(
    std::vector<onboardDetector::box3D> &incomeDynamicBBoxes,
    const Eigen::Vector3d &robotSize) {
  incomeDynamicBBoxes.clear();
  for (int i = 0; i < int(this->dynamicBBoxes_.size()); i++) {
    onboardDetector::box3D box = this->dynamicBBoxes_[i];
    box.x_width += robotSize(0);
    box.y_width += robotSize(1);
    box.z_width += robotSize(2);
    incomeDynamicBBoxes.push_back(box);
  }
}

// 用户函数：获取动态障碍物历史
void dynamicDetector::getDynamicObstaclesHist(
    std::vector<std::vector<Eigen::Vector3d>> &posHist,
    std::vector<std::vector<Eigen::Vector3d>> &velHist,
    std::vector<std::vector<Eigen::Vector3d>> &sizeHist,
    const Eigen::Vector3d &robotSize) {
  posHist.clear();
  velHist.clear();
  sizeHist.clear();

  if (this->boxHist_.size()) {
    for (size_t i = 0; i < this->boxHist_.size(); ++i) {
      if (this->boxHist_[i][0].is_dynamic or this->boxHist_[i][0].is_human) {
        std::vector<Eigen::Vector3d> obPosHist, obVelHist, obSizeHist;
        for (size_t j = 0; j < this->boxHist_[i].size(); ++j) {
          Eigen::Vector3d pos(this->boxHist_[i][j].x, this->boxHist_[i][j].y,
                              this->boxHist_[i][j].z);
          Eigen::Vector3d vel(this->boxHist_[i][j].Vx, this->boxHist_[i][j].Vy,
                              0);
          Eigen::Vector3d size(this->boxHist_[i][j].x_width,
                               this->boxHist_[i][j].y_width,
                               this->boxHist_[i][j].z_width);
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

// 将Livox CustomMsg格式转换为PointCloud2格式
void dynamicDetector::convertCustomMsgToPointCloud2(
    const livox_ros_driver2::CustomMsgConstPtr &customMsg,
    sensor_msgs::PointCloud2 &cloud) {
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
  for (size_t i = 0; i < customMsg->points.size(); ++i) {
    const auto &point = customMsg->points[i];
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end_time -
  // start_time); ROS_INFO_THROTTLE(1.0, "%s: CustomMsg to PointCloud2
  // conversion took %.3f ms for %u points",
  //                   this->hint_.c_str(), duration.count() / 1000.0,
  //                   customMsg->point_num);
}

// Livox CustomMsg + Pose 回调函数
void dynamicDetector::lidarCustomPoseCB(
    const livox_ros_driver2::CustomMsgConstPtr &customMsg,
    const geometry_msgs::PoseStampedConstPtr &pose) {
  // // [Performance Timing] 测量回调函数耗时
  // auto start_time = std::chrono::high_resolution_clock::now();

  // 将CustomMsg转换为PointCloud2
  sensor_msgs::PointCloud2 cloudMsg;
  this->convertCustomMsgToPointCloud2(customMsg, cloudMsg);

  // 转换为ConstPtr并调用原有的处理函数
  sensor_msgs::PointCloud2ConstPtr cloudMsgPtr =
      boost::make_shared<sensor_msgs::PointCloud2>(cloudMsg);
  this->lidarPoseCB(cloudMsgPtr, pose);

  // // [Performance Timing] 输出耗时
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end_time -
  // start_time); ROS_INFO_THROTTLE(1.0, "%s: lidarCustomPoseCB took %.3f ms",
  // this->hint_.c_str(), duration.count() / 1000.0);
}

// Livox CustomMsg + Odometry 回调函数
void dynamicDetector::lidarCustomOdomCB(
    const livox_ros_driver2::CustomMsgConstPtr &customMsg,
    const nav_msgs::OdometryConstPtr &odom) {
  // // [Performance Timing] 测量回调函数耗时
  // auto start_time = std::chrono::high_resolution_clock::now();

  // 将CustomMsg转换为PointCloud2
  sensor_msgs::PointCloud2 cloudMsg;
  this->convertCustomMsgToPointCloud2(customMsg, cloudMsg);

  // 转换为ConstPtr并调用原有的处理函数
  sensor_msgs::PointCloud2ConstPtr cloudMsgPtr =
      boost::make_shared<sensor_msgs::PointCloud2>(cloudMsg);
  this->lidarOdomCB(cloudMsgPtr, odom);

  // // [Performance Timing] 输出耗时
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration =
  // std::chrono::duration_cast<std::chrono::microseconds>(end_time -
  // start_time); ROS_INFO_THROTTLE(1.0, "%s: lidarCustomOdomCB took %.3f ms",
  // this->hint_.c_str(), duration.count() / 1000.0);
}
} // namespace onboardDetector