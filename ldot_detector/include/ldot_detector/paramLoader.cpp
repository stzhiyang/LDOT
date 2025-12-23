/*
    FILE: paramLoader.cpp
    ---------------------------------
    参数加载器实现文件
    将参数读取逻辑从 dynamicDetector 中分离出来
    日志规范：成功读取用 ROS_INFO_STREAM，失败用 ROS_WARN_STREAM
*/
#include <boost/math/distributions/chi_squared.hpp>
#include <ldot_detector/dynamicDetector.h>
#include <ldot_detector/paramLoader.h>

namespace onboardDetector {

ParamLoader::ParamLoader(ros::NodeHandle &nh, const std::string &ns,
                         const std::string &hint)
    : nh_(nh), ns_(ns), hint_(hint) {}

void ParamLoader::loadAllParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " ========== Loading Parameters ==========");

  loadTopicParams(detector);
  loadTransformParams(detector);
  loadSystemParams(detector);
  loadFilterParams(detector);
  loadDBSCANParams(detector);
  loadVoxelParams(detector);
  loadStaticFilterParams(detector);
  loadTrackingParams(detector);
  loadClassificationParams(detector);
  loadRobustnessParams(detector);
  loadSizeConstraintParams(detector);
  loadNMSParams(detector);
  loadObjectClassifyParams(detector);
  loadKalmanFilterParams(detector);
  loadTrajectoryPredictionParams(detector);

  ROS_INFO_STREAM(hint_ << " ========== Parameters Loaded ==========");
}

// ==================== ROS Topic Parameters ====================
void ParamLoader::loadTopicParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- ROS Topic Parameters ---");

  // 是否使用 Livox CustomMsg 格式
  if (not nh_.getParam(ns_ + "/use_livox_custom_msg",
                       detector->useLivoxCustomMsg_)) {
    detector->useLivoxCustomMsg_ = false;
    ROS_WARN_STREAM(hint_ << " No use_livox_custom_msg param. Use default: "
                             "false (PointCloud2)");
  } else {
    ROS_INFO_STREAM(hint_ << " use_livox_custom_msg: "
                          << (detector->useLivoxCustomMsg_ ? "true" : "false"));
  }

  // 激光雷达点云话题
  if (not nh_.getParam(ns_ + "/lidar_pointcloud_topic",
                       detector->lidarTopicName_)) {
    detector->lidarTopicName_ = "/cloud_registered";
    ROS_WARN_STREAM(hint_ << " No lidar_pointcloud_topic param. Use default: "
                             "/cloud_registered");
  } else {
    ROS_INFO_STREAM(hint_ << " lidar_pointcloud_topic: "
                          << detector->lidarTopicName_);
  }

  // 里程计话题
  if (not nh_.getParam(ns_ + "/odom_topic", detector->odomTopicName_)) {
    detector->odomTopicName_ = "/CERLAB/quadcopter/odom";
    ROS_WARN_STREAM(hint_ << " No odom_topic param. Use default: "
                             "/CERLAB/quadcopter/odom");
  } else {
    ROS_INFO_STREAM(hint_ << " odom_topic: " << detector->odomTopicName_);
  }
}

// ==================== Transform Parameters (Extrinsics) ====================
void ParamLoader::loadTransformParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- Transform Parameters ---");

  std::vector<double> body2LidarVec(16);
  if (not nh_.getParam(ns_ + "/body_to_lidar", body2LidarVec)) {
    ROS_ERROR_STREAM(hint_ << " body_to_lidar matrix not found! Please check "
                              "your config file.");
  } else {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        detector->body2Lidar_(i, j) = body2LidarVec[i * 4 + j];
      }
    }
    ROS_INFO_STREAM(hint_ << " body_to_lidar matrix loaded successfully");
  }
}

// ==================== System Parameters ====================
void ParamLoader::loadSystemParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- System Parameters ---");

  // 时间步长
  if (not nh_.getParam(ns_ + "/time_step", detector->dt_)) {
    detector->dt_ = 0.033;
    ROS_WARN_STREAM(hint_ << " No time_step param. Use default: 0.033s");
  } else {
    ROS_INFO_STREAM(hint_ << " time_step: " << detector->dt_ << "s");
  }

  // 静态地图预热时长
  if (not nh_.getParam(ns_ + "/static_map_warmup_duration", detector->staticMapWarmupDuration_)) {
    detector->staticMapWarmupDuration_ = 3.0;
    ROS_WARN_STREAM(hint_ << " No static_map_warmup_duration param. Use default: 3.0s");
  } else {
    ROS_INFO_STREAM(hint_ << " static_map_warmup_duration: " << detector->staticMapWarmupDuration_ << "s");
  }
  
  // 运动补偿参数
  if (not nh_.getParam(ns_ + "/enable_motion_compensation", detector->enableMotionCompensation_)) {
    detector->enableMotionCompensation_ = false;
    ROS_WARN_STREAM(hint_ << " No enable_motion_compensation param. Use default: false");
  } else {
    ROS_INFO_STREAM(hint_ << " enable_motion_compensation: " 
                          << (detector->enableMotionCompensation_ ? "true" : "false"));
  }
  
  if (not nh_.getParam(ns_ + "/odom_history_size", detector->odomHistorySize_)) {
    detector->odomHistorySize_ = 50;
    ROS_WARN_STREAM(hint_ << " No odom_history_size param. Use default: 50");
  } else {
    ROS_INFO_STREAM(hint_ << " odom_history_size: " << detector->odomHistorySize_);
  }
}

// ==================== Point Cloud Filter Parameters ====================
void ParamLoader::loadFilterParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- Point Cloud Filter Parameters ---");

  // 地面高度阈值
  if (not nh_.getParam(ns_ + "/ground_height", detector->groundHeight_)) {
    detector->groundHeight_ = 0.1;
    ROS_WARN_STREAM(hint_ << " No ground_height param. Use default: 0.1m");
  } else {
    ROS_INFO_STREAM(hint_ << " ground_height: " << detector->groundHeight_
                          << "m");
  }

  // 天花板高度阈值
  if (not nh_.getParam(ns_ + "/roof_height", detector->roofHeight_)) {
    detector->roofHeight_ = 2.0;
    ROS_WARN_STREAM(hint_ << " No roof_height param. Use default: 2.0m");
  } else {
    ROS_INFO_STREAM(hint_ << " roof_height: " << detector->roofHeight_ << "m");
  }

  // 激光雷达检测范围
  std::vector<double> detectionRange;
  if (not nh_.getParam(ns_ + "/lidar_detection_range", detectionRange)) {
    detector->localLidarRange_ = Eigen::Vector3d(10.0, 10.0, 3.0);
    ROS_WARN_STREAM(hint_ << " No lidar_detection_range param. Use default: "
                             "[10.0, 10.0, 3.0]m");
  } else {
    if (detectionRange.size() == 3) {
      detector->localLidarRange_ = Eigen::Vector3d(
          detectionRange[0], detectionRange[1], detectionRange[2]);
      ROS_INFO_STREAM(hint_ << " lidar_detection_range: ["
                            << detector->localLidarRange_.x() << ", "
                            << detector->localLidarRange_.y() << ", "
                            << detector->localLidarRange_.z() << "]m");
    } else {
      detector->localLidarRange_ = Eigen::Vector3d(10.0, 10.0, 3.0);
      ROS_WARN_STREAM(hint_ << " Invalid lidar_detection_range size. Use "
                               "default: [10.0, 10.0, 3.0]m");
    }
  }
}

// ==================== DBSCAN Clustering Parameters ====================
void ParamLoader::loadDBSCANParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- DBSCAN Clustering Parameters ---");

  // 最小点数
  if (not nh_.getParam(ns_ + "/lidar_DBSCAN_min_points",
                       detector->lidarDBMinPoints_)) {
    detector->lidarDBMinPoints_ = 10;
    ROS_WARN_STREAM(hint_ << " No lidar_DBSCAN_min_points param. Use default: "
                             "10");
  } else {
    ROS_INFO_STREAM(hint_ << " lidar_DBSCAN_min_points: "
                          << detector->lidarDBMinPoints_);
  }

  // 搜索半径
  if (not nh_.getParam(ns_ + "/lidar_DBSCAN_epsilon",
                       detector->lidarDBEpsilon_)) {
    detector->lidarDBEpsilon_ = 0.2;
    ROS_WARN_STREAM(hint_ << " No lidar_DBSCAN_epsilon param. Use default: "
                             "0.2m");
  } else {
    ROS_INFO_STREAM(hint_ << " lidar_DBSCAN_epsilon: "
                          << detector->lidarDBEpsilon_ << "m");
  }

  // 是否启用自适应 epsilon
  if (not nh_.getParam(ns_ + "/lidar_DBSCAN_use_adaptive",
                       detector->lidarDBUseAdaptive_)) {
    detector->lidarDBUseAdaptive_ = false;
    ROS_WARN_STREAM(hint_ << " No lidar_DBSCAN_use_adaptive param. Use "
                             "default: false");
  } else {
    ROS_INFO_STREAM(hint_ << " lidar_DBSCAN_use_adaptive: "
                          << (detector->lidarDBUseAdaptive_ ? "true" : "false"));
  }

  // 自适应距离缩放因子
  if (not nh_.getParam(ns_ + "/lidar_DBSCAN_distance_scale",
                       detector->lidarDBDistanceScale_)) {
    detector->lidarDBDistanceScale_ = 0.05;
    ROS_WARN_STREAM(hint_ << " No lidar_DBSCAN_distance_scale param. Use "
                             "default: 0.05");
  } else {
    ROS_INFO_STREAM(hint_ << " lidar_DBSCAN_distance_scale: "
                          << detector->lidarDBDistanceScale_);
  }
}

// ==================== Voxel Downsampling Parameters ====================
void ParamLoader::loadVoxelParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- Voxel Downsampling Parameters ---");

  // 是否启用体素下采样
  if (not nh_.getParam(ns_ + "/enable_voxel_downsampling",
                       detector->enableVoxelDownsampling_)) {
    detector->enableVoxelDownsampling_ = false;
    ROS_WARN_STREAM(hint_ << " No enable_voxel_downsampling param. Use "
                             "default: false");
  } else {
    ROS_INFO_STREAM(hint_ << " enable_voxel_downsampling: "
                          << (detector->enableVoxelDownsampling_ ? "enabled"
                                                                 : "disabled"));
  }

  // 基础体素大小
  if (not nh_.getParam(ns_ + "/voxel_base_leaf_size",
                       detector->voxelBaseLeafSize_)) {
    detector->voxelBaseLeafSize_ = 0.05f;
    ROS_WARN_STREAM(hint_ << " No voxel_base_leaf_size param. Use default: "
                             "0.05m");
  } else {
    ROS_INFO_STREAM(hint_ << " voxel_base_leaf_size: "
                          << detector->voxelBaseLeafSize_ << "m");
  }

  // 目标点云数量
  if (not nh_.getParam(ns_ + "/voxel_target_point_count",
                       detector->voxelTargetPointCount_)) {
    detector->voxelTargetPointCount_ = 30000;
    ROS_WARN_STREAM(hint_ << " No voxel_target_point_count param. Use default: "
                             "30000");
  } else {
    ROS_INFO_STREAM(hint_ << " voxel_target_point_count: "
                          << detector->voxelTargetPointCount_);
  }
}

// ==================== Static Point Filter Parameters ====================
void ParamLoader::loadStaticFilterParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- Static Point Filter Parameters ---");

  // 是否启用静态点滤波
  if (not nh_.getParam(ns_ + "/static_filter_enabled",
                       detector->staticFilterEnabled_)) {
    detector->staticFilterEnabled_ = false;
    ROS_WARN_STREAM(hint_ << " No static_filter_enabled param. Use default: "
                             "false");
  } else {
    ROS_INFO_STREAM(hint_ << " static_filter_enabled: "
                          << (detector->staticFilterEnabled_ ? "true"
                                                             : "false"));
  }

  // 静态滤波器体素大小
  if (not nh_.getParam(ns_ + "/static_filter_voxel_size",
                       detector->staticFilterVoxelSize_)) {
    detector->staticFilterVoxelSize_ = 0.1;
    ROS_WARN_STREAM(hint_ << " No static_filter_voxel_size param. Use default: "
                             "0.1m");
  } else {
    ROS_INFO_STREAM(hint_ << " static_filter_voxel_size: "
                          << detector->staticFilterVoxelSize_ << "m");
  }

  // 静态点命中阈值
  if (not nh_.getParam(ns_ + "/static_filter_hit_threshold",
                       detector->staticFilterHitThreshold_)) {
    detector->staticFilterHitThreshold_ = 5;
    ROS_WARN_STREAM(hint_ << " No static_filter_hit_threshold param. Use "
                             "default: 5");
  } else {
    ROS_INFO_STREAM(hint_ << " static_filter_hit_threshold: "
                          << detector->staticFilterHitThreshold_);
  }

  // 静态点时间阈值
  if (not nh_.getParam(ns_ + "/static_filter_time_threshold",
                       detector->staticFilterTimeThreshold_)) {
    detector->staticFilterTimeThreshold_ = 5.0;
    ROS_WARN_STREAM(hint_ << " No static_filter_time_threshold param. Use "
                             "default: 5.0s");
  } else {
    ROS_INFO_STREAM(hint_ << " static_filter_time_threshold: "
                          << detector->staticFilterTimeThreshold_ << "s");
  }

  // 是否启用静态聚类滤波
  if (not nh_.getParam(ns_ + "/static_cluster_filter_enabled",
                       detector->staticClusterFilterEnabled_)) {
    detector->staticClusterFilterEnabled_ = true;
    ROS_WARN_STREAM(hint_ << " No static_cluster_filter_enabled param. Use "
                             "default: true");
  } else {
    ROS_INFO_STREAM(hint_ << " static_cluster_filter_enabled: "
                          << (detector->staticClusterFilterEnabled_ ? "true"
                                                                    : "false"));
  }

  // 静态聚类比例阈值
  if (not nh_.getParam(ns_ + "/static_cluster_filter_ratio",
                       detector->staticClusterFilterRatio_)) {
    detector->staticClusterFilterRatio_ = 0.7;
    ROS_WARN_STREAM(hint_ << " No static_cluster_filter_ratio param. Use "
                             "default: 0.7");
  } else {
    ROS_INFO_STREAM(hint_ << " static_cluster_filter_ratio: "
                          << detector->staticClusterFilterRatio_);
  }

  // 是否启用邻域投票（用于稀疏点云的静态点判断增强）
  if (not nh_.getParam(ns_ + "/static_filter_use_neighbor_voting",
                       detector->staticFilterUseNeighborVoting_)) {
    detector->staticFilterUseNeighborVoting_ = true;
    ROS_WARN_STREAM(hint_ << " No static_filter_use_neighbor_voting param. Use "
                             "default: true");
  } else {
    ROS_INFO_STREAM(hint_ << " static_filter_use_neighbor_voting: "
                          << (detector->staticFilterUseNeighborVoting_ ? "true"
                                                                       : "false"));
  }

  // 最小邻域投票数
  if (not nh_.getParam(ns_ + "/static_filter_min_neighbor_votes",
                       detector->staticFilterMinNeighborVotes_)) {
    detector->staticFilterMinNeighborVotes_ = 3;
    ROS_WARN_STREAM(hint_ << " No static_filter_min_neighbor_votes param. Use "
                             "default: 3");
  } else {
    ROS_INFO_STREAM(hint_ << " static_filter_min_neighbor_votes: "
                          << detector->staticFilterMinNeighborVotes_);
  }

  // 初始化静态点滤波器
  detector->staticFilter_.reset(new StaticPointFilter());
  detector->staticFilter_->setParams(
      detector->staticFilterEnabled_, detector->staticFilterVoxelSize_,
      detector->staticFilterHitThreshold_, detector->staticFilterTimeThreshold_,
      detector->staticFilterUseNeighborVoting_, detector->staticFilterMinNeighborVotes_);
  if (detector->staticFilterEnabled_ || detector->staticClusterFilterEnabled_) {
    ROS_INFO_STREAM(hint_ << " Static Point Filter initialized (voxel: "
                          << detector->staticFilterVoxelSize_
                          << "m, hits: " << detector->staticFilterHitThreshold_
                          << ", time: " << detector->staticFilterTimeThreshold_ << "s)");
    if (detector->staticClusterFilterEnabled_) {
      ROS_INFO_STREAM(hint_ << " Static Cluster Filter ENABLED (ratio: "
                            << detector->staticClusterFilterRatio_ << ")");
    }
  }
}


// ==================== Tracking and Data Association Parameters ====================
void ParamLoader::loadTrackingParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- Tracking and Data Association Parameters ---");

  // 关联置信度
  if (not nh_.getParam(ns_ + "/association_gate_confidence",
                       detector->associationGateConfidence_)) {
    detector->associationGateConfidence_ = 0.99;
    ROS_WARN_STREAM(hint_ << " No association_gate_confidence param. Use "
                             "default: 0.99");
  } else {
    ROS_INFO_STREAM(hint_ << " association_gate_confidence: "
                          << detector->associationGateConfidence_);
  }

  // 根据置信度计算 3D 门限（卡方分布）
  {
    boost::math::chi_squared_distribution<double> chi2_3d(3);
    detector->gateThreshold3D_ =
        boost::math::quantile(chi2_3d, detector->associationGateConfidence_);
    ROS_INFO_STREAM(hint_ << " gateThreshold3D (computed): "
                          << detector->gateThreshold3D_);
  }

  // 位置代价权重
  if (not nh_.getParam(ns_ + "/association_pos_cost_weight",
                       detector->associationPosCostWeight_)) {
    detector->associationPosCostWeight_ = 1.0;
    ROS_WARN_STREAM(hint_ << " No association_pos_cost_weight param. Use "
                             "default: 1.0");
  } else {
    ROS_INFO_STREAM(hint_ << " association_pos_cost_weight: "
                          << detector->associationPosCostWeight_);
  }

  // IoU 代价权重
  if (not nh_.getParam(ns_ + "/association_iou_cost_weight",
                       detector->associationIoUCostWeight_)) {
    detector->associationIoUCostWeight_ = 0.4;
    ROS_WARN_STREAM(hint_ << " No association_iou_cost_weight param. Use "
                             "default: 0.4");
  } else {
    ROS_INFO_STREAM(hint_ << " association_iou_cost_weight: "
                          << detector->associationIoUCostWeight_);
  }

  // 跟踪历史长度
  if (not nh_.getParam(ns_ + "/history_size", detector->histSize_)) {
    detector->histSize_ = 5;
    ROS_WARN_STREAM(hint_ << " No history_size param. Use default: 5");
  } else {
    ROS_INFO_STREAM(hint_ << " history_size: " << detector->histSize_);
  }

  // 最大丢失帧数
  if (not nh_.getParam(ns_ + "/max_missed_frames", detector->maxMissedFrames_)) {
    detector->maxMissedFrames_ = 5;
    ROS_WARN_STREAM(hint_ << " No max_missed_frames param. Use default: 5");
  } else {
    ROS_INFO_STREAM(hint_ << " max_missed_frames: "
                          << detector->maxMissedFrames_);
  }

  // 重复轨迹 IoU 阈值
  if (not nh_.getParam(ns_ + "/duplicate_track_iou_threshold",
                       detector->duplicateTrackIoUThreshold_)) {
    detector->duplicateTrackIoUThreshold_ = 0.5;
    ROS_WARN_STREAM(hint_ << " No duplicate_track_iou_threshold param. Use "
                             "default: 0.5");
  } else {
    ROS_INFO_STREAM(hint_ << " duplicate_track_iou_threshold: "
                          << detector->duplicateTrackIoUThreshold_);
  }

  // 重复轨迹距离阈值
  if (not nh_.getParam(ns_ + "/duplicate_track_distance_threshold",
                       detector->duplicateTrackDistanceThreshold_)) {
    detector->duplicateTrackDistanceThreshold_ = 2.0;
    ROS_WARN_STREAM(hint_ << " No duplicate_track_distance_threshold param. "
                             "Use default: 2.0m");
  } else {
    ROS_INFO_STREAM(hint_ << " duplicate_track_distance_threshold: "
                          << detector->duplicateTrackDistanceThreshold_ << "m");
  }

  // 重复轨迹速度相似度阈值
  if (not nh_.getParam(ns_ + "/duplicate_track_velocity_similarity_threshold",
                       detector->duplicateTrackVelocitySimilarityThreshold_)) {
    detector->duplicateTrackVelocitySimilarityThreshold_ = 0.7;
    ROS_WARN_STREAM(hint_ << " No duplicate_track_velocity_similarity_threshold "
                             "param. Use default: 0.7");
  } else {
    ROS_INFO_STREAM(hint_ << " duplicate_track_velocity_similarity_threshold: "
                          << detector->duplicateTrackVelocitySimilarityThreshold_);
  }

  // coasting 轨迹门限放松因子
  if (not nh_.getParam(ns_ + "/coasting_track_gate_relax_factor",
                       detector->coastingTrackGateRelaxFactor_)) {
    detector->coastingTrackGateRelaxFactor_ = 2.0;
    ROS_WARN_STREAM(hint_ << " No coasting_track_gate_relax_factor param. Use "
                             "default: 2.0");
  } else {
    ROS_INFO_STREAM(hint_ << " coasting_track_gate_relax_factor: "
                          << detector->coastingTrackGateRelaxFactor_);
  }

  // 包围框尺寸平滑系数
  if (not nh_.getParam(ns_ + "/box_size_smoothing_alpha",
                       detector->boxSizeSmoothingAlpha_)) {
    detector->boxSizeSmoothingAlpha_ = 0.3;
    ROS_WARN_STREAM(hint_ << " No box_size_smoothing_alpha param. Use default: "
                             "0.3");
  } else {
    ROS_INFO_STREAM(hint_ << " box_size_smoothing_alpha: "
                          << detector->boxSizeSmoothingAlpha_);
  }

  // 尺寸保持比例阈值
  if (not nh_.getParam(ns_ + "/size_retain_ratio", detector->sizeRetainRatio_)) {
    detector->sizeRetainRatio_ = 0.7;
    ROS_WARN_STREAM(hint_ << " No size_retain_ratio param. Use default: 0.7");
  } else {
    ROS_INFO_STREAM(hint_ << " size_retain_ratio: "
                          << detector->sizeRetainRatio_);
  }
}

// ==================== Dynamic/Static Classification Parameters ====================
void ParamLoader::loadClassificationParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- Dynamic/Static Classification Parameters ---");

  // 分类检查时间间隔
  if (not nh_.getParam(ns_ + "/classification_interval_sec",
                       detector->classificationIntervalSec_)) {
    detector->classificationIntervalSec_ = 3.0;
    ROS_WARN_STREAM(hint_ << " No classification_interval_sec param. Use "
                             "default: 3.0s");
  } else {
    ROS_INFO_STREAM(hint_ << " classification_interval_sec: "
                          << detector->classificationIntervalSec_ << "s");
  }

  // 帧跳过数
  if (not nh_.getParam(ns_ + "/frame_skip", detector->skipFrame_)) {
    detector->skipFrame_ = 5;
    ROS_WARN_STREAM(hint_ << " No frame_skip param. Use default: 5");
  } else {
    ROS_INFO_STREAM(hint_ << " frame_skip: " << detector->skipFrame_);
  }

  // 动态速度阈值
  if (not nh_.getParam(ns_ + "/dynamic_velocity_threshold",
                       detector->dynaVelThresh_)) {
    detector->dynaVelThresh_ = 0.35;
    ROS_WARN_STREAM(hint_ << " No dynamic_velocity_threshold param. Use "
                             "default: 0.35 m/s");
  } else {
    ROS_INFO_STREAM(hint_ << " dynamic_velocity_threshold: "
                          << detector->dynaVelThresh_ << " m/s");
  }

  // 动态投票阈值
  if (not nh_.getParam(ns_ + "/dynamic_voting_threshold",
                       detector->dynaVoteThresh_)) {
    detector->dynaVoteThresh_ = 0.8;
    ROS_WARN_STREAM(hint_ << " No dynamic_voting_threshold param. Use default: "
                             "0.8");
  } else {
    ROS_INFO_STREAM(hint_ << " dynamic_voting_threshold: "
                          << detector->dynaVoteThresh_);
  }

  // 强制动态帧数
  if (not nh_.getParam(ns_ + "/frames_force_dynamic",
                       detector->forceDynaFrames_)) {
    detector->forceDynaFrames_ = 20;
    ROS_WARN_STREAM(hint_ << " No frames_force_dynamic param. Use default: 20");
  } else {
    ROS_INFO_STREAM(hint_ << " frames_force_dynamic: "
                          << detector->forceDynaFrames_);
  }

  // 强制动态检查范围
  if (not nh_.getParam(ns_ + "/frames_force_dynamic_check_range",
                       detector->forceDynaCheckRange_)) {
    detector->forceDynaCheckRange_ = 30;
    ROS_WARN_STREAM(hint_ << " No frames_force_dynamic_check_range param. Use "
                             "default: 30");
  } else {
    ROS_INFO_STREAM(hint_ << " frames_force_dynamic_check_range: "
                          << detector->forceDynaCheckRange_);
  }

  // 动态一致性阈值
  if (not nh_.getParam(ns_ + "/dynamic_consistency_threshold",
                       detector->dynamicConsistThresh_)) {
    detector->dynamicConsistThresh_ = 3;
    ROS_WARN_STREAM(hint_ << " No dynamic_consistency_threshold param. Use "
                             "default: 3");
  } else {
    ROS_INFO_STREAM(hint_ << " dynamic_consistency_threshold: "
                          << detector->dynamicConsistThresh_);
  }

  // 检查历史长度是否足够
  if (detector->histSize_ < detector->forceDynaCheckRange_ + 1) {
    ROS_ERROR_STREAM(hint_ << " history_size is too short to perform "
                              "force-dynamic check!");
  }

  // 点云匹配距离
  if (not nh_.getParam(ns_ + "/classification_min_neighbor_distance",
                       detector->classificationMinNeighborDist_)) {
    detector->classificationMinNeighborDist_ = 2.0;
    ROS_WARN_STREAM(hint_ << " No classification_min_neighbor_distance param. "
                             "Use default: 2.0m");
  } else {
    ROS_INFO_STREAM(hint_ << " classification_min_neighbor_distance: "
                          << detector->classificationMinNeighborDist_ << "m");
  }

  // 尺寸合并阈值
  if (not nh_.getParam(ns_ + "/classification_size_merge_threshold",
                       detector->sizeMergeThresh_)) {
    detector->sizeMergeThresh_ = 1.5;
    ROS_WARN_STREAM(hint_ << " No classification_size_merge_threshold param. "
                             "Use default: 1.5");
  } else {
    ROS_INFO_STREAM(hint_ << " classification_size_merge_threshold: "
                          << detector->sizeMergeThresh_);
  }

  // 点数合并阈值
  if (not nh_.getParam(ns_ + "/classification_point_count_merge_threshold",
                       detector->pointCountMergeThresh_)) {
    detector->pointCountMergeThresh_ = 1.5;
    ROS_WARN_STREAM(hint_ << " No classification_point_count_merge_threshold "
                             "param. Use default: 1.5");
  } else {
    ROS_INFO_STREAM(hint_ << " classification_point_count_merge_threshold: "
                          << detector->pointCountMergeThresh_);
  }

  // 尺寸重置帧数
  if (not nh_.getParam(ns_ + "/classification_size_reset_frames",
                       detector->sizeResetFrames_)) {
    detector->sizeResetFrames_ = 30;
    ROS_WARN_STREAM(hint_ << " No classification_size_reset_frames param. Use "
                             "default: 30");
  } else {
    ROS_INFO_STREAM(hint_ << " classification_size_reset_frames: "
                          << detector->sizeResetFrames_);
  }

  // 体素清除动态帧数
  if (not nh_.getParam(ns_ + "/voxel_clear_dynamic_frames",
                       detector->voxelClearDynamicFrames_)) {
    detector->voxelClearDynamicFrames_ = 5;
    ROS_WARN_STREAM(hint_ << " No voxel_clear_dynamic_frames param. Use "
                             "default: 5");
  } else {
    ROS_INFO_STREAM(hint_ << " voxel_clear_dynamic_frames: "
                          << detector->voxelClearDynamicFrames_);
  }
}


// ==================== Robustness Enhancement Parameters ====================
void ParamLoader::loadRobustnessParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- Robustness Enhancement Parameters ---");

  // 最小可靠点数
  if (not nh_.getParam(ns_ + "/min_reliable_points",
                       detector->minReliablePoints_)) {
    detector->minReliablePoints_ = 20;
    ROS_WARN_STREAM(hint_ << " No min_reliable_points param. Use default: 20");
  } else {
    ROS_INFO_STREAM(hint_ << " min_reliable_points: "
                          << detector->minReliablePoints_);
  }

  // 点数下降阈值
  if (not nh_.getParam(ns_ + "/point_count_drop_threshold",
                       detector->pointCountDropThreshold_)) {
    detector->pointCountDropThreshold_ = 0.5;
    ROS_WARN_STREAM(hint_ << " No point_count_drop_threshold param. Use "
                             "default: 0.5");
  } else {
    ROS_INFO_STREAM(hint_ << " point_count_drop_threshold: "
                          << detector->pointCountDropThreshold_);
  }

  // 滞后系数
  if (not nh_.getParam(ns_ + "/hysteresis_lower", detector->hysteresisLower_)) {
    detector->hysteresisLower_ = 0.7;
    ROS_WARN_STREAM(hint_ << " No hysteresis_lower param. Use default: 0.7");
  } else {
    ROS_INFO_STREAM(hint_ << " hysteresis_lower: "
                          << detector->hysteresisLower_);
  }
}

// ==================== Size Constraint Parameters ====================
void ParamLoader::loadSizeConstraintParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- Size Constraint Parameters ---");

  // 最大物体尺寸
  std::vector<double> maxObjectSizeTemp;
  if (not nh_.getParam(ns_ + "/max_object_size", maxObjectSizeTemp)) {
    detector->maxObjectSize_ = Eigen::Vector3d(2.0, 2.0, 2.0);
    ROS_WARN_STREAM(hint_ << " No max_object_size param. Use default: [2.0, "
                             "2.0, 2.0]m");
  } else {
    if (maxObjectSizeTemp.size() >= 3) {
      detector->maxObjectSize_(0) = maxObjectSizeTemp[0];
      detector->maxObjectSize_(1) = maxObjectSizeTemp[1];
      detector->maxObjectSize_(2) = maxObjectSizeTemp[2];
      ROS_INFO_STREAM(hint_ << " max_object_size: ["
                            << detector->maxObjectSize_(0) << ", "
                            << detector->maxObjectSize_(1) << ", "
                            << detector->maxObjectSize_(2) << "]m");
    } else {
      detector->maxObjectSize_ = Eigen::Vector3d(2.0, 2.0, 2.0);
      ROS_WARN_STREAM(hint_ << " Invalid max_object_size size. Use default: "
                               "[2.0, 2.0, 2.0]m");
    }
  }
}

// ==================== NMS (Non-Maximum Suppression) Parameters ====================
void ParamLoader::loadNMSParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- NMS Parameters ---");

  // 是否启用检测 NMS
  if (not nh_.getParam(ns_ + "/enable_detection_nms",
                       detector->enableDetectionNMS_)) {
    detector->enableDetectionNMS_ = true;
    ROS_WARN_STREAM(hint_ << " No enable_detection_nms param. Use default: "
                             "true");
  } else {
    ROS_INFO_STREAM(hint_ << " enable_detection_nms: "
                          << (detector->enableDetectionNMS_ ? "enabled"
                                                            : "disabled"));
  }

  // NMS IoU 阈值
  if (not nh_.getParam(ns_ + "/detection_nms_iou_threshold",
                       detector->detectionNMSIoUThreshold_)) {
    detector->detectionNMSIoUThreshold_ = 0.3;
    ROS_WARN_STREAM(hint_ << " No detection_nms_iou_threshold param. Use "
                             "default: 0.3");
  } else {
    ROS_INFO_STREAM(hint_ << " detection_nms_iou_threshold: "
                          << detector->detectionNMSIoUThreshold_);
  }

  // NMS 距离缩放参数
  if (not nh_.getParam(ns_ + "/detection_nms_dist_scale",
                       detector->detectionNMSDistScale_)) {
    detector->detectionNMSDistScale_ = 2.0;
    ROS_WARN_STREAM(hint_ << " No detection_nms_dist_scale param. Use default: "
                             "2.0");
  } else {
    ROS_INFO_STREAM(hint_ << " detection_nms_dist_scale: "
                          << detector->detectionNMSDistScale_);
  }
}

// ==================== Object Classification Threshold Parameters ====================
void ParamLoader::loadObjectClassifyParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- Object Classification Threshold Parameters ---");

  // 人的分类阈值
  std::vector<double> classifyHumanThresh;
  if (not nh_.getParam(ns_ + "/classify_human_threshold", classifyHumanThresh)) {
    detector->classifyHumanZWidthRatio_ = 2.0;
    detector->classifyHumanCentroidZRatio_ = 0.5;
    ROS_WARN_STREAM(hint_ << " No classify_human_threshold param. Use default: "
                             "[2.0, 0.5]");
  } else {
    detector->classifyHumanZWidthRatio_ = classifyHumanThresh[0];
    detector->classifyHumanCentroidZRatio_ = classifyHumanThresh[1];
    ROS_INFO_STREAM(hint_ << " classify_human_threshold: ["
                          << detector->classifyHumanZWidthRatio_ << ", "
                          << detector->classifyHumanCentroidZRatio_ << "]");
  }

  // 车的分类阈值
  std::vector<double> classifyVehicleThresh;
  if (not nh_.getParam(ns_ + "/classify_vehicle_threshold",
                       classifyVehicleThresh)) {
    detector->classifyVehicleXYWidthRatio_ = 1.5;
    detector->classifyVehicleCentroidZRatio_ = 0.8;
    ROS_WARN_STREAM(hint_ << " No classify_vehicle_threshold param. Use "
                             "default: [1.5, 0.8]");
  } else {
    detector->classifyVehicleXYWidthRatio_ = classifyVehicleThresh[0];
    detector->classifyVehicleCentroidZRatio_ = classifyVehicleThresh[1];
    ROS_INFO_STREAM(hint_ << " classify_vehicle_threshold: ["
                          << detector->classifyVehicleXYWidthRatio_ << ", "
                          << detector->classifyVehicleCentroidZRatio_ << "]");
  }

  // 无人机的分类阈值
  std::vector<double> classifyUAVThresh;
  if (not nh_.getParam(ns_ + "/classify_uav_threshold", classifyUAVThresh)) {
    detector->classifyUAVMaxSize_ = 0.6;
    detector->classifyUAVCentroidZRatio_ = 1.2;
    ROS_WARN_STREAM(hint_ << " No classify_uav_threshold param. Use default: "
                             "[0.6, 1.2]");
  } else {
    detector->classifyUAVMaxSize_ = classifyUAVThresh[0];
    detector->classifyUAVCentroidZRatio_ = classifyUAVThresh[1];
    ROS_INFO_STREAM(hint_ << " classify_uav_threshold: ["
                          << detector->classifyUAVMaxSize_ << ", "
                          << detector->classifyUAVCentroidZRatio_ << "]");
  }

  // box 与无人机 xy 轴距离阈值
  if (not nh_.getParam(ns_ + "/classify_xy_distance_threshold",
                       detector->classifyXYDistanceThreshold_)) {
    detector->classifyXYDistanceThreshold_ = 3.0;
    ROS_WARN_STREAM(hint_ << " No classify_xy_distance_threshold param. Use "
                             "default: 3.0m");
  } else {
    ROS_INFO_STREAM(hint_ << " classify_xy_distance_threshold: "
                          << detector->classifyXYDistanceThreshold_ << "m");
  }

  // 分类开始帧数
  if (not nh_.getParam(ns_ + "/classification_start_frame",
                       detector->classificationStartFrame_)) {
    detector->classificationStartFrame_ = 10;
    ROS_WARN_STREAM(hint_ << " No classification_start_frame param. Use "
                             "default: 10");
  } else {
    ROS_INFO_STREAM(hint_ << " classification_start_frame: "
                          << detector->classificationStartFrame_);
  }

  // 固定尺寸分类阈值
  if (not nh_.getParam(ns_ + "/fix_size_classification_threshold",
                       detector->fixSizeClassificationThreshold_)) {
    detector->fixSizeClassificationThreshold_ = 3;
    ROS_WARN_STREAM(hint_ << " No fix_size_classification_threshold param. Use "
                             "default: 3");
  } else {
    ROS_INFO_STREAM(hint_ << " fix_size_classification_threshold: "
                          << detector->fixSizeClassificationThreshold_);
  }
}

// ==================== Kalman Filter Parameters ====================
void ParamLoader::loadKalmanFilterParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- Kalman Filter Parameters ---");

  // 自适应窗口大小
  if (not nh_.getParam(ns_ + "/kalman_filter/adaptive_window_size",
                       detector->kfParams_.adaptive_window_size)) {
    detector->kfParams_.adaptive_window_size = 5;
    ROS_WARN_STREAM(hint_ << " No kalman_filter/adaptive_window_size param. "
                             "Use default: 5");
  } else {
    ROS_INFO_STREAM(hint_ << " kalman_filter/adaptive_window_size: "
                          << detector->kfParams_.adaptive_window_size);
  }

  // 自适应 alpha
  if (not nh_.getParam(ns_ + "/kalman_filter/adaptive_alpha",
                       detector->kfParams_.adaptive_alpha)) {
    detector->kfParams_.adaptive_alpha = 0.3;
    ROS_WARN_STREAM(hint_ << " No kalman_filter/adaptive_alpha param. Use "
                             "default: 0.3");
  } else {
    ROS_INFO_STREAM(hint_ << " kalman_filter/adaptive_alpha: "
                          << detector->kfParams_.adaptive_alpha);
  }

  // 自适应 R alpha
  if (not nh_.getParam(ns_ + "/kalman_filter/adaptive_r_alpha",
                       detector->kfParams_.adaptive_r_alpha)) {
    detector->kfParams_.adaptive_r_alpha = 0.3;
    ROS_WARN_STREAM(hint_ << " No kalman_filter/adaptive_r_alpha param. Use "
                             "default: 0.3");
  } else {
    ROS_INFO_STREAM(hint_ << " kalman_filter/adaptive_r_alpha: "
                          << detector->kfParams_.adaptive_r_alpha);
  }

  // 自适应最小噪声比
  if (not nh_.getParam(ns_ + "/kalman_filter/adaptive_min_noise_ratio",
                       detector->kfParams_.adaptive_min_noise_ratio)) {
    detector->kfParams_.adaptive_min_noise_ratio = 0.5;
    ROS_WARN_STREAM(hint_ << " No kalman_filter/adaptive_min_noise_ratio "
                             "param. Use default: 0.5");
  } else {
    ROS_INFO_STREAM(hint_ << " kalman_filter/adaptive_min_noise_ratio: "
                          << detector->kfParams_.adaptive_min_noise_ratio);
  }

  // 协方差上限参数
  if (not nh_.getParam(ns_ + "/kalman_filter/enable_cov_limit",
                       detector->kfParams_.enable_cov_limit)) {
    detector->kfParams_.enable_cov_limit = false;
    ROS_WARN_STREAM(hint_ << " No kalman_filter/enable_cov_limit param. Use "
                             "default: false");
  } else {
    ROS_INFO_STREAM(hint_ << " kalman_filter/enable_cov_limit: "
                          << (detector->kfParams_.enable_cov_limit ? "enabled"
                                                                   : "disabled"));
  }

  // 最大位置协方差
  if (not nh_.getParam(ns_ + "/kalman_filter/max_pos_cov",
                       detector->kfParams_.max_pos_cov)) {
    detector->kfParams_.max_pos_cov = 9.0;
    ROS_WARN_STREAM(hint_ << " No kalman_filter/max_pos_cov param. Use "
                             "default: 9.0");
  } else {
    ROS_INFO_STREAM(hint_ << " kalman_filter/max_pos_cov: "
                          << detector->kfParams_.max_pos_cov);
  }

  // 最大速度协方差
  if (not nh_.getParam(ns_ + "/kalman_filter/max_vel_cov",
                       detector->kfParams_.max_vel_cov)) {
    detector->kfParams_.max_vel_cov = 16.0;
    ROS_WARN_STREAM(hint_ << " No kalman_filter/max_vel_cov param. Use "
                             "default: 16.0");
  } else {
    ROS_INFO_STREAM(hint_ << " kalman_filter/max_vel_cov: "
                          << detector->kfParams_.max_vel_cov);
  }

  // 最大加速度协方差
  if (not nh_.getParam(ns_ + "/kalman_filter/max_acc_cov",
                       detector->kfParams_.max_acc_cov)) {
    detector->kfParams_.max_acc_cov = 100.0;
    ROS_WARN_STREAM(hint_ << " No kalman_filter/max_acc_cov param. Use "
                             "default: 100.0");
  } else {
    ROS_INFO_STREAM(hint_ << " kalman_filter/max_acc_cov: "
                          << detector->kfParams_.max_acc_cov);
  }

  // 预测协方差乘数
  if (not nh_.getParam(ns_ + "/kalman_filter/prediction_cov_multiplier",
                       detector->kfParams_.prediction_cov_multiplier)) {
    detector->kfParams_.prediction_cov_multiplier = 5.0;
    ROS_WARN_STREAM(hint_ << " No kalman_filter/prediction_cov_multiplier "
                             "param. Use default: 5.0");
  } else {
    ROS_INFO_STREAM(hint_ << " kalman_filter/prediction_cov_multiplier: "
                          << detector->kfParams_.prediction_cov_multiplier);
  }

  // 输出协方差限制汇总
  if (detector->kfParams_.enable_cov_limit) {
    ROS_INFO_STREAM(hint_ << " Cov limits: pos=" << detector->kfParams_.max_pos_cov
                          << " vel=" << detector->kfParams_.max_vel_cov
                          << " acc=" << detector->kfParams_.max_acc_cov
                          << " pred_mult=" << detector->kfParams_.prediction_cov_multiplier);
  }

  // -------------------- CA Model (Human) --------------------
  ROS_INFO_STREAM(hint_ << " --- CA Model (Human) ---");

  if (not nh_.getParam(ns_ + "/kalman_filter/ca_model/human/jerk_sigma",
                       detector->kfParams_.ca_human.jerk_sigma)) {
    detector->kfParams_.ca_human.jerk_sigma = 1.0;
    ROS_WARN_STREAM(hint_ << " No ca_model/human/jerk_sigma param. Use "
                             "default: 1.0");
  } else {
    ROS_INFO_STREAM(hint_ << " ca_model/human/jerk_sigma: "
                          << detector->kfParams_.ca_human.jerk_sigma);
  }

  if (not nh_.getParam(ns_ + "/kalman_filter/ca_model/human/init_cov",
                       detector->kfParams_.ca_human.init_cov)) {
    detector->kfParams_.ca_human.init_cov = {0.1, 0.1, 0.1, 1.0, 1.0, 10.0, 10.0};
    ROS_WARN_STREAM(hint_ << " No ca_model/human/init_cov param. Use default");
  } else {
    ROS_INFO_STREAM(hint_ << " ca_model/human/init_cov loaded");
  }

  if (not nh_.getParam(ns_ + "/kalman_filter/ca_model/human/meas_noise",
                       detector->kfParams_.ca_human.meas_noise)) {
    detector->kfParams_.ca_human.meas_noise = {0.1, 0.1, 0.1};
    ROS_WARN_STREAM(hint_ << " No ca_model/human/meas_noise param. Use default");
  } else {
    ROS_INFO_STREAM(hint_ << " ca_model/human/meas_noise loaded");
  }

  if (not nh_.getParam(ns_ + "/kalman_filter/ca_model/human/z_process_noise",
                       detector->kfParams_.ca_human.z_process_noise)) {
    detector->kfParams_.ca_human.z_process_noise = 0.01;
    ROS_WARN_STREAM(hint_ << " No ca_model/human/z_process_noise param. Use "
                             "default: 0.01");
  } else {
    ROS_INFO_STREAM(hint_ << " ca_model/human/z_process_noise: "
                          << detector->kfParams_.ca_human.z_process_noise);
  }

  // -------------------- CA Model (UAV) --------------------
  ROS_INFO_STREAM(hint_ << " --- CA Model (UAV) ---");

  if (not nh_.getParam(ns_ + "/kalman_filter/ca_model/uav/jerk_sigma",
                       detector->kfParams_.ca_uav.jerk_sigma)) {
    detector->kfParams_.ca_uav.jerk_sigma = 1.0;
    ROS_WARN_STREAM(hint_ << " No ca_model/uav/jerk_sigma param. Use default: "
                             "1.0");
  } else {
    ROS_INFO_STREAM(hint_ << " ca_model/uav/jerk_sigma: "
                          << detector->kfParams_.ca_uav.jerk_sigma);
  }

  if (not nh_.getParam(ns_ + "/kalman_filter/ca_model/uav/init_cov",
                       detector->kfParams_.ca_uav.init_cov)) {
    detector->kfParams_.ca_uav.init_cov = {0.1, 0.1, 0.1, 1.0, 1.0, 1.0, 10.0, 10.0, 10.0};
    ROS_WARN_STREAM(hint_ << " No ca_model/uav/init_cov param. Use default");
  } else {
    ROS_INFO_STREAM(hint_ << " ca_model/uav/init_cov loaded");
  }

  if (not nh_.getParam(ns_ + "/kalman_filter/ca_model/uav/meas_noise",
                       detector->kfParams_.ca_uav.meas_noise)) {
    detector->kfParams_.ca_uav.meas_noise = {0.1, 0.1, 0.1};
    ROS_WARN_STREAM(hint_ << " No ca_model/uav/meas_noise param. Use default");
  } else {
    ROS_INFO_STREAM(hint_ << " ca_model/uav/meas_noise loaded");
  }

  // -------------------- CV Model --------------------
  ROS_INFO_STREAM(hint_ << " --- CV Model ---");

  if (not nh_.getParam(ns_ + "/kalman_filter/cv_model/acc_sigma",
                       detector->kfParams_.cv.acc_sigma)) {
    detector->kfParams_.cv.acc_sigma = 3.0;
    ROS_WARN_STREAM(hint_ << " No cv_model/acc_sigma param. Use default: 3.0");
  } else {
    ROS_INFO_STREAM(hint_ << " cv_model/acc_sigma: "
                          << detector->kfParams_.cv.acc_sigma);
  }

  if (not nh_.getParam(ns_ + "/kalman_filter/cv_model/init_cov",
                       detector->kfParams_.cv.init_cov)) {
    detector->kfParams_.cv.init_cov = {0.5, 0.5, 0.5, 2.0, 2.0, 2.0};
    ROS_WARN_STREAM(hint_ << " No cv_model/init_cov param. Use default");
  } else {
    ROS_INFO_STREAM(hint_ << " cv_model/init_cov loaded");
  }

  if (not nh_.getParam(ns_ + "/kalman_filter/cv_model/meas_noise",
                       detector->kfParams_.cv.meas_noise)) {
    detector->kfParams_.cv.meas_noise = {0.3, 0.3, 0.3};
    ROS_WARN_STREAM(hint_ << " No cv_model/meas_noise param. Use default");
  } else {
    ROS_INFO_STREAM(hint_ << " cv_model/meas_noise loaded");
  }

  // -------------------- CTRA Model --------------------
  ROS_INFO_STREAM(hint_ << " --- CTRA Model ---");

  if (not nh_.getParam(ns_ + "/kalman_filter/ctra_model/init_cov",
                       detector->kfParams_.ctra.init_cov)) {
    detector->kfParams_.ctra.init_cov = {0.1, 0.1, 0.1, 1.0, 10.0, 0.5, 1.0};
    ROS_WARN_STREAM(hint_ << " No ctra_model/init_cov param. Use default");
  } else {
    ROS_INFO_STREAM(hint_ << " ctra_model/init_cov loaded");
  }

  if (not nh_.getParam(ns_ + "/kalman_filter/ctra_model/process_noise",
                       detector->kfParams_.ctra.process_noise)) {
    detector->kfParams_.ctra.process_noise = {0.1, 0.1, 0.01, 1.0, 10.0, 0.1, 1.0};
    ROS_WARN_STREAM(hint_ << " No ctra_model/process_noise param. Use default");
  } else {
    ROS_INFO_STREAM(hint_ << " ctra_model/process_noise loaded");
  }

  if (not nh_.getParam(ns_ + "/kalman_filter/ctra_model/meas_noise",
                       detector->kfParams_.ctra.meas_noise)) {
    detector->kfParams_.ctra.meas_noise = {0.1, 0.1, 0.1};
    ROS_WARN_STREAM(hint_ << " No ctra_model/meas_noise param. Use default");
  } else {
    ROS_INFO_STREAM(hint_ << " ctra_model/meas_noise loaded");
  }
}

// ==================== Trajectory Prediction Parameters ====================
void ParamLoader::loadTrajectoryPredictionParams(dynamicDetector *detector) {
  ROS_INFO_STREAM(hint_ << " --- Trajectory Prediction Parameters ---");

  // 默认预测时域（秒）
  if (not nh_.getParam(ns_ + "/trajectory_prediction/default_horizon",
                       detector->trajPredDefaultHorizon_)) {
    detector->trajPredDefaultHorizon_ = 3.0;
    ROS_WARN_STREAM(hint_ << " No trajectory_prediction/default_horizon param. "
                             "Use default: 3.0s");
  } else {
    ROS_INFO_STREAM(hint_ << " trajectory_prediction/default_horizon: "
                          << detector->trajPredDefaultHorizon_ << "s");
  }

  // 默认预测步长（秒）
  if (not nh_.getParam(ns_ + "/trajectory_prediction/default_dt",
                       detector->trajPredDefaultDt_)) {
    detector->trajPredDefaultDt_ = 0.2;
    ROS_WARN_STREAM(hint_ << " No trajectory_prediction/default_dt param. Use "
                             "default: 0.2s");
  } else {
    ROS_INFO_STREAM(hint_ << " trajectory_prediction/default_dt: "
                          << detector->trajPredDefaultDt_ << "s");
  }

  // 碰撞检测膨胀系数（米）
  if (not nh_.getParam(ns_ + "/trajectory_prediction/collision_inflation",
                       detector->trajPredCollisionInflation_)) {
    detector->trajPredCollisionInflation_ = 0.1;
    ROS_WARN_STREAM(hint_ << " No trajectory_prediction/collision_inflation "
                             "param. Use default: 0.1m");
  } else {
    ROS_INFO_STREAM(hint_ << " trajectory_prediction/collision_inflation: "
                          << detector->trajPredCollisionInflation_ << "m");
  }

  // 最大轨迹点数限制
  if (not nh_.getParam(ns_ + "/trajectory_prediction/max_trajectory_points",
                       detector->trajPredMaxPoints_)) {
    detector->trajPredMaxPoints_ = 30;
    ROS_WARN_STREAM(hint_ << " No trajectory_prediction/max_trajectory_points "
                             "param. Use default: 30");
  } else {
    ROS_INFO_STREAM(hint_ << " trajectory_prediction/max_trajectory_points: "
                          << detector->trajPredMaxPoints_);
  }

  // 初始化激光雷达检测器（避免每次回调时重复初始化）
  detector->lidarDetector_.reset(new lidarDetector());
  detector->lidarDetector_->setParams(
      detector->lidarDBEpsilon_, detector->lidarDBMinPoints_,
      detector->lidarDBUseAdaptive_, detector->lidarDBDistanceScale_);
  ROS_INFO_STREAM(hint_ << " Lidar detector initialized");
}

} // namespace onboardDetector
