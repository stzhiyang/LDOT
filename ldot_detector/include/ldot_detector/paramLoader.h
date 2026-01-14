/*
    FILE: paramLoader.h
    ---------------------------------
    参数加载器头文件
    将参数读取逻辑从 dynamicDetector 中分离出来
*/
#ifndef ONBOARDDETECTOR_PARAMLOADER_H
#define ONBOARDDETECTOR_PARAMLOADER_H

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace onboardDetector {

// 前向声明
class dynamicDetector;

/*!
 * \brief 参数加载器类
 * 负责从 ROS 参数服务器加载所有配置参数
 */
class ParamLoader {
public:
  /*!
   * \brief 构造函数
   * \param nh ROS 节点句柄
   * \param ns 命名空间
   * \param hint 日志前缀
   */
  ParamLoader(ros::NodeHandle &nh, const std::string &ns,
              const std::string &hint);

  /*!
   * \brief 加载所有参数到 dynamicDetector 实例
   * \param detector 目标检测器实例指针
   */
  void loadAllParams(dynamicDetector *detector);

private:
  ros::NodeHandle &nh_;
  std::string ns_;
  std::string hint_;

  // ==================== 各模块参数加载函数 ====================
  
  // 加载 ROS 话题相关参数
  void loadTopicParams(dynamicDetector *detector);

  // 加载坐标系转换参数（外参）
  void loadTransformParams(dynamicDetector *detector);

  // 加载系统运行参数
  void loadSystemParams(dynamicDetector *detector);

  // 加载点云过滤参数（地面、天花板、检测范围）
  void loadFilterParams(dynamicDetector *detector);

  // 加载 DBSCAN 聚类参数
  void loadDBSCANParams(dynamicDetector *detector);

  // 加载体素下采样参数
  void loadVoxelParams(dynamicDetector *detector);

  // 加载静态点滤波器参数
  void loadStaticFilterParams(dynamicDetector *detector);

  // 加载目标跟踪与数据关联参数
  void loadTrackingParams(dynamicDetector *detector);

  // 加载动态/静态分类参数
  void loadClassificationParams(dynamicDetector *detector);

  // 加载尺寸约束参数
  void loadSizeConstraintParams(dynamicDetector *detector);

  // 加载物体分类阈值参数
  void loadObjectClassifyParams(dynamicDetector *detector);

  // 加载卡尔曼滤波器参数
  void loadKalmanFilterParams(dynamicDetector *detector);

  // 加载轨迹预测参数
  void loadTrajectoryPredictionParams(dynamicDetector *detector);
};

} // namespace onboardDetector

#endif
