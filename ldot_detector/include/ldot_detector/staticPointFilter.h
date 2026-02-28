/*
    FILE: staticPointFilter.h
    ---------------------------------
    静态点滤波器头文件
*/
#ifndef ONBOARDDETECTOR_STATICPOINTFILTER_H
#define ONBOARDDETECTOR_STATICPOINTFILTER_H

#include <Eigen/Dense>
#include <ldot_detector/lidarDetector.h>
#include <ldot_detector/utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <unordered_map>

namespace onboardDetector {

// 体素状态结构体
struct VoxelStatus {
  int hit_count;
  double last_seen_time;

  VoxelStatus() : hit_count(0), last_seen_time(0.0) {}
};

class StaticPointFilter {
public:
  StaticPointFilter();
  ~StaticPointFilter();

  // 设置参数
  void setParams(bool enabled, float voxel_size, int hit_threshold,
                 double time_threshold);

  // 仅更新地图
  // sensor_position: 传感器在全局坐标系中的位置（用于近距离累积抑制）
  void updateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                 double current_time,
                 const Eigen::Vector3d &sensor_position);

  // 点级过滤 (原 filter 函数)
  void filterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                    const std::vector<onboardDetector::box3D> &protected_boxes =
                        std::vector<onboardDetector::box3D>());

  // 清理旧体素
  void cleanMap(double current_time);

  // 检查带尺寸的包围框是否与静态地图碰撞
  // center: 包围框中心位置
  // size: 包围框尺寸 (x_width, y_width, z_width)
  // inflation: 膨胀系数（米），用于安全裕度
  bool checkBoxCollision(const Eigen::Vector3d &center,
                         const Eigen::Vector3d &size, double inflation = 0.0);

private:
  // 计算体素键值的辅助函数
  long long getVoxelKey(const pcl::PointXYZ &point);

  // 检查点是否在边界框内
  bool isPointInBox(const pcl::PointXYZ &pt, const onboardDetector::box3D &box);

  // 检查点是否为静态 (基于当前地图)
  bool isPointStatic(const pcl::PointXYZ &pt);

  // 距离自适应阈值 - 远距离降低判定门槛
  // 使用成员变量 sensor_position_ 计算距离
  int getAdaptiveThreshold(const pcl::PointXYZ &pt);

  bool enabled_;
  float voxel_size_;
  int hit_threshold_;
  double time_threshold_;

  // 帧计数器，用于控制清理频率
  int frame_count_;

  // 体素地图：键值 -> 状态
  std::unordered_map<long long, VoxelStatus> voxel_map_;

  // 【新增】传感器位置（全局坐标系），用于距离计算
  // 在 updateMap 时更新，供 isPointStatic 等函数使用
  Eigen::Vector3d sensor_position_;
};

} // namespace onboardDetector

#endif // ONBOARDDETECTOR_STATICPOINTFILTER_H
