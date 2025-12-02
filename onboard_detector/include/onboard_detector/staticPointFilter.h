/*
    FILE: staticPointFilter.h
    ---------------------------------
    静态点滤波器头文件
*/
#ifndef ONBOARDDETECTOR_STATICPOINTFILTER_H
#define ONBOARDDETECTOR_STATICPOINTFILTER_H

#include <onboard_detector/lidarDetector.h>
#include <onboard_detector/utils.h>
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
  void updateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                 double current_time);

  // 点级过滤 (原 filter 函数)
  void filterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                    const std::vector<onboardDetector::box3D> &protected_boxes =
                        std::vector<onboardDetector::box3D>());

  // 聚类级过滤
  void filterClusters(std::vector<onboardDetector::Cluster> &clusters,
                      std::vector<onboardDetector::box3D> &bboxes,
                      float static_ratio_threshold);

  // 清理动态物体历史轨迹区域的体素（动态反哺机制）
  void
  clearDynamicRegions(const std::vector<onboardDetector::box3D> &dynamic_boxes);

  // 清理旧体素
  void cleanMap(double current_time);

private:
  // 计算体素键值的辅助函数
  long long getVoxelKey(const pcl::PointXYZ &point);

  // 检查点是否在边界框内
  bool isPointInBox(const pcl::PointXYZ &pt, const onboardDetector::box3D &box);

  // 检查点是否为静态 (基于当前地图)
  bool isPointStatic(const pcl::PointXYZ &pt);

  bool enabled_;
  float voxel_size_;
  int hit_threshold_;
  double time_threshold_;

  // 体素地图：键值 -> 状态
  std::unordered_map<long long, VoxelStatus> voxel_map_;
};

} // namespace onboardDetector

#endif // ONBOARDDETECTOR_STATICPOINTFILTER_H
