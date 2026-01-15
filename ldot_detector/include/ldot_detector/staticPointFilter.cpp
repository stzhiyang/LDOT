/*
    FILE: staticPointFilter.cpp
    ---------------------------------
    静态点滤波器实现
*/
#include <cmath>
#include <ldot_detector/staticPointFilter.h>
#include <ldot_detector/utils.h>

namespace onboardDetector {

StaticPointFilter::StaticPointFilter()
    : enabled_(false), voxel_size_(0.1), hit_threshold_(5),
      time_threshold_(5.0),
      ray_cast_decrement_(1), ray_cast_skip_counter_(0), frame_count_(0),
      sensor_position_(Eigen::Vector3d::Zero()) {}

StaticPointFilter::~StaticPointFilter() {}

void StaticPointFilter::setParams(bool enabled, float voxel_size,
                                  int hit_threshold, double time_threshold,
                                  int ray_cast_decrement) {
  enabled_ = enabled;
  voxel_size_ = voxel_size;
  hit_threshold_ = hit_threshold;
  time_threshold_ = time_threshold;
  ray_cast_decrement_ = ray_cast_decrement;
}

long long StaticPointFilter::getVoxelKey(const pcl::PointXYZ &point) {
  // 向下取整
  int x_idx = std::floor(point.x / voxel_size_);
  int y_idx = std::floor(point.y / voxel_size_);
  int z_idx = std::floor(point.z / voxel_size_);

  return getVoxelKeyFromCoords(x_idx, y_idx, z_idx);
}

long long StaticPointFilter::getVoxelKeyFromCoords(int x_idx, int y_idx,
                                                   int z_idx) {
  // 每个坐标的质数。为了下面进行异或运算和防止不同点云占据同一个体素格子，即哈希函数的运算
  const long long p1 = 73856093;
  const long long p2 = 19349663;
  const long long p3 = 83492791;

  // 这个就是哈希函数，给每一个小方块一个独一无二的 ID，方便快速查找
  return (long long)(x_idx * p1) ^ (long long)(y_idx * p2) ^
         (long long)(z_idx * p3);
}

bool StaticPointFilter::isPointInBox(const pcl::PointXYZ &pt,
                                     const onboardDetector::box3D &box) {
  // 计算点相对于box中心的位置
  float dx = pt.x - box.x;
  float dy = pt.y - box.y;
  float dz = pt.z - box.z;

  // 在box局部坐标系中进行AABB检查
  float half_x = box.x_width / 2.0;
  float half_y = box.y_width / 2.0;
  float half_z = box.z_width / 2.0;

  if (std::abs(dx) <= half_x && std::abs(dy) <= half_y &&
      std::abs(dz) <= half_z) {
    return true;
  }
  return false;
}

// 判断体素格子的命中次数，如果大于阈值为静态，返回true
// 注意：此函数依赖 sensor_position_ 成员变量，需要先调用 updateMap 更新传感器位置
bool StaticPointFilter::isPointStatic(const pcl::PointXYZ &pt) {
  // 使用距离自适应阈值
  long long key = getVoxelKey(pt);
  if (voxel_map_.find(key) != voxel_map_.end()) {
    int adaptive_thresh = getAdaptiveThreshold(pt);
    return voxel_map_[key].hit_count > adaptive_thresh;
  }
  return false;
}

// 距离自适应阈值 - 远距离降低判定门槛，补偿点云稀疏性
// 使用成员变量 sensor_position_ 计算点到传感器的距离
int StaticPointFilter::getAdaptiveThreshold(const pcl::PointXYZ &pt) {
  double dx = pt.x - sensor_position_.x();
  double dy = pt.y - sensor_position_.y();
  double dist_sq = dx * dx + dy * dy;
  
  if (dist_sq < 25.0) {  
    return hit_threshold_;
  } else if (dist_sq < 100.0) {  
    return std::max(2, hit_threshold_ - 1);
  } else if (dist_sq < 400.0) {  
    return std::max(2, hit_threshold_ - 2);
  } else {
    return std::max(2, hit_threshold_ - 3);
  }
}

// 使用 Bresenham 3D 算法进行射线投射
// 核心思想：从传感器位置到激光点的射线路径上，所有穿过的体素都应该是"空闲"的
// 因为激光穿过了这些位置而没有发生碰撞。对这些体素进行递减操作，可以快速清除
// 射线投射清除：清理射线路径上的体素（不包括终点）
// 
// 清理逻辑：
// 1. 当前帧被观测的体素：绝对不清除（新发现的静态物体）
// 2. 保护区内的体素：直接清除（动态物体不应累积到静态地图）
// 3. 非保护区 + hit_count >= hit_threshold_ * 0.5：不清除（稳定静态物体）
// 4. 非保护区 + hit_count < hit_threshold_ * 0.5：递减（可能是动态残影）
void StaticPointFilter::rayCast(const Eigen::Vector3d &sensor_position,
                                const pcl::PointXYZ &end_point,
                                const std::vector<onboardDetector::box3D> *protected_boxes) {
  // 将起点和终点转换为体素索引
  int x0 = std::floor(sensor_position.x() / voxel_size_);
  int y0 = std::floor(sensor_position.y() / voxel_size_);
  int z0 = std::floor(sensor_position.z() / voxel_size_);

  int x1 = std::floor(end_point.x / voxel_size_);
  int y1 = std::floor(end_point.y / voxel_size_);
  int z1 = std::floor(end_point.z / voxel_size_);

  // 计算差值
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int dz = std::abs(z1 - z0);

  // 确定步进方向
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int sz = (z0 < z1) ? 1 : -1;

  int dm = std::max({dx, dy, dz});

  // 如果射线太短（小于2个体素），跳过
  if (dm < 2) {
    return;
  }

  // 初始化当前位置
  int x = x0, y = y0, z = z0;

  // 误差累积器
  int err_x = dm / 2;
  int err_y = dm / 2;
  int err_z = dm / 2;

  // 预先计算终点体素的哈希
  long long end_key = getVoxelKeyFromCoords(x1, y1, z1);

  // 沿着射线遍历所有体素（不包括终点）
  for (int i = 0; i < dm - 1; ++i) {
    // Bresenham 步进逻辑
    err_x -= dx;
    if (err_x < 0) {
      x += sx;
      err_x += dm;
    }

    err_y -= dy;
    if (err_y < 0) {
      y += sy;
      err_y += dm;
    }

    err_z -= dz;
    if (err_z < 0) {
      z += sz;
      err_z += dm;
    }

    long long key = getVoxelKeyFromCoords(x, y, z);
    
    // 跳过终点体素（终点已在第一步处理）
    if (key == end_key) {
      break;
    }
    
    // 【规则1】当前帧被观测的体素：绝对不清除
    if (current_frame_hits_.count(key) > 0) {
      continue;
    }
    
    auto it = voxel_map_.find(key);
    if (it == voxel_map_.end()) {
      continue;
    }
    
    // 检查当前体素是否在保护区内
    bool in_protected = false;
    if (protected_boxes != nullptr) {
      pcl::PointXYZ voxel_center;
      voxel_center.x = x * voxel_size_ + voxel_size_ / 2.0f;
      voxel_center.y = y * voxel_size_ + voxel_size_ / 2.0f;
      voxel_center.z = z * voxel_size_ + voxel_size_ / 2.0f;
      
      for (const auto &box : *protected_boxes) {
        if (isPointInBox(voxel_center, box)) {
          in_protected = true;
          break;
        }
      }
    }
    
    if (in_protected) {
      // 【规则2】保护区内：直接清除
      voxel_map_.erase(it);
    } else if (it->second.hit_count < hit_threshold_ * 0.5) {
      // 【规则4】非保护区 + 低 hit_count：递减（可能是动态残影）
      if (it->second.hit_count <= ray_cast_decrement_) {
        voxel_map_.erase(it);
      } else {
        it->second.hit_count -= ray_cast_decrement_;
      }
    }
    // 【规则3】非保护区 + hit_count >= hit_threshold_ * 0.5：不处理（稳定静态物体）
  }
}

void StaticPointFilter::updateMap(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double current_time,
    const Eigen::Vector3d &sensor_position,
    const std::vector<onboardDetector::box3D> *protected_boxes) {
  if (cloud->empty()) {
    return;
  }

  // 更新传感器位置（供后续 isPointStatic 等函数使用）
  sensor_position_ = sensor_position;

  // 清空当前帧命中集合，准备记录新的命中
  current_frame_hits_.clear();

  // 【第一遍】处理终点体素
  // - 非保护区：累积 hit_count
  // - 保护区：递减 hit_count（动态物体表面的体素）
  for (const auto &point : cloud->points) {
    // 检查点是否在保护区域内
    bool in_protected_area = false;
    if (protected_boxes != nullptr) {
      for (const auto &box : *protected_boxes) {
        if (isPointInBox(point, box)) {
          in_protected_area = true;
          break;
        }
      }
    }
    
    long long key = getVoxelKey(point);
    
    if (in_protected_area) {
      // 保护区内：递减已有体素的 hit_count（清除动态物体残影）
      auto it = voxel_map_.find(key);
      if (it != voxel_map_.end()) {
        if (it->second.hit_count <= ray_cast_decrement_) {
          voxel_map_.erase(it);
        } else {
          it->second.hit_count -= ray_cast_decrement_;
        }
      }
    } else {
      // 非保护区：累积
      // 记录当前帧命中的体素（用于射线投射时保护）
      current_frame_hits_.insert(key);

      // 更新体素状态
      VoxelStatus &status = voxel_map_[key]; 
      status.last_seen_time = current_time;

      // 增加体素格子命中计数，但进行截断以避免溢出
      if (status.hit_count <= hit_threshold_ + 1) {
        status.hit_count++;
      }
    }
  }

  // 【第二遍】射线投射清除（降采样）
  // 使用遮挡检测保护静态物体，不再需要距离过滤
  const int ray_cast_skip_interval = 5; // 每5个点做一次射线投射

  ray_cast_skip_counter_ = 0; // 重置计数器
  for (const auto &point : cloud->points) {
    // 【射线投射降采样】只对部分点进行射线投射
    ray_cast_skip_counter_++;
    if (ray_cast_skip_counter_ >= ray_cast_skip_interval) {
      rayCast(sensor_position, point, protected_boxes);
      ray_cast_skip_counter_ = 0;
    }
  }

  // 每10帧清理一次地图
  frame_count_++;
  if (frame_count_ >= 1) {
    cleanMap(current_time);
    frame_count_ = 0;
  }
}

void StaticPointFilter::filterPoints(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::vector<onboardDetector::box3D> &protected_boxes) {
  if (!enabled_ || cloud->empty()) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  filtered_cloud->reserve(cloud->size());

  for (const auto &point : cloud->points) {
    // 检查是否为静态
    if (isPointStatic(point)) {
      // 静态点：检查是否在保护区域内
      bool protected_point = false;
      for (const auto &box : protected_boxes) {
        if (isPointInBox(point, box)) {
          protected_point = true;
          break;
        }
      }

      if (protected_point) {
        // 在保护区内的静态点：保留（可能是动态物体表面）
        filtered_cloud->push_back(point);
      }
      // 否则，它是静态的且不在保护区内，移除它
    } else {
      // 非静态，保留
      filtered_cloud->push_back(point);
    }
  }

  // 用过滤后的点云替换原始点云
  *cloud = *filtered_cloud;
}


void StaticPointFilter::cleanMap(double current_time) {
  for (auto it = voxel_map_.begin(); it != voxel_map_.end();) {
    if (current_time - it->second.last_seen_time > time_threshold_) {
      it = voxel_map_.erase(it);
    } else {
      ++it;
    }
  }
}

bool StaticPointFilter::checkBoxCollision(const Eigen::Vector3d &center,
                                          const Eigen::Vector3d &size,
                                          double inflation) {
  // 计算膨胀后的半尺寸
  double half_x = (size.x() / 2.0) + inflation;
  double half_y = (size.y() / 2.0) + inflation;
  double half_z = (size.z() / 2.0) + inflation;

  // 计算需要检查的体素范围
  int x_range = static_cast<int>(std::ceil(half_x / voxel_size_)) + 1;
  int y_range = static_cast<int>(std::ceil(half_y / voxel_size_)) + 1;
  int z_range = static_cast<int>(std::ceil(half_z / voxel_size_)) + 1;

  // 遍历包围框范围内的所有体素
  for (int dx = -x_range; dx <= x_range; ++dx) {
    for (int dy = -y_range; dy <= y_range; ++dy) {
      for (int dz = -z_range; dz <= z_range; ++dz) {
        // 计算体素中心点
        pcl::PointXYZ voxel_center;
        voxel_center.x = static_cast<float>(center.x() + dx * voxel_size_);
        voxel_center.y = static_cast<float>(center.y() + dy * voxel_size_);
        voxel_center.z = static_cast<float>(center.z() + dz * voxel_size_);

        // 检查体素中心是否在膨胀后的包围框内
        double local_x = std::abs(voxel_center.x - center.x());
        double local_y = std::abs(voxel_center.y - center.y());
        double local_z = std::abs(voxel_center.z - center.z());

        if (local_x <= half_x && local_y <= half_y && local_z <= half_z) {
          // 体素在包围框内，检查是否为静态体素
          if (isPointStatic(voxel_center)) {
            // 发现碰撞，立即返回
            return true;
          }
        }
      }
    }
  }

  // 未发现碰撞
  return false;
}

} // namespace onboardDetector
