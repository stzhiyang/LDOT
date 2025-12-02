/*
        FILE: utils.h
        --------------------------
        function utils for detectors
    文件: utils.h
    --------------------------
    检测器功能工具
 */

#ifndef ONBOARD_DETECTOR_UTILS_H
#define ONBOARD_DETECTOR_UTILS_H
#include <Eigen/Eigen>
#include <geometry_msgs/Quaternion.h>
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace onboardDetector {
// 定义圆周率常量
const double PI_const = 3.1415926;
// 定义3D包围框结构体
struct box3D {
  /* data */
  // 包围框中心点坐标
  double x, y, z;
  // 包围框在x, y, z方向的宽度
  double x_width, y_width, z_width;
  // 包围框ID
  double id;
  // 包围框在x, y, z方向的速度
  double Vx = 0, Vy = 0, Vz = 0;
  // 包围框在x, y, z方向的加速度
  double Ax = 0, Ay = 0, Az = 0;
  // 是否为人
  bool is_human = false;
  // 是否为车
  bool is_che = false;
  // 是否为无人机
  bool is_uav = false;
  // 其他类别
  bool is_else = false;
  // 是否被检测为动态物体 (false: 未被检测为动态, true: 被检测为动态)
  bool is_dynamic = false;
  // 强制未来包围框尺寸固定的标志
  bool fix_size = false;
  // 是否为动态候选物体
  bool is_dynamic_candidate = false;
  // 是否经过估计
  bool is_estimated = false;
  // 边界框的偏航角（yaw），用于支持有向边界框（OBB）
  // 0表示未旋转，弧度制，绕z轴旋转
  double yaw = 0.0;
};

// 从roll, pitch, yaw角度创建四元数
inline geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch,
                                                     double yaw) {
  // 将yaw角度规范化到[-PI, PI]
  if (yaw > PI_const) {
    yaw = yaw - 2 * PI_const;
  }
  tf2::Quaternion quaternion_tf2;
  // 设置roll, pitch, yaw角度
  quaternion_tf2.setRPY(roll, pitch, yaw);
  // 转换为geometry_msgs::Quaternion
  geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
  return quaternion;
}

// 从四元数中提取yaw角度
inline double rpy_from_quaternion(const geometry_msgs::Quaternion &quat) {
  // 返回值范围是[0, 2pi]
  tf2::Quaternion tf_quat;
  tf2::convert(quat, tf_quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
  return yaw;
}

// 从四元数中提取roll, pitch, yaw角度
inline void rpy_from_quaternion(const geometry_msgs::Quaternion &quat,
                                double &roll, double &pitch, double &yaw) {
  tf2::Quaternion tf_quat;
  tf2::convert(quat, tf_quat);
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
}

// 计算两个向量之间的夹角
inline double angleBetweenVectors(const Eigen::Vector3d &a,
                                  const Eigen::Vector3d &b) {
  return std::atan2(a.cross(b).norm(), a.dot(b));
}

// 计算点云的中心
inline Eigen::Vector3d
computeCenter(const std::vector<Eigen::Vector3d> &points) {
  Eigen::Vector3d center(0.0, 0.0, 0.0);
  if (points.empty()) {
    return center;
  }

  // 遍历所有点，将各点坐标累加到center变量中
  // 将累加结果除以点的总数，得到平均坐标即为中心点
  for (const auto &p : points) {
    center += p;
  }
  center /= static_cast<double>(points.size());

  return center;
}

// 计算点云的标准差
inline Eigen::Vector3d computeStd(const std::vector<Eigen::Vector3d> &points,
                                  const Eigen::Vector3d &center) {
  Eigen::Vector3d stds(0.0, 0.0, 0.0);
  if (points.empty()) {
    return stds;
  }

  for (const auto &p : points) {
    Eigen::Vector3d diff = p - center;
    stds(0) += diff(0) * diff(0);
    stds(1) += diff(1) * diff(1);
    stds(2) += diff(2) * diff(2);
  }
  stds /= static_cast<double>(points.size());
  stds = stds.array().sqrt();

  return stds;
}
} // namespace onboardDetector

#endif