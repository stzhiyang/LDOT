/*
    文件: lidarDetector.h
    ---------------------------------
    基于激光雷达的障碍物检测器的头文件
*/
#ifndef ONBOARDDETECTOR_LIDARDETECTOR_H
#define ONBOARDDETECTOR_LIDARDETECTOR_H

#include <ros/ros.h>
#include <ldot_detector/dbscan.h>
#include <ldot_detector/utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>

namespace onboardDetector{
    // 定义一个结构体 `Cluster`，用于存储单个点云簇的相关信息
    struct Cluster
    {
        int cluster_id;               // 点云簇的ID
        Eigen::Vector4f centroid;     // 点云簇的中心
        pcl::PointCloud<pcl::PointXYZ>::Ptr points; // 属于该簇的点云

        // 几何信息
        Eigen::Vector3f dimensions;    // 边界框尺寸
        Eigen::Matrix3f eigen_vectors; // 通过主成分分析（PCA）得到的特征向量
        Eigen::Vector3f eigen_values;  // 通过主成分分析（PCA）得到的特征值

        // 构造函数
        Cluster():
            cluster_id(-1), // 默认ID为-1
            centroid(Eigen::Vector4f::Zero()), // 质心初始化为零
            points(new pcl::PointCloud<pcl::PointXYZ>()) {} // 初始化点云指针
    };

    // `lidarDetector` 类，用于实现基于激光雷达的障碍物检测
    class lidarDetector{
    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_; // 当前处理的点云
        std::vector<onboardDetector::Cluster> clusters_; // 检测到的点云簇列表
        std::vector<onboardDetector::box3D> bboxes_; // 激光雷达检测到的3D边界框列表
        
        // 激光雷达DBSCAN聚类参数
        double eps_;          // DBSCAN的邻域搜索半径 (epsilon)
        int minPts_;          // DBSCAN形成一个簇所需的最小点数
        bool useAdaptive_;    // 是否使用基于距离的自适应epsilon
        double distanceScale_; // 自适应epsilon的距离缩放因子
        double groundHeight_; // 用于过滤地面点的Z轴高度阈值
        double roofHeight_;   // 用于过滤天花板/屋顶点的Z轴高度阈值
        
        // 传感器位置（全局坐标系），用于自适应DBSCAN
        Eigen::Vector3d sensorPosition_;
        
        // 质心补偿参数
        bool enableCentroidCompensation_;   // 是否启用质心补偿
        double compensationRatio_;          // 补偿比例系数 (0.0-1.0)
        double minCompDistance_;            // 最小补偿距离（米）
        double maxCompDistance_;            // 最大补偿距离（米）
        
    public:
        // 构造函数
        lidarDetector();

        // 设置DBSCAN聚类算法的参数
        void setParams(double eps, int minPts, bool useAdaptive = false, double distScale = 0.05);

        // 获取并设置待处理的输入点云
        void getPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

        // 设置传感器位置（全局坐标系），用于自适应DBSCAN
        void setSensorPosition(const Eigen::Vector3d& position);

        // 设置质心补偿参数
        void setCentroidCompensationParams(bool enable, double ratio, double minDist, double maxDist);

        // 对点云执行DBSCAN聚类
        void lidarDBSCAN();

        // 获取检测到的点云簇列表
        std::vector<onboardDetector::Cluster>& getClusters();

        // 获取3D边界框列表
        std::vector<onboardDetector::box3D>& getBBoxes();
    };
}


#endif