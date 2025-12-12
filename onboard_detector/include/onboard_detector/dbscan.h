/*
    FILE: dbscan.h
    ------------------
    helper class header for dbscan
*/
#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;
namespace onboardDetector{
    typedef struct Point_
    {
        float x, y, z;  // X, Y, Z position
        int clusterID;  // clustered ID
    }Point;

    class DBSCAN {
    public:    
        // DBSCAN构造函数，支持普通和自适应模式
        // sensor_pos: 传感器在全局坐标系中的位置（用于自适应epsilon计算）
        DBSCAN(unsigned int minPts, float eps, vector<Point> points, 
               bool useAdaptive = false, float distScale = 0.05,
               float sensor_x = 0.0f, float sensor_y = 0.0f, float sensor_z = 0.0f){
            m_minPoints = minPts;
            m_epsilon = eps;  // 作为基础epsilon
            m_points = points;
            m_pointSize = points.size();
            m_useAdaptiveEps = useAdaptive;
            m_distanceScale = distScale;
            m_sensorX = sensor_x;
            m_sensorY = sensor_y;
            m_sensorZ = sensor_z;
            buildKdTree();  // 构建KD-Tree
        }
        ~DBSCAN(){}

        int run();
        vector<int> calculateClusterKdTree(Point point, int pointIdx);  // 使用KD-Tree的版本
        int expandClusterKdTree(int pointIdx, int clusterID);  // 使用KD-Tree的版本
        inline double calculateDistanceToSensor(const Point& point);  // 计算点到传感器的距离（全局坐标系下）
        inline double getAdaptiveEpsilon(const Point& point);  // 获取自适应epsilon
        void buildKdTree();  // 构建KD-Tree
        
    public:
        vector<Point> m_points;
        
    private:    
        unsigned int m_pointSize;
        unsigned int m_minPoints;
        float m_epsilon;  // 基础epsilon值
        bool m_useAdaptiveEps;  // 是否使用基于距离的自适应epsilon
        float m_distanceScale;  // 距离缩放因子，用于计算自适应epsilon
        
        // KD-Tree相关成员
        pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
        pcl::KdTreeFLANN<pcl::PointXYZ> m_kdtree;
        
        // 传感器位置（全局坐标系），用于自适应epsilon计算
        float m_sensorX;
        float m_sensorY;
        float m_sensorZ;
    };
}
#endif // DBSCAN_H
