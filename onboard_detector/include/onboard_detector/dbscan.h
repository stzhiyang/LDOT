/*
    FILE: dbscan.h
    ------------------
    helper class header for dbscan
*/
#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>

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
        DBSCAN(unsigned int minPts, float eps, vector<Point> points, bool useAdaptive = false, float distScale = 0.05){
            m_minPoints = minPts;
            m_epsilon = eps;  // 作为基础epsilon
            m_points = points;
            m_pointSize = points.size();
            m_useAdaptiveEps = useAdaptive;
            m_distanceScale = distScale;
        }
        ~DBSCAN(){}

        int run();
        vector<int> calculateCluster(Point point);
        int expandCluster(Point point, int clusterID);
        inline double calculateDistance(const Point& pointCore, const Point& pointTarget);
        inline double calculateDistanceToOrigin(const Point& point);  // 计算点到原点的距离
        inline double getAdaptiveEpsilon(const Point& point);  // 获取自适应epsilon
        
    public:
        vector<Point> m_points;
        
    private:    
        unsigned int m_pointSize;
        unsigned int m_minPoints;
        float m_epsilon;  // 基础epsilon值
        bool m_useAdaptiveEps;  // 是否使用基于距离的自适应epsilon
        float m_distanceScale;  // 距离缩放因子，用于计算自适应epsilon
    };
}
#endif // DBSCAN_H
