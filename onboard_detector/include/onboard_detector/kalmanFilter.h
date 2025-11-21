/*
	FILE: kalman_filter.h
	--------------------------------------
	header of kalman_filter velocity estimator
*/

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;

namespace onboardDetector{
    /**
     * @class kalman_filter
     * @brief 一个通用的卡尔曼滤波器实现类
     * 
     * 该类封装了卡尔曼滤波器的核心矩阵和算法步骤，包括状态预测和测量更新。
     * 使用前需要通过 setup() 函数进行初始化。
     */
    class kalman_filter
    {
        private:
        // members
        bool is_initialized; // 滤波器是否已初始化的标志

        // --- 卡尔曼滤波器核心矩阵 ---
        MatrixXd states; // 状态向量 (x)，例如 [px, py, vx, vy, ax, ay]'
        MatrixXd A;      // 状态转移矩阵 (State Transition Matrix)，描述状态如何从上一时刻演变到当前时刻
        MatrixXd B;      // 控制输入矩阵 (Control Input Matrix)，描述控制输入如何影响状态
        MatrixXd H;      // 观测矩阵 (Observation Matrix)，将真实状态空间映射到观测空间
        MatrixXd P;      // 状态协方差矩阵 (State Covariance Matrix)，表示状态估计的不确定性
        MatrixXd Q;      // 过程噪声协方差矩阵 (Process Noise Covariance)，表示状态转移模型的不确定性
        MatrixXd R;      // 观测噪声协方差矩阵 (Measurement Noise Covariance)，表示传感器测量的不确定性

        public:
        /**
         * @brief 构造函数
         */
        kalman_filter();

        /**
         * @brief 初始化或设置滤波器参数
         * @param states 初始状态向量
         * @param A 状态转移矩阵
         * @param B 控制输入矩阵
         * @param H 观测矩阵
         * @param P 初始状态协方差矩阵
         * @param Q 过程噪声协方差矩阵
         * @param R 观测噪声协方差矩阵
         */
        void setup(const MatrixXd& states,
                   const MatrixXd& A,
                   const MatrixXd& B,
                   const MatrixXd& H,
                   const MatrixXd& P,
                   const MatrixXd& Q,
                   const MatrixXd& R);

        /**
         * @brief 单独设置状态转移矩阵 A
         * @param A 新的状态转移矩阵。当采样时间 dt 变化时，此函数非常有用。
         */
        void setA(const MatrixXd& A);

        /**
         * @brief 执行一次完整的卡尔曼滤波（预测+更新）
         * @param z 当前的测量向量 (Measurement)
         * @param u 当前的控制输入向量 (Control Input)
         */
        void estimate(const MatrixXd& z, const MatrixXd& u);

        /**
         * @brief 从状态向量中读取指定的输出值
         * @param state_index 要读取的状态在状态向量中的索引
         * @return 返回该状态的估计值
         */
        double output(int state_index);

        /**
         * @brief 获取当前状态向量
         * @return 返回状态向量的常量引用
         */
        const MatrixXd& getStates() const { return states; }

        /**
         * @brief 获取当前状态协方差矩阵
         * @return 返回协方差矩阵的常量引用
         */
        const MatrixXd& getCovariance() const { return P; }
    };
}

#endif