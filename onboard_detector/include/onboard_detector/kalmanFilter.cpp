/*
	FILE: kalman_filter.cpp
	--------------------------------------
	function definition of kalman_filter velocity estimator
*/
#include <onboard_detector/kalmanFilter.h>
using Eigen::MatrixXd;

namespace onboardDetector{
    /**
     * @brief 默认构造函数
     *        初始化时将 is_initialized 标志设为 false。
     */
    kalman_filter::kalman_filter()
    {
        this->is_initialized = false;
    }

    /**
     * @brief 设置并初始化卡尔曼滤波器的所有矩阵和初始状态。
     */
    void kalman_filter::setup(const MatrixXd& states, const MatrixXd& A, const MatrixXd& B, const MatrixXd& H, const MatrixXd& P, const MatrixXd& Q, const MatrixXd& R)
    {
        this->states = states; // 状态向量 x
        this->A = A;           // 状态转移矩阵
        this->B = B;           // 控制输入矩阵
        this->H = H;           // 观测矩阵
        this->P = P;           // 状态协方差矩阵
        this->Q = Q;           // 过程噪声协方差
        this->R = R;           // 观测噪声协方差
        this->is_initialized = true; // 标记为已初始化
    }

    /**
     * @brief 单独设置状态转移矩阵 A。
     *        当系统的时间步长 dt 发生变化时，这个函数很有用。
     */
    void kalman_filter::setA(const MatrixXd& A)
    {
        this->A = A;
    }

    /**
     * @brief 执行一次完整的卡尔曼滤波估计，包括预测和更新两个步骤。
     * @param z 当前的测量值向量。
     * @param u 当前的控制输入向量。
     */
    void kalman_filter::estimate(const MatrixXd& z, const MatrixXd& u)
    {
        // --- 1. 预测 (Predict) ---
        // 预测下一时刻的状态
        this->states = this->A * this->states + this->B * u;
        // 预测下一时刻的状态协方差（不确定性），A.transpose()矩阵A的转置
        this->P = this->A * this->P * this->A.transpose() + this->Q;

        // cout << "prediction: " << endl;
        // cout << this->states << endl;

        // --- 2. 更新 (Update) ---
        // 计算新息协方差矩阵 S (Innovation Covariance)
        MatrixXd S = this->R + this->H * this->P * this->H.transpose(); // innovation matrix
        // 计算最优卡尔曼增益 K (Kalman Gain)，矩阵S的逆矩阵
        MatrixXd K = this->P * this->H.transpose() * S.inverse(); // kalman gain
 
        // 使用测量值 z 和卡尔曼增益 K 来更新状态估计
        this->states = this->states + K * (z - this->H * this->states);
        // 更新状态协方差矩阵 P，减小不确定性
        this->P = (MatrixXd::Identity(this->P.rows(),this->P.cols()) - K * this->H) * this->P;

    }

    /**
     * @brief 获取状态向量中指定索引的估计值。
     * @param state_index 要获取的状态的索引。
     * @return 如果滤波器已初始化，则返回对应的状态值；否则返回0。
     */
    double kalman_filter::output(int state_index)
    {
        if(this->is_initialized)
        {
            return this->states(state_index, 0);
        }
        else
        {
            return 0;
        }
    }

}