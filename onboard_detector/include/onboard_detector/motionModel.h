/*
    FILE: motionModel.h
    ------------------------------
    Multi-model motion models for object tracking
    Includes: CA (Constant Acceleration), CV (Constant Velocity), CTRA (Constant
   Turn Rate and Acceleration)
*/

#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace onboardDetector {

/**
 * @brief 抽象运动模型基类
 */
class MotionModel {
public:
  MotionModel() : state_dim_(-1), meas_dim_(-1), dt_(0.1) {}
  virtual ~MotionModel() = default;

  // 核心接口函数
  virtual Eigen::VectorXd getInitState(const Eigen::VectorXd &detection) = 0;
  virtual Eigen::MatrixXd getInitCovP() = 0;
  virtual Eigen::MatrixXd getProcessNoiseQ() = 0;
  virtual Eigen::MatrixXd getMeasNoiseR() = 0;

  // 获取状态转移矩阵（线性模型）或雅可比矩阵（非线性模型）
  virtual Eigen::MatrixXd getTransitionF(const Eigen::VectorXd &state) = 0;

  // 获取观测矩阵（线性模型）或雅可比矩阵（非线性模型）
  virtual Eigen::MatrixXd getMeasurementH(const Eigen::VectorXd &state) = 0;

  // 状态转移函数（用于EKF的非线性模型）
  virtual Eigen::VectorXd stateTransition(const Eigen::VectorXd &state) {
    return getTransitionF(state) * state;
  }

  // 状态到测量的映射（用于EKF）
  virtual Eigen::VectorXd stateToMeasurement(const Eigen::VectorXd &state) = 0;

  // 工具函数
  virtual void normalizeYaw(Eigen::VectorXd &state) = 0;
  virtual void normalizeYawInResidual(Eigen::VectorXd &residual) = 0;

  // 设置时间步长
  void setDt(double dt) { dt_ = dt; }

  // 获取维度
  int getStateDim() const { return state_dim_; }
  int getMeasDim() const { return meas_dim_; }

protected:
  int state_dim_; // 状态向量维度
  int meas_dim_;  // 测量向量维度
  double dt_;     // 时间步长
  double sigma_;  // 过程噪声标准差

  // 工具函数：将角度归一化到 [-pi, pi]
  static double wrapToPi(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }
};

/**
 * @brief CA模型 - 恒定加速度模型 (用于人、无人机)
 *
 * 人的状态向量: [x, y, z, vx, vy, ax, ay]^T (7维) - 增加z轴位置
 * 无人机的状态向量: [x, y, z, vx, vy, vz, ax, ay, az]^T (9维)
 *
 * 测量向量: [x, y, z]^T (3维)
 */
class CA_Model : public MotionModel {
public:
  /**
   * @param use_3d true表示3D模型(无人机)，false表示2D模型(人)
   * @param sigma 过程噪声标准差 (jerk noise)
   */
  explicit CA_Model(bool use_3d = false, double sigma = 1.0);

  Eigen::VectorXd getInitState(const Eigen::VectorXd &detection) override;
  Eigen::MatrixXd getInitCovP() override;
  Eigen::MatrixXd getProcessNoiseQ() override;
  Eigen::MatrixXd getMeasNoiseR() override;
  Eigen::MatrixXd getTransitionF(const Eigen::VectorXd &state) override;
  Eigen::MatrixXd getMeasurementH(const Eigen::VectorXd &state) override;
  Eigen::VectorXd stateToMeasurement(const Eigen::VectorXd &state) override;

  void normalizeYaw(Eigen::VectorXd &state) override {} // CA模型无yaw
  void normalizeYawInResidual(Eigen::VectorXd &residual) override {
  } // CA模型无yaw

private:
  bool use_3d_; // 是否使用3D模型
};

/**
 * @brief CV模型 - 恒定速度模型 (用于其他类别)
 *
 * 状态向量: [x, y, z, vx, vy, vz]^T (6维)
 * 测量向量: [x, y, z]^T (3维)
 */
class CV_Model : public MotionModel {
public:
  /**
   * @param sigma 过程噪声标准差 (acceleration noise)
   */
  explicit CV_Model(double sigma = 1.0);

  Eigen::VectorXd getInitState(const Eigen::VectorXd &detection) override;
  Eigen::MatrixXd getInitCovP() override;
  Eigen::MatrixXd getProcessNoiseQ() override;
  Eigen::MatrixXd getMeasNoiseR() override;
  Eigen::MatrixXd getTransitionF(const Eigen::VectorXd &state) override;
  Eigen::MatrixXd getMeasurementH(const Eigen::VectorXd &state) override;
  Eigen::VectorXd stateToMeasurement(const Eigen::VectorXd &state) override;

  void normalizeYaw(Eigen::VectorXd &state) override {} // CV模型无yaw
  void normalizeYawInResidual(Eigen::VectorXd &residual) override {
  } // CV模型无yaw
};

/**
 * @brief CTRA模型 - 恒定转弯率和加速度模型 (用于车辆)
 *
 * 状态向量: [x, y, z, v, a, yaw, yaw_rate]^T (7维) - 增加z轴位置
 * - x, y, z: 位置
 * - v: 行驶速度 (标量)
 * - a: 加速度
 * - yaw: 偏航角
 * - yaw_rate: 偏航角速度
 *
 * 测量向量: [x, y, z]^T (3维)
 */
class CTRA_Model : public MotionModel {
public:
  /**
   * @param sigma 过程噪声标准差
   */
  explicit CTRA_Model(double sigma = 1.0);

  Eigen::VectorXd getInitState(const Eigen::VectorXd &detection) override;
  Eigen::MatrixXd getInitCovP() override;
  Eigen::MatrixXd getProcessNoiseQ() override;
  Eigen::MatrixXd getMeasNoiseR() override;
  Eigen::MatrixXd getTransitionF(const Eigen::VectorXd &state) override;
  Eigen::MatrixXd getMeasurementH(const Eigen::VectorXd &state) override;
  Eigen::VectorXd stateTransition(const Eigen::VectorXd &state) override;
  Eigen::VectorXd stateToMeasurement(const Eigen::VectorXd &state) override;

  void normalizeYaw(Eigen::VectorXd &state) override;
  void normalizeYawInResidual(Eigen::VectorXd &residual) override;
};

} // namespace onboardDetector

#endif // MOTION_MODEL_H
