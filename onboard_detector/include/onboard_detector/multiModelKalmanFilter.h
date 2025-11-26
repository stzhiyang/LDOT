/*
    FILE: multiModelKalmanFilter.h
    ------------------------------
    Multi-model Kalman Filter for different object categories
    Includes Linear KF and Extended KF
*/

#ifndef MULTI_MODEL_KALMAN_FILTER_H
#define MULTI_MODEL_KALMAN_FILTER_H

#include "motionModel.h"
#include <Eigen/Dense>
#include <memory>

namespace onboardDetector {

struct KF_Params {
  CA_Params ca_human;
  CA_Params ca_uav;
  CV_Params cv;
  CTRA_Params ctra;
};

/**
 * @brief 卡尔曼滤波器基类
 */
class KalmanFilterBase {
public:
  KalmanFilterBase(std::shared_ptr<MotionModel> model)
      : model_(model), is_initialized_(false) {
    state_dim_ = model_->getStateDim();
    meas_dim_ = model_->getMeasDim();
  }

  virtual ~KalmanFilterBase() = default;

  /**
   * @brief 初始化滤波器
   * @param detection 初始检测 (位置信息)
   */
  virtual void initialize(const Eigen::VectorXd &detection) {
    // 初始化状态
    state_ = model_->getInitState(detection);

    // 初始化协方差
    P_ = model_->getInitCovP();

    // 初始化噪声矩阵
    Q_ = model_->getProcessNoiseQ();
    R_ = model_->getMeasNoiseR();

    is_initialized_ = true;
  }

  /**
   * @brief 预测步骤
   */
  virtual void predict() = 0;

  /**
   * @brief 更新步骤
   * @param measurement 测量值
   */
  virtual void update(const Eigen::VectorXd &measurement) = 0;

  /**
   * @brief 获取当前状态
   */
  const Eigen::VectorXd &getState() const { return state_; }

  /**
   * @brief 获取当前协方差
   */
  const Eigen::MatrixXd &getCovariance() const { return P_; }

  /**
   * @brief 检查是否已初始化
   */
  bool isInitialized() const { return is_initialized_; }

  /**
   * @brief 设置时间步长
   */
  void setDt(double dt) { model_->setDt(dt); }

  /**
   * @brief 设置测量噪声协方差矩阵R (用于自适应调整)
   * @param R_new 新的测量噪声协方差矩阵
   */
  void setMeasNoiseR(const Eigen::MatrixXd &R_new) { R_ = R_new; }

  /**
   * @brief 获取当前测量噪声协方差矩阵R
   * @return 测量噪声协方差矩阵
   */
  const Eigen::MatrixXd &getMeasNoiseR() const { return R_; }

  /**
   * @brief 获取底层运动模型 (用于访问原始参数)
   * @return 运动模型的智能指针
   */
  std::shared_ptr<MotionModel> getModel() { return model_; }

protected:
  std::shared_ptr<MotionModel> model_; // 运动模型

  Eigen::VectorXd state_; // 状态向量
  Eigen::MatrixXd P_;     // 状态协方差矩阵
  Eigen::MatrixXd Q_;     // 过程噪声协方差
  Eigen::MatrixXd R_;     // 测量噪声协方差

  int state_dim_;       // 状态维度
  int meas_dim_;        // 测量维度
  bool is_initialized_; // 初始化标志
};

/**
 * @brief 线性卡尔曼滤波器 (用于CA和CV模型)
 */
class LinearKalmanFilter : public KalmanFilterBase {
public:
  LinearKalmanFilter(std::shared_ptr<MotionModel> model)
      : KalmanFilterBase(model) {}

  void predict() override {
    if (!is_initialized_)
      return;

    // 获取状态转移矩阵
    Eigen::MatrixXd F = model_->getTransitionF(state_);

    // 预测状态
    state_ = F * state_;

    // 预测协方差
    P_ = F * P_ * F.transpose() + Q_;

    // 归一化yaw（如果有）
    model_->normalizeYaw(state_);
  }

  void update(const Eigen::VectorXd &measurement) override {
    if (!is_initialized_)
      return;

    // 获取观测矩阵
    Eigen::MatrixXd H = model_->getMeasurementH(state_);

    // 计算新息 (innovation)
    Eigen::VectorXd y = measurement - H * state_;

    // 归一化新息中的角度
    model_->normalizeYawInResidual(y);

    // 新息协方差
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;

    // 卡尔曼增益
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // 更新状态
    state_ = state_ + K * y;

    // 更新协方差
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    P_ = (I - K * H) * P_;

    // 归一化yaw
    model_->normalizeYaw(state_);
  }
};

/**
 * @brief 扩展卡尔曼滤波器 (用于CTRA等非线性模型)
 */
class ExtendedKalmanFilter : public KalmanFilterBase {
public:
  ExtendedKalmanFilter(std::shared_ptr<MotionModel> model)
      : KalmanFilterBase(model) {}

  void predict() override {
    if (!is_initialized_)
      return;

    // 使用非线性状态转移函数
    state_ = model_->stateTransition(state_);

    // 计算雅可比矩阵
    Eigen::MatrixXd F = model_->getTransitionF(state_);

    // 预测协方差
    P_ = F * P_ * F.transpose() + Q_;

    // 归一化yaw
    model_->normalizeYaw(state_);
  }

  void update(const Eigen::VectorXd &measurement) override {
    if (!is_initialized_)
      return;

    // 将状态映射到测量空间（非线性）
    Eigen::VectorXd predicted_meas = model_->stateToMeasurement(state_);

    // 计算新息
    Eigen::VectorXd y = measurement - predicted_meas;

    // 归一化新息中的角度
    model_->normalizeYawInResidual(y);

    // 计算观测雅可比矩阵
    Eigen::MatrixXd H = model_->getMeasurementH(state_);

    // 新息协方差
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;

    // 卡尔曼增益
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // 更新状态
    state_ = state_ + K * y;

    // 更新协方差 (Joseph form for numerical stability)
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    Eigen::MatrixXd I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();

    // 归一化yaw
    model_->normalizeYaw(state_);
  }
};

/**
 * @brief 工厂函数：根据物体类别创建相应的卡尔曼滤波器
 * @param is_human 是否为人
 * @param is_che 是否为车
 * @param is_uav 是否为无人机
 * @param is_else 是否为其他类别
 * @return 卡尔曼滤波器的智能指针
 */
inline std::shared_ptr<KalmanFilterBase>
createKalmanFilter(bool is_human, bool is_che, bool is_uav, bool is_else,
                   const KF_Params &params) {

  if (is_human) {
    // 人 -> CA-KF (2D)
    auto model = std::make_shared<CA_Model>(params.ca_human, false); // 2D
    return std::make_shared<LinearKalmanFilter>(model);
  } else if (is_uav) {
    // 无人机 -> CA-KF (3D)
    auto model = std::make_shared<CA_Model>(params.ca_uav, true); // 3D
    return std::make_shared<LinearKalmanFilter>(model);
  } else if (is_che) {
    // 车 -> CTRA-EKF
    auto model = std::make_shared<CTRA_Model>(params.ctra);
    return std::make_shared<ExtendedKalmanFilter>(model);
  } else {
    // 其他 -> CV-KF (3D)
    auto model = std::make_shared<CV_Model>(params.cv);
    return std::make_shared<LinearKalmanFilter>(model);
  }
}

} // namespace onboardDetector

#endif // MULTI_MODEL_KALMAN_FILTER_H
