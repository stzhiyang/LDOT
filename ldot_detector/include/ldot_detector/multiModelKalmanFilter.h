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
#include <deque>
#include <memory>

namespace onboardDetector {

struct KF_Params {
  CA_Params ca_human;
  CA_Params ca_uav;
  CV_Params cv;
  CTRA_Params ctra;
  int adaptive_window_size = 5;          // 自适应Q估计的窗口大小 (0 表示禁用)
  double adaptive_alpha = 0.3;           // Q 更新的平滑因子 (0.0 - 1.0)
  double adaptive_r_alpha = 0.3;         // R 更新的平滑因子 (0.0 - 1.0)
  double adaptive_min_noise_ratio = 0.5; // 最小Q/R阈值系数
  
  // 协方差上限参数
  bool enable_cov_limit = false;         // 是否启用协方差上限限制
  double max_pos_cov = 9.0;              // 位置协方差上限 (m²)，对应标准差3米
  double max_vel_cov = 16.0;             // 速度协方差上限 ((m/s)²)，对应标准差4m/s
  double max_acc_cov = 100.0;            // 加速度协方差上限 ((m/s²)²)，对应标准差10m/s²
  double prediction_cov_multiplier = 5.0; // 预测时协方差上限倍数（相对于跟踪器上限）
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
    window_size_ = 5;                          // 默认值
    alpha_ = 0.3;                              // 默认平滑因子
    r_alpha_ = 0.3;                            // R 默认平滑因子
    min_q_ = model_->getProcessNoiseQ() * 0.5; // 最小Q阈值 (初始Q的50%)
    min_r_ = model_->getMeasNoiseR() * 0.5;    // 最小R阈值 (初始R的50%)
  }

  void setAdaptiveParams(int size, double alpha, double r_alpha,
                         double min_noise_ratio) {
    window_size_ = size;
    alpha_ = alpha;
    r_alpha_ = r_alpha;
    // 更新最小阈值
    min_q_ = model_->getProcessNoiseQ() * min_noise_ratio;
    min_r_ = model_->getMeasNoiseR() * min_noise_ratio;
  }

  /**
   * @brief 设置协方差上限参数
   * @param enable 是否启用协方差上限限制
   * @param max_pos 位置协方差上限 (m²)
   * @param max_vel 速度协方差上限 ((m/s)²)
   * @param max_acc 加速度协方差上限 ((m/s²)²)
   */
  void setCovLimitParams(bool enable, double max_pos, double max_vel, double max_acc) {
    enable_cov_limit_ = enable;
    max_pos_cov_ = max_pos;
    max_vel_cov_ = max_vel;
    max_acc_cov_ = max_acc;
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

protected:
  // 自适应Q估计
  // 参考: "Adaptive Estimation and Prediction" - 基于新息的Q自适应
  // 公式: Q_new = K * (C_gamma - H * (P_pred - Q_old) * H' - R) * K'
  void estimateAdaptiveQ(const Eigen::VectorXd &innovation,
                         const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
                         const Eigen::MatrixXd &K) {
    if (window_size_ <= 1)
      return;

    // 1. 更新新息缓冲区 (已移至外部或由 updateInnovationBuffer 处理)
    // innovation_buffer_.push_back(innovation);
    // if (innovation_buffer_.size() > static_cast<size_t>(window_size_)) {
    //   innovation_buffer_.pop_front();
    // }

    // 需要足够的样本
    if (innovation_buffer_.size() < static_cast<size_t>(window_size_))
      return;

    // 2. 计算新息的实际协方差 (C_gamma)
    Eigen::MatrixXd C_gamma = Eigen::MatrixXd::Zero(meas_dim_, meas_dim_);
    for (const auto &gamma : innovation_buffer_) {
      C_gamma += gamma * gamma.transpose();
    }
    C_gamma /= innovation_buffer_.size();

    // 3. 估计 Q
    // 理论: C_gamma ~ H * P_pred * H' + R
    // P_pred = F * P_post_prev * F' + Q
    // 所以 C_gamma ~ H * (F * P_post_prev * F' + Q) * H' + R
    // 我们希望找到 Q 使得该式成立。
    // 令 P_prop = F * P_post_prev * F' (没有新 Q 时的传播协方差)
    // 注意: 在当前的 predict() 中，P_ 已经被更新为 P_pred = P_prop + Q_old
    // 所以 P_prop = P_ - Q_

    Eigen::MatrixXd P_prop = P_ - Q_;

    // M 代表归因于过程噪声的"未解释"新息协方差
    // M = C_gamma - H * P_prop * H' - R
    Eigen::MatrixXd M = C_gamma - H * P_prop * H.transpose() - R;

    // 使用卡尔曼增益将 M 投影到状态空间
    // Q_new = K * M * K'
    Eigen::MatrixXd Q_new = K * M * K.transpose();

    // 4. 更新 Q_ (对角化并保证正定性)
    // 公式 (12): Q_k = diag(max{0, Q_hat(i,i)})
    for (int i = 0; i < state_dim_; ++i) {
      double val = Q_new(i, i);
      if (val < 0)
        val = 0;

      // 可选: 平滑更新还是直接替换？
      // 论文暗示直接替换以获得"自适应"响应。
      // 我们也可以使用遗忘因子，但让我们遵循论文的"估计"风格。
      // 然而，为了避免不稳定性，我们可能想要限制它或混合它。
      // 目前，让我们替换对角元素。

      // 注意: 我们只更新对角元素以保持简单和稳定，正如论文所述

      // 5. 平滑更新 (EMA) 与 最小值约束
      // Q_k = alpha * Q_new + (1 - alpha) * Q_k-1
      double smoothed_val = alpha_ * val + (1.0 - alpha_) * Q_(i, i);

      // 确保不低于最小阈值，防止 Q 消失导致滤波器过拟合模型
      if (smoothed_val < min_q_(i, i)) {
        smoothed_val = min_q_(i, i);
      }

      Q_(i, i) = smoothed_val;
    }

    // 注意: 我们不更新非对角元素以保持独立性假设结构（如果原始 Q 是对角的）。
    // 如果原始 Q 有相关性，这可能会破坏它们。但通常在这些模型中 Q 是对角的。
  }

  // 自适应R估计
  // 理论: C_gamma ~ H * P_pred * H' + R
  // 所以 R ~ C_gamma - H * P_pred * H'
  void estimateAdaptiveR(const Eigen::VectorXd &innovation,
                         const Eigen::MatrixXd &H) {
    if (window_size_ <= 1)
      return;

    // 1. 更新新息缓冲区 (注意：Q和R共用一个buffer，因为都是基于新息)
    // 但为了避免重复添加，我们假设 estimateAdaptiveQ 已经添加了。
    // 或者我们应该分开调用？
    // 为了安全起见，我们在 update() 中只 push 一次，或者让这两个函数都不 push，
    // 而是有一个专门的 updateInnovationBuffer()。
    // 鉴于目前架构，我们在 update() 中手动管理 buffer 更清晰。
    // 这里假设 buffer 已经是最新的。

    if (innovation_buffer_.size() < static_cast<size_t>(window_size_))
      return;

    // 2. 计算 C_gamma
    Eigen::MatrixXd C_gamma = Eigen::MatrixXd::Zero(meas_dim_, meas_dim_);
    for (const auto &gamma : innovation_buffer_) {
      C_gamma += gamma * gamma.transpose();
    }
    C_gamma /= innovation_buffer_.size();

    // 3. 估计 R
    // R_new = C_gamma - H * P_pred * H'
    // 注意：这里的 P_ 是 P_pred (预测协方差)
    Eigen::MatrixXd R_new = C_gamma - H * P_ * H.transpose();

    // 4. 更新 R_ (对角化、正定性、平滑、最小值)
    for (int i = 0; i < meas_dim_; ++i) {
      double val = R_new(i, i);
      if (val < 0)
        val = 0;

      // 平滑更新
      double smoothed_val = r_alpha_ * val + (1.0 - r_alpha_) * R_(i, i);

      // 最小值约束
      if (smoothed_val < min_r_(i, i)) {
        smoothed_val = min_r_(i, i);
      }

      R_(i, i) = smoothed_val;
    }
  }

  // 辅助函数：更新新息缓冲区
  void updateInnovationBuffer(const Eigen::VectorXd &innovation) {
    innovation_buffer_.push_back(innovation);
    if (innovation_buffer_.size() > static_cast<size_t>(window_size_)) {
      innovation_buffer_.pop_front();
    }
  }

  std::shared_ptr<MotionModel> model_; // 运动模型

  Eigen::VectorXd state_; // 状态向量
  Eigen::MatrixXd P_;     // 状态协方差矩阵
  Eigen::MatrixXd Q_;     // 过程噪声协方差
  Eigen::MatrixXd R_;     // 测量噪声协方差

  int state_dim_;       // 状态维度
  int meas_dim_;        // 测量维度
  bool is_initialized_; // 初始化标志

  // Adaptive Q & R
  std::deque<Eigen::VectorXd> innovation_buffer_;
  int window_size_;
  double alpha_;
  double r_alpha_;
  Eigen::MatrixXd min_q_;
  Eigen::MatrixXd min_r_;

  // 协方差上限参数
  bool enable_cov_limit_ = false;  // 是否启用协方差上限限制
  double max_pos_cov_ = 9.0;       // 位置协方差上限 (m²)
  double max_vel_cov_ = 16.0;      // 速度协方差上限 ((m/s)²)
  double max_acc_cov_ = 100.0;     // 加速度协方差上限 ((m/s²)²)

  /**
   * @brief 限制协方差矩阵的对角元素不超过上限
   * 根据状态维度自动判断各状态量的类型（位置/速度/加速度）
   */
  void clampCovariance() {
    if (!enable_cov_limit_) return;
    
    // 根据状态维度判断模型类型并限制协方差
    // CV模型: [x, y, z, vx, vy, vz] dim=6
    // Human CA模型: [x, y, z, vx, vy, ax, ay] dim=7
    // CTRA模型: [x, y, z, v, a, yaw, yaw_rate] dim=7
    // UAV CA模型: [x, y, z, vx, vy, vz, ax, ay, az] dim=9
    
    if (state_dim_ == 6) {
      // CV模型: 位置[0-2], 速度[3-5]
      for (int i = 0; i < 3; ++i) P_(i, i) = std::min(P_(i, i), max_pos_cov_);
      for (int i = 3; i < 6; ++i) P_(i, i) = std::min(P_(i, i), max_vel_cov_);
    } else if (state_dim_ == 7) {
      // CA或CTRA模型: 位置[0-2], 速度/加速度[3-6]
      for (int i = 0; i < 3; ++i) P_(i, i) = std::min(P_(i, i), max_pos_cov_);
      P_(3, 3) = std::min(P_(3, 3), max_vel_cov_);  // vx 或 v
      P_(4, 4) = std::min(P_(4, 4), max_vel_cov_);  // vy 或 a
      P_(5, 5) = std::min(P_(5, 5), max_acc_cov_);  // ax 或 yaw
      P_(6, 6) = std::min(P_(6, 6), max_acc_cov_);  // ay 或 yaw_rate
    } else if (state_dim_ == 9) {
      // UAV CA模型: 位置[0-2], 速度[3-5], 加速度[6-8]
      for (int i = 0; i < 3; ++i) P_(i, i) = std::min(P_(i, i), max_pos_cov_);
      for (int i = 3; i < 6; ++i) P_(i, i) = std::min(P_(i, i), max_vel_cov_);
      for (int i = 6; i < 9; ++i) P_(i, i) = std::min(P_(i, i), max_acc_cov_);
    }
  }
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

    // 限制协方差上限
    clampCovariance();

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

    // --- 自适应 Q & R 估计 ---
    if (window_size_ > 0) {
      updateInnovationBuffer(y); // 统一更新缓冲区

      // 1. 估计 R (基于当前 P_pred)
      estimateAdaptiveR(y, H);

      // 2. 估计 Q (使用更新后的 R_)
      // 初始 K 用于估计 Q (使用旧的 P_ 和 新的 R_)
      Eigen::MatrixXd S_temp = H * P_ * H.transpose() + R_;
      Eigen::MatrixXd K_temp = P_ * H.transpose() * S_temp.inverse();

      Eigen::MatrixXd Q_old = Q_;
      estimateAdaptiveQ(
          y, H, R_,
          K_temp); // 更新 Q_ (注意estimateAdaptiveQ内部不再push buffer)
      P_ = P_ - Q_old + Q_; // 调整 P_pred
    }
    // -----------------------------

    // 新息协方差 (使用最终的 P_)
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;

    // 卡尔曼增益
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // 更新状态
    state_ = state_ + K * y;

    // 更新协方差
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    P_ = (I - K * H) * P_;

    // 限制协方差上限
    clampCovariance();

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

    // 限制协方差上限
    clampCovariance();

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

    // --- 自适应 Q & R 估计 ---
    if (window_size_ > 0) {
      updateInnovationBuffer(y); // 统一更新缓冲区

      // 1. 估计 R (基于当前 P_pred)
      estimateAdaptiveR(y, H);

      // 2. 估计 Q (使用更新后的 R_)
      // 初始 K 用于估计 Q (使用旧的 P_ 和 新的 R_)
      Eigen::MatrixXd S_temp = H * P_ * H.transpose() + R_;
      Eigen::MatrixXd K_temp = P_ * H.transpose() * S_temp.inverse();

      Eigen::MatrixXd Q_old = Q_;
      estimateAdaptiveQ(
          y, H, R_,
          K_temp); // 更新 Q_ (注意estimateAdaptiveQ内部不再push buffer)
      P_ = P_ - Q_old + Q_; // 调整 P_pred
    }
    // -----------------------------

    // 新息协方差 (使用最终的 P_)
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;

    // 卡尔曼增益
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // 更新状态
    state_ = state_ + K * y;

    // 更新协方差 (Joseph form for numerical stability)
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    Eigen::MatrixXd I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();

    // 限制协方差上限
    clampCovariance();

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
    auto kf = std::make_shared<LinearKalmanFilter>(model);
    kf->setAdaptiveParams(params.adaptive_window_size, params.adaptive_alpha,
                          params.adaptive_r_alpha,
                          params.adaptive_min_noise_ratio);
    kf->setCovLimitParams(params.enable_cov_limit, params.max_pos_cov,
                          params.max_vel_cov, params.max_acc_cov);
    return kf;
  } else if (is_uav) {
    // 无人机 -> CA-KF (3D)
    auto model = std::make_shared<CA_Model>(params.ca_uav, true); // 3D
    auto kf = std::make_shared<LinearKalmanFilter>(model);
    kf->setAdaptiveParams(params.adaptive_window_size, params.adaptive_alpha,
                          params.adaptive_r_alpha,
                          params.adaptive_min_noise_ratio);
    kf->setCovLimitParams(params.enable_cov_limit, params.max_pos_cov,
                          params.max_vel_cov, params.max_acc_cov);
    return kf;
  } else if (is_che) {
    // 车 -> CTRA-EKF
    auto model = std::make_shared<CTRA_Model>(params.ctra);
    auto kf = std::make_shared<ExtendedKalmanFilter>(model);
    kf->setAdaptiveParams(params.adaptive_window_size, params.adaptive_alpha,
                          params.adaptive_r_alpha,
                          params.adaptive_min_noise_ratio);
    kf->setCovLimitParams(params.enable_cov_limit, params.max_pos_cov,
                          params.max_vel_cov, params.max_acc_cov);
    return kf;
  } else {
    // 其他 -> CV-KF (3D)
    auto model = std::make_shared<CV_Model>(params.cv);
    auto kf = std::make_shared<LinearKalmanFilter>(model);
    kf->setAdaptiveParams(params.adaptive_window_size, params.adaptive_alpha,
                          params.adaptive_r_alpha,
                          params.adaptive_min_noise_ratio);
    kf->setCovLimitParams(params.enable_cov_limit, params.max_pos_cov,
                          params.max_vel_cov, params.max_acc_cov);
    return kf;
  }
}

} // namespace onboardDetector

#endif // MULTI_MODEL_KALMAN_FILTER_H
