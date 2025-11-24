/*
    FILE: motionModel.cpp
    ------------------------------
    Implementation of multi-model motion models
*/

#include "motionModel.h"

namespace onboardDetector {

// ============================================================================
// CA Model Implementation (2D for human, 3D for UAV)
// ============================================================================

CA_Model::CA_Model(bool use_3d, double sigma) : use_3d_(use_3d) {
  sigma_ = sigma;
  if (use_3d_) {
    // 3D模型 (无人机): [x, y, z, vx, vy, vz, ax, ay, az]
    state_dim_ = 9;
    meas_dim_ = 3; // 测量: [x, y, z]
  } else {
    // 2D模型 (人): [x, y, z, vx, vy, ax, ay] - 增加z轴
    state_dim_ = 7;
    meas_dim_ = 3; // 测量: [x, y, z]
  }
}

Eigen::VectorXd CA_Model::getInitState(const Eigen::VectorXd &detection) {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(state_dim_);

  // detection = [x, y, z]
  state(0) = detection(0); // x
  state(1) = detection(1); // y
  state(2) = detection(2); // z

  // 速度和加速度初始化为0

  return state;
}

Eigen::MatrixXd CA_Model::getInitCovP() {
  // 初始协方差矩阵
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(state_dim_, state_dim_);

  if (use_3d_) {
    // 3D模型: [x, y, z, vx, vy, vz, ax, ay, az]
    P.diagonal() << 0.1, 0.1, 0.1, // pos
        1.0, 1.0, 1.0,             // vel
        10.0, 10.0, 10.0;          // acc
  } else {
    // 2D模型 (Human): [x, y, z, vx, vy, ax, ay]
    P.diagonal() << 0.1, 0.1, 0.1, // pos
        1.0, 1.0,                  // vel
        10.0, 10.0;                // acc
  }

  return P;
}

Eigen::MatrixXd CA_Model::getProcessNoiseQ() {
  // 过程噪声 - Discrete White Noise Jerk Model
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(state_dim_, state_dim_);

  double dt = dt_;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  double dt5 = dt4 * dt;

  double q_var = sigma_ * sigma_; // jerk variance

  // Q block for one dimension (3x3)
  // [dt^5/20, dt^4/8, dt^3/6]
  // [dt^4/8,  dt^3/3, dt^2/2]
  // [dt^3/6,  dt^2/2, dt    ]

  auto setQBlock = [&](int idx_p, int idx_v, int idx_a) {
    Q(idx_p, idx_p) = dt5 / 20.0 * q_var;
    Q(idx_p, idx_v) = dt4 / 8.0 * q_var;
    Q(idx_p, idx_a) = dt3 / 6.0 * q_var;

    Q(idx_v, idx_p) = dt4 / 8.0 * q_var;
    Q(idx_v, idx_v) = dt3 / 3.0 * q_var;
    Q(idx_v, idx_a) = dt2 / 2.0 * q_var;

    Q(idx_a, idx_p) = dt3 / 6.0 * q_var;
    Q(idx_a, idx_v) = dt2 / 2.0 * q_var;
    Q(idx_a, idx_a) = dt * q_var;
  };

  if (use_3d_) {
    // 3D: [x, y, z, vx, vy, vz, ax, ay, az]
    setQBlock(0, 3, 6); // x
    setQBlock(1, 4, 7); // y
    setQBlock(2, 5, 8); // z
  } else {
    // 2D (Human): [x, y, z, vx, vy, ax, ay]
    setQBlock(0, 3, 5); // x
    setQBlock(1, 4, 6); // y

    // z轴独立噪声 (假设静止或缓慢移动)
    Q(2, 2) = 0.01;
  }

  return Q;
}

Eigen::MatrixXd CA_Model::getMeasNoiseR() {
  // 测量噪声 - [x, y, z]
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(meas_dim_, meas_dim_);

  R(0, 0) = 0.1; // x
  R(1, 1) = 0.1; // y
  R(2, 2) = 0.1; // z

  return R;
}

Eigen::MatrixXd CA_Model::getTransitionF(const Eigen::VectorXd &state) {
  // CA模型的状态转移矩阵
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  double dt = dt_;
  double dt2 = dt * dt / 2.0;

  if (use_3d_) {
    // 3D: [x, y, z, vx, vy, vz, ax, ay, az]
    // x_k+1 = x_k + vx*dt + ax*dt^2/2
    F(0, 3) = dt;
    F(0, 6) = dt2; // x <- vx, ax
    F(1, 4) = dt;
    F(1, 7) = dt2; // y <- vy, ay
    F(2, 5) = dt;
    F(2, 8) = dt2; // z <- vz, az

    // vx_k+1 = vx_k + ax*dt
    F(3, 6) = dt; // vx <- ax
    F(4, 7) = dt; // vy <- ay
    F(5, 8) = dt; // vz <- az
  } else {
    // 2D (Human): [x, y, z, vx, vy, ax, ay]
    // x_k+1 = x_k + vx*dt + ax*dt^2/2
    F(0, 3) = dt;
    F(0, 5) = dt2; // x <- vx, ax
    F(1, 4) = dt;
    F(1, 6) = dt2; // y <- vy, ay

    // z保持不变 (F(2,2)=1)

    // vx_k+1 = vx_k + ax*dt
    F(3, 5) = dt; // vx <- ax
    F(4, 6) = dt; // vy <- ay
  }

  return F;
}

Eigen::MatrixXd CA_Model::getMeasurementH(const Eigen::VectorXd &state) {
  // 观测矩阵 - 只观测位置 [x, y, z]
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(meas_dim_, state_dim_);

  H(0, 0) = 1.0; // x
  H(1, 1) = 1.0; // y
  H(2, 2) = 1.0; // z

  return H;
}

Eigen::VectorXd CA_Model::stateToMeasurement(const Eigen::VectorXd &state) {
  // 提取位置 [x, y, z]
  Eigen::VectorXd meas(meas_dim_);
  meas(0) = state(0);
  meas(1) = state(1);
  meas(2) = state(2);
  return meas;
}

// ============================================================================
// CV Model Implementation (3D)
// ============================================================================

CV_Model::CV_Model(double sigma) {
  sigma_ = sigma;
  // 状态向量: [x, y, z, vx, vy, vz]
  state_dim_ = 6;
  meas_dim_ = 3; // 测量: [x, y, z]
}

Eigen::VectorXd CV_Model::getInitState(const Eigen::VectorXd &detection) {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(state_dim_);

  // detection = [x, y, z]
  state(0) = detection(0); // x
  state(1) = detection(1); // y
  state(2) = detection(2); // z
  // vx, vy, vz 初始化为0

  return state;
}

Eigen::MatrixXd CV_Model::getInitCovP() {
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(state_dim_, state_dim_);

  P.diagonal() << 0.1, 0.1, 0.1, // pos
      1.0, 1.0, 1.0;             // vel

  return P;
}

Eigen::MatrixXd CV_Model::getProcessNoiseQ() {
  // 过程噪声 - Discrete White Noise Acceleration Model
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(state_dim_, state_dim_);

  double dt = dt_;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;

  double q_var = sigma_ * sigma_; // acceleration variance

  // Q block for one dimension (2x2)
  // [dt^4/4, dt^3/2]
  // [dt^3/2, dt^2  ]

  auto setQBlock = [&](int idx_p, int idx_v) {
    Q(idx_p, idx_p) = dt4 / 4.0 * q_var;
    Q(idx_p, idx_v) = dt3 / 2.0 * q_var;

    Q(idx_v, idx_p) = dt3 / 2.0 * q_var;
    Q(idx_v, idx_v) = dt2 * q_var;
  };

  // [x, y, z, vx, vy, vz]
  setQBlock(0, 3); // x
  setQBlock(1, 4); // y
  setQBlock(2, 5); // z

  return Q;
}

Eigen::MatrixXd CV_Model::getMeasNoiseR() {
  // 测量噪声 - [x, y, z]
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(meas_dim_, meas_dim_);

  R(0, 0) = 0.1; // x
  R(1, 1) = 0.1; // y
  R(2, 2) = 0.1; // z

  return R;
}

Eigen::MatrixXd CV_Model::getTransitionF(const Eigen::VectorXd &state) {
  // CV模型的状态转移矩阵
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  double dt = dt_;

  // x_k+1 = x_k + vx*dt
  F(0, 3) = dt; // x <- vx
  F(1, 4) = dt; // y <- vy
  F(2, 5) = dt; // z <- vz

  return F;
}

Eigen::MatrixXd CV_Model::getMeasurementH(const Eigen::VectorXd &state) {
  // 观测矩阵 - 只观测位置 [x, y, z]
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(meas_dim_, state_dim_);

  H(0, 0) = 1.0; // x
  H(1, 1) = 1.0; // y
  H(2, 2) = 1.0; // z

  return H;
}

Eigen::VectorXd CV_Model::stateToMeasurement(const Eigen::VectorXd &state) {
  // 提取位置 [x, y, z]
  Eigen::VectorXd meas(meas_dim_);
  meas(0) = state(0);
  meas(1) = state(1);
  meas(2) = state(2);
  return meas;
}

// ============================================================================
// CTRA Model Implementation (for vehicles)
// ============================================================================

CTRA_Model::CTRA_Model(double sigma) {
  sigma_ = sigma;
  // 状态向量: [x, y, z, v, a, yaw, yaw_rate]
  state_dim_ = 7;
  meas_dim_ = 3; // 测量: [x, y, z]
}

Eigen::VectorXd CTRA_Model::getInitState(const Eigen::VectorXd &detection) {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(state_dim_);

  // detection = [x, y, z]
  state(0) = detection(0); // x
  state(1) = detection(1); // y
  state(2) = detection(2); // z
  // v, a, yaw, yaw_rate 初始化为0

  return state;
}

Eigen::MatrixXd CTRA_Model::getInitCovP() {
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(state_dim_, state_dim_);

  P.diagonal() << 0.1, 0.1, 0.1, // pos
      1.0,                       // v
      10.0,                      // a
      0.5,                       // yaw
      1.0;                       // yaw_rate

  return P;
}

Eigen::MatrixXd CTRA_Model::getProcessNoiseQ() {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(state_dim_, state_dim_);

  // 简单对角噪声，因为CTRA非线性较强，且z轴独立
  Q.diagonal() << 0.1, 0.1, 0.01, // pos (z noise small)
      1.0,                        // v
      10.0,                       // a
      0.1,                        // yaw
      1.0;                        // yaw_rate

  return Q;
}

Eigen::MatrixXd CTRA_Model::getMeasNoiseR() {
  // 测量噪声 - [x, y, z]
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(meas_dim_, meas_dim_);

  R(0, 0) = 0.1; // x
  R(1, 1) = 0.1; // y
  R(2, 2) = 0.1; // z

  return R;
}

Eigen::VectorXd CTRA_Model::stateTransition(const Eigen::VectorXd &state) {
  // CTRA模型的非线性状态转移
  // state = [x, y, z, v, a, yaw, yaw_rate]

  double x = state(0);
  double y = state(1);
  double z = state(2);
  double v = state(3);
  double a = state(4);
  double yaw = state(5);
  double omega = state(6); // yaw_rate

  double dt = dt_;
  double sin_yaw = std::sin(yaw);
  double cos_yaw = std::cos(yaw);

  // 预测下一时刻的状态
  double v_next = v + a * dt;
  double yaw_next = yaw + omega * dt;

  Eigen::VectorXd next_state(state_dim_);

  // 处理小转弯率的情况（近似直线运动）
  if (std::abs(omega) < 0.001) {
    double displacement = v * dt + 0.5 * a * dt * dt;
    next_state(0) = x + displacement * cos_yaw;
    next_state(1) = y + displacement * sin_yaw;
  } else {
    // 一般转弯情况
    double omega_inv = 1.0 / omega;
    double omega_inv_sq = omega_inv * omega_inv;

    double sin_yaw_next = std::sin(yaw_next);
    double cos_yaw_next = std::cos(yaw_next);

    next_state(0) =
        x + omega_inv_sq * (v_next * omega * sin_yaw_next + a * cos_yaw_next -
                            v * omega * sin_yaw - a * cos_yaw);
    next_state(1) =
        y + omega_inv_sq * (-v_next * omega * cos_yaw_next + a * sin_yaw_next +
                            v * omega * cos_yaw - a * sin_yaw);
  }

  next_state(2) = z; // z保持不变
  next_state(3) = v_next;
  next_state(4) = a;
  next_state(5) = yaw_next;
  next_state(6) = omega;

  return next_state;
}

Eigen::MatrixXd CTRA_Model::getTransitionF(const Eigen::VectorXd &state) {
  // CTRA模型的雅可比矩阵
  // F = d(stateTransition) / d(state)

  // state = [x, y, z, v, a, yaw, yaw_rate]
  // indices: 0, 1, 2, 3, 4, 5,   6

  double v = state(3);
  double a = state(4);
  double yaw = state(5);
  double omega = state(6);

  double dt = dt_;
  double sin_yaw = std::sin(yaw);
  double cos_yaw = std::cos(yaw);

  double v_next = v + a * dt;
  double yaw_next = yaw + omega * dt;
  double sin_yaw_next = std::sin(yaw_next);
  double cos_yaw_next = std::cos(yaw_next);

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_dim_, state_dim_);

  // 处理小转弯率的情况
  if (std::abs(omega) < 0.001) {
    double displacement = v * dt + 0.5 * a * dt * dt;

    F(0, 3) = dt * cos_yaw;            // dx/dv
    F(0, 4) = 0.5 * dt * dt * cos_yaw; // dx/da
    F(0, 5) = -displacement * sin_yaw; // dx/dyaw

    F(1, 3) = dt * sin_yaw;            // dy/dv
    F(1, 4) = 0.5 * dt * dt * sin_yaw; // dy/da
    F(1, 5) = displacement * cos_yaw;  // dy/dyaw
  } else {
    // 一般转弯情况
    double omega_inv = 1.0 / omega;
    double omega_inv_sq = omega_inv * omega_inv;
    double omega_inv_cube = omega_inv_sq * omega_inv;

    // dx/dv (idx 3)
    F(0, 3) = -omega_inv * (sin_yaw - sin_yaw_next);
    // dx/da (idx 4)
    F(0, 4) = -omega_inv_sq * (cos_yaw - cos_yaw_next) +
              omega_inv * dt * sin_yaw_next;
    // dx/dyaw (idx 5)
    F(0, 5) = omega_inv_sq * a * (sin_yaw - sin_yaw_next) +
              omega_inv * (v_next * cos_yaw_next - v * cos_yaw);
    // dx/domega (idx 6)
    // FIXED: corrected the coefficient of a*dt*sin_yaw_next from 2.0 to 1.0
    F(0, 6) = omega_inv_cube * 2.0 * a * (cos_yaw - cos_yaw_next) +
              omega_inv_sq * (v * sin_yaw - v_next * sin_yaw_next -
                              a * dt * sin_yaw_next) +
              omega_inv * dt * v_next * cos_yaw_next;

    // dy/dv (idx 3)
    F(1, 3) = omega_inv * (cos_yaw - cos_yaw_next);
    // dy/da (idx 4)
    F(1, 4) = -omega_inv_sq * (sin_yaw - sin_yaw_next) -
              omega_inv * dt * cos_yaw_next;
    // dy/dyaw (idx 5)
    F(1, 5) = omega_inv_sq * a * (-cos_yaw + cos_yaw_next) +
              omega_inv * (v_next * sin_yaw_next - v * sin_yaw);
    // dy/domega (idx 6)
    // FIXED: corrected the coefficient of a*dt*cos_yaw_next from 2.0 to 1.0
    F(1, 6) = omega_inv_cube * 2.0 * a * (sin_yaw - sin_yaw_next) +
              omega_inv_sq * (v_next * cos_yaw_next - v * cos_yaw +
                              a * dt * cos_yaw_next) +
              omega_inv * dt * v_next * sin_yaw_next;
  }

  // dv/da
  F(3, 4) = dt;

  // dyaw/domega
  F(5, 6) = dt;

  // z, a, omega保持不变(已在单位矩阵中)

  return F;
}

Eigen::MatrixXd CTRA_Model::getMeasurementH(const Eigen::VectorXd &state) {
  // 观测矩阵 - 只观测位置 [x, y, z]
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(meas_dim_, state_dim_);

  H(0, 0) = 1.0; // x
  H(1, 1) = 1.0; // y
  H(2, 2) = 1.0; // z

  return H;
}

Eigen::VectorXd CTRA_Model::stateToMeasurement(const Eigen::VectorXd &state) {
  // 提取位置 [x, y, z]
  Eigen::VectorXd meas(meas_dim_);
  meas(0) = state(0);
  meas(1) = state(1);
  meas(2) = state(2);
  return meas;
}

void CTRA_Model::normalizeYaw(Eigen::VectorXd &state) {
  // 归一化yaw到[-pi, pi], yaw index = 5
  state(5) = wrapToPi(state(5));
}

void CTRA_Model::normalizeYawInResidual(Eigen::VectorXd &residual) {
  // CTRA模型测量向量: [x, y, z]
  // 测量向量中没有角度，不需要归一化
}

} // namespace onboardDetector
