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

CA_Model::CA_Model(bool use_3d) : use_3d_(use_3d) {
    if (use_3d_) {
        // 3D模型 (无人机): [x, y, z, vx, vy, vz, ax, ay, az]
        state_dim_ = 9;
        meas_dim_ = 9;  // 与状态向量相同
    } else {
        // 2D模型 (人): [x, y, vx, vy, ax, ay]
        state_dim_ = 6;
        meas_dim_ = 6;  // 与状态向量相同
    }
}

Eigen::VectorXd CA_Model::getInitState(const Eigen::VectorXd& detection) {
    Eigen::VectorXd state = Eigen::VectorXd::Zero(state_dim_);
    
    if (use_3d_) {
        // 3D: detection = [x, y, z, ...]
        state(0) = detection(0);  // x
        state(1) = detection(1);  // y
        state(2) = detection(2);  // z
        // vx, vy, vz, ax, ay, az 初始化为0
    } else {
        // 2D: detection = [x, y, ...]
        state(0) = detection(0);  // x
        state(1) = detection(1);  // y
        // vx, vy, ax, ay 初始化为0
    }
    
    return state;
}

Eigen::MatrixXd CA_Model::getInitCovP() {
    // 初始协方差矩阵 - 位置确定，速度和加速度不确定
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    
    if (use_3d_) {
        // 3D模型
        P(0, 0) = 0.1;   P(1, 1) = 0.1;   P(2, 2) = 0.1;   // 位置
        P(3, 3) = 1.0;   P(4, 4) = 1.0;   P(5, 5) = 1.0;   // 速度
        P(6, 6) = 10.0;  P(7, 7) = 10.0;  P(8, 8) = 10.0;  // 加速度
    } else {
        // 2D模型
        P(0, 0) = 0.1;   P(1, 1) = 0.1;   // 位置
        P(2, 2) = 1.0;   P(3, 3) = 1.0;   // 速度
        P(4, 4) = 10.0;  P(5, 5) = 10.0;  // 加速度
    }
    
    return P;
}

Eigen::MatrixXd CA_Model::getProcessNoiseQ() {
    // 过程噪声 - 主要在加速度上
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    
    if (use_3d_) {
        Q(0, 0) = 0.1;   Q(1, 1) = 0.1;   Q(2, 2) = 0.1;    // 位置噪声小
        Q(3, 3) = 1.0;   Q(4, 4) = 1.0;   Q(5, 5) = 1.0;    // 速度噪声
        Q(6, 6) = 100.0; Q(7, 7) = 100.0; Q(8, 8) = 100.0; // 加速度噪声大
    } else {
        Q(0, 0) = 0.1;   Q(1, 1) = 0.1;   // 位置噪声小
        Q(2, 2) = 1.0;   Q(3, 3) = 1.0;   // 速度噪声
        Q(4, 4) = 100.0; Q(5, 5) = 100.0; // 加速度噪声大
    }
    
    return Q;
}

Eigen::MatrixXd CA_Model::getMeasNoiseR() {
    // 测量噪声 - 位置、速度、加速度有不同的噪声
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(meas_dim_, meas_dim_);
    
    if (use_3d_) {
        // 3D: [x, y, z, vx, vy, vz, ax, ay, az]
        R(0, 0) = 0.1;  R(1, 1) = 0.1;  R(2, 2) = 0.1;    // 位置
        R(3, 3) = 1.0;  R(4, 4) = 1.0;  R(5, 5) = 1.0;    // 速度
        R(6, 6) = 10.0; R(7, 7) = 10.0; R(8, 8) = 10.0;   // 加速度
    } else {
        // 2D: [x, y, vx, vy, ax, ay]
        R(0, 0) = 0.1;  R(1, 1) = 0.1;   // 位置
        R(2, 2) = 1.0;  R(3, 3) = 1.0;   // 速度
        R(4, 4) = 10.0; R(5, 5) = 10.0;  // 加速度
    }
    
    return R;
}

Eigen::MatrixXd CA_Model::getTransitionF(const Eigen::VectorXd& state) {
    // CA模型的状态转移矩阵
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    double dt = dt_;
    double dt2 = dt * dt / 2.0;
    
    if (use_3d_) {
        // 3D: [x, y, z, vx, vy, vz, ax, ay, az]
        // x_k+1 = x_k + vx*dt + ax*dt^2/2
        F(0, 3) = dt;  F(0, 6) = dt2;  // x <- vx, ax
        F(1, 4) = dt;  F(1, 7) = dt2;  // y <- vy, ay
        F(2, 5) = dt;  F(2, 8) = dt2;  // z <- vz, az
        
        // vx_k+1 = vx_k + ax*dt
        F(3, 6) = dt;  // vx <- ax
        F(4, 7) = dt;  // vy <- ay
        F(5, 8) = dt;  // vz <- az
        
        // ax, ay, az保持不变
    } else {
        // 2D: [x, y, vx, vy, ax, ay]
        F(0, 2) = dt;  F(0, 4) = dt2;  // x <- vx, ax
        F(1, 3) = dt;  F(1, 5) = dt2;  // y <- vy, ay
        
        F(2, 4) = dt;  // vx <- ax
        F(3, 5) = dt;  // vy <- ay
    }
    
    return F;
}

Eigen::MatrixXd CA_Model::getMeasurementH(const Eigen::VectorXd& state) {
    // 观测矩阵 - 观测所有状态
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(meas_dim_, state_dim_);
    return H;
}

Eigen::VectorXd CA_Model::stateToMeasurement(const Eigen::VectorXd& state) {
    // 测量向量与状态向量相同
    return state;
}

// ============================================================================
// CV Model Implementation (3D)
// ============================================================================

CV_Model::CV_Model() {
    // 状态向量: [x, y, z, vx, vy, vz]
    state_dim_ = 6;
    meas_dim_ = 6;  // 与状态向量相同
}

Eigen::VectorXd CV_Model::getInitState(const Eigen::VectorXd& detection) {
    Eigen::VectorXd state = Eigen::VectorXd::Zero(state_dim_);
    
    // detection = [x, y, z, ...]
    state(0) = detection(0);  // x
    state(1) = detection(1);  // y
    state(2) = detection(2);  // z
    // vx, vy, vz 初始化为0
    
    return state;
}

Eigen::MatrixXd CV_Model::getInitCovP() {
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    
    P(0, 0) = 0.1;  P(1, 1) = 0.1;  P(2, 2) = 0.1;  // 位置
    P(3, 3) = 1.0;  P(4, 4) = 1.0;  P(5, 5) = 1.0;  // 速度
    
    return P;
}

Eigen::MatrixXd CV_Model::getProcessNoiseQ() {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    
    Q(0, 0) = 0.1;  Q(1, 1) = 0.1;  Q(2, 2) = 0.1;  // 位置噪声小
    Q(3, 3) = 10.0; Q(4, 4) = 10.0; Q(5, 5) = 10.0; // 速度噪声
    
    return Q;
}

Eigen::MatrixXd CV_Model::getMeasNoiseR() {
    // 测量噪声 - 位置和速度有不同的噪声
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(meas_dim_, meas_dim_);
    
    R(0, 0) = 0.1;  R(1, 1) = 0.1;  R(2, 2) = 0.1;  // 位置
    R(3, 3) = 1.0;  R(4, 4) = 1.0;  R(5, 5) = 1.0;  // 速度
    
    return R;
}

Eigen::MatrixXd CV_Model::getTransitionF(const Eigen::VectorXd& state) {
    // CV模型的状态转移矩阵
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    double dt = dt_;
    
    // x_k+1 = x_k + vx*dt
    F(0, 3) = dt;  // x <- vx
    F(1, 4) = dt;  // y <- vy
    F(2, 5) = dt;  // z <- vz
    
    // vx, vy, vz保持不变
    
    return F;
}

Eigen::MatrixXd CV_Model::getMeasurementH(const Eigen::VectorXd& state) {
    // 观测矩阵 - 观测所有状态
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(meas_dim_, state_dim_);
    return H;
}

Eigen::VectorXd CV_Model::stateToMeasurement(const Eigen::VectorXd& state) {
    // 测量向量与状态向量相同
    return state;
}

// ============================================================================
// CTRA Model Implementation (for vehicles)
// ============================================================================

CTRA_Model::CTRA_Model() {
    // 状态向量: [x, y, v, a, yaw, yaw_rate]
    state_dim_ = 6;
    meas_dim_ = 6;  // 与状态向量相同
}

Eigen::VectorXd CTRA_Model::getInitState(const Eigen::VectorXd& detection) {
    Eigen::VectorXd state = Eigen::VectorXd::Zero(state_dim_);
    
    // detection = [x, y, ...]
    state(0) = detection(0);  // x
    state(1) = detection(1);  // y
    // v, a, yaw, yaw_rate 初始化为0
    
    return state;
}

Eigen::MatrixXd CTRA_Model::getInitCovP() {
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    
    P(0, 0) = 0.1;   // x
    P(1, 1) = 0.1;   // y
    P(2, 2) = 1.0;   // v
    P(3, 3) = 10.0;  // a
    P(4, 4) = 0.5;   // yaw
    P(5, 5) = 1.0;   // yaw_rate
    
    return P;
}

Eigen::MatrixXd CTRA_Model::getProcessNoiseQ() {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    
    Q(0, 0) = 0.1;   // x
    Q(1, 1) = 0.1;   // y
    Q(2, 2) = 1.0;   // v
    Q(3, 3) = 10.0;  // a
    Q(4, 4) = 0.1;   // yaw
    Q(5, 5) = 1.0;   // yaw_rate
    
    return Q;
}

Eigen::MatrixXd CTRA_Model::getMeasNoiseR() {
    // 测量噪声 - 不同状态有不同的噪声
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(meas_dim_, meas_dim_);
    
    R(0, 0) = 0.1;   // x
    R(1, 1) = 0.1;   // y
    R(2, 2) = 1.0;   // v
    R(3, 3) = 10.0;  // a
    R(4, 4) = 0.1;   // yaw
    R(5, 5) = 1.0;   // yaw_rate
    
    return R;
}

Eigen::VectorXd CTRA_Model::stateTransition(const Eigen::VectorXd& state) {
    // CTRA模型的非线性状态转移
    // state = [x, y, v, a, yaw, yaw_rate]
    
    double x = state(0);
    double y = state(1);
    double v = state(2);
    double a = state(3);
    double yaw = state(4);
    double omega = state(5);  // yaw_rate
    
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
        
        next_state(0) = x + omega_inv_sq * (v_next * omega * sin_yaw_next + a * cos_yaw_next 
                                          - v * omega * sin_yaw - a * cos_yaw);
        next_state(1) = y + omega_inv_sq * (-v_next * omega * cos_yaw_next + a * sin_yaw_next 
                                          + v * omega * cos_yaw - a * sin_yaw);
    }
    
    next_state(2) = v_next;
    next_state(3) = a;
    next_state(4) = yaw_next;
    next_state(5) = omega;
    
    return next_state;
}

Eigen::MatrixXd CTRA_Model::getTransitionF(const Eigen::VectorXd& state) {
    // CTRA模型的雅可比矩阵
    // F = d(stateTransition) / d(state)
    
    double x = state(0);
    double y = state(1);
    double v = state(2);
    double a = state(3);
    double yaw = state(4);
    double omega = state(5);
    
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
        
        F(0, 2) = dt * cos_yaw;
        F(0, 3) = 0.5 * dt * dt * cos_yaw;
        F(0, 4) = -displacement * sin_yaw;
        
        F(1, 2) = dt * sin_yaw;
        F(1, 3) = 0.5 * dt * dt * sin_yaw;
        F(1, 4) = displacement * cos_yaw;
    } else {
        // 一般转弯情况
        double omega_inv = 1.0 / omega;
        double omega_inv_sq = omega_inv * omega_inv;
        double omega_inv_cube = omega_inv_sq * omega_inv;
        
        // dx/dv
        F(0, 2) = -omega_inv * (sin_yaw - sin_yaw_next);
        // dx/da
        F(0, 3) = -omega_inv_sq * (cos_yaw - cos_yaw_next) + omega_inv * dt * sin_yaw_next;
        // dx/dyaw
        F(0, 4) = omega_inv_sq * a * (sin_yaw - sin_yaw_next) + omega_inv * (v_next * cos_yaw_next - v * cos_yaw);
        // dx/domega
        F(0, 5) = omega_inv_cube * 2.0 * a * (cos_yaw - cos_yaw_next) 
                + omega_inv_sq * (v * sin_yaw - v_next * sin_yaw_next - 2.0 * a * dt * sin_yaw_next) 
                + omega_inv * dt * v_next * cos_yaw_next;
        
        // dy/dv
        F(1, 2) = omega_inv * (cos_yaw - cos_yaw_next);
        // dy/da
        F(1, 3) = -omega_inv_sq * (sin_yaw - sin_yaw_next) - omega_inv * dt * cos_yaw_next;
        // dy/dyaw
        F(1, 4) = omega_inv_sq * a * (-cos_yaw + cos_yaw_next) + omega_inv * (v_next * sin_yaw_next - v * sin_yaw);
        // dy/domega
        F(1, 5) = omega_inv_cube * 2.0 * a * (sin_yaw - sin_yaw_next) 
                + omega_inv_sq * (v_next * cos_yaw_next - v * cos_yaw + 2.0 * a * dt * cos_yaw_next) 
                + omega_inv * dt * v_next * sin_yaw_next;
    }
    
    // dv/da
    F(2, 3) = dt;
    
    // dyaw/domega
    F(4, 5) = dt;
    
    // a和omega保持不变(已在单位矩阵中)
    
    return F;
}

Eigen::MatrixXd CTRA_Model::getMeasurementH(const Eigen::VectorXd& state) {
    // 观测矩阵 - 观测所有状态
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(meas_dim_, state_dim_);
    return H;
}

Eigen::VectorXd CTRA_Model::stateToMeasurement(const Eigen::VectorXd& state) {
    // 测量向量与状态向量相同
    return state;
}

void CTRA_Model::normalizeYaw(Eigen::VectorXd& state) {
    // 归一化yaw到[-pi, pi]
    state(4) = wrapToPi(state(4));
}

void CTRA_Model::normalizeYawInResidual(Eigen::VectorXd& residual) {
    // 残差中没有yaw (只有位置测量)
    // 不需要处理
}

} // namespace onboardDetector
