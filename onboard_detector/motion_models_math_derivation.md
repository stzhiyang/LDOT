# 多模型运动跟踪算法全流程数学详解

本文档详细记录了 `onboard_detector` 模块中使用的三种运动模型（CA、CV、CTRA）的完整数学计算过程，包括初始化、状态预测、协方差预测、观测更新以及所有涉及的矩阵运算。

**格式说明：本文档使用纯文本和 ASCII 格式展示公式，以确保通用可读性。**

---

## 1. 算法总体流程

对于每一个被跟踪的目标，算法执行以下循环步骤：

1.  **初始化 (Initialization)**: 当第一次检测到目标时，根据检测结果构建初始状态向量 `x_0` 和初始协方差矩阵 `P_0`。
2.  **预测 (Prediction)**: 根据上一时刻的状态 `x_{k-1}` 和时间步长 `dt`，预测当前时刻的先验状态 `x_{k|k-1}` 和先验协方差 `P_{k|k-1}`。
3.  **数据关联 (Data Association)**: (本文档不涉及，假设已知测量值 `z_k` 对应当前目标)。
4.  **更新 (Update)**: 利用当前时刻的测量值 `z_k` 修正先验状态，得到后验状态 `x_{k|k}` 和后验协方差 `P_{k|k}`。

---

## 2. 符号定义

- `x`: 状态向量 (State Vector)
- `P`: 状态协方差矩阵 (State Covariance Matrix)
- `F`: 状态转移矩阵 / 雅可比矩阵 (State Transition Matrix / Jacobian)
- `Q`: 过程噪声协方差矩阵 (Process Noise Covariance)
- `z`: 测量向量 (Measurement Vector)
- `H`: 观测矩阵 (Measurement Matrix)
- `R`: 测量噪声协方差矩阵 (Measurement Noise Covariance)
- `K`: 卡尔曼增益 (Kalman Gain)
- `y`: 新息 / 残差 (Innovation / Residual)
- `S`: 新息协方差 (Innovation Covariance)

---

## 3. CV 模型 (Constant Velocity - 恒定速度)

适用于通用物体或未知类别。

### 3.1 初始化
输入检测: `det = [det_x, det_y, det_z]`
- **初始状态 `x_0` (6x1)**:
  ```text
  x_0 = [det_x, det_y, det_z, 0, 0, 0]^T
  ```
  (速度分量初始化为 0)
- **初始协方差 `P_0` (6x6)**:
  对角矩阵，位置方差 0.1，速度方差 1.0。
  ```text
  P_0 = diag(0.1, 0.1, 0.1, 1.0, 1.0, 1.0)
  ```

### 3.2 预测步骤 (Prediction)
状态向量: `x = [px, py, pz, vx, vy, vz]^T`

**1. 状态预测方程**:
```text
px_{k|k-1} = px_{k-1} + vx_{k-1} * dt
py_{k|k-1} = py_{k-1} + vy_{k-1} * dt
pz_{k|k-1} = pz_{k-1} + vz_{k-1} * dt
vx_{k|k-1} = vx_{k-1}
vy_{k|k-1} = vy_{k-1}
vz_{k|k-1} = vz_{k-1}
```

**2. 状态转移矩阵 F**:
```text
F = | 1  0  0  dt 0  0  |
    | 0  1  0  0  dt 0  |
    | 0  0  1  0  0  dt |
    | 0  0  0  1  0  0  |
    | 0  0  0  0  1  0  |
    | 0  0  0  0  0  1  |
```

**3. 过程噪声 Q**:
基于离散白噪声加速模型 (Discrete White Noise Acceleration)。
设加速度标准差为 `sigma` (代码中默认为 1.0)。
对于每个轴 (如 x 轴)，噪声块 `Q_block` (2x2) 为:
```text
q_var = sigma^2
Q_block = q_var * | dt^4/4   dt^3/2 |
                  | dt^3/2   dt^2   |
```
总 `Q` (6x6) 为由三个 `Q_block` 组成的块对角矩阵。

**4. 协方差预测**:
```text
P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q
```

### 3.3 更新步骤 (Update)
测量向量: `z = [meas_x, meas_y, meas_z]^T`

**1. 观测矩阵 H (3x6)**:
```text
H = | 1  0  0  0  0  0 |
    | 0  1  0  0  0  0 |
    | 0  0  1  0  0  0 |
```

**2. 测量噪声 R (3x3)**:
```text
R = diag(0.1, 0.1, 0.1)
```

**3. 卡尔曼更新公式**:
```text
y = z - H * x_{k|k-1}
S = H * P_{k|k-1} * H^T + R
K = P_{k|k-1} * H^T * S^{-1}
x_{k|k} = x_{k|k-1} + K * y
P_{k|k} = (I - K * H) * P_{k|k-1}
```

---

## 4. CA 模型 (Constant Acceleration - 恒定加速度)

适用于行人 (2D) 和无人机 (3D)。

### 4.1 初始化
输入检测: `det = [det_x, det_y, det_z]`
**3D 模式 (无人机):**
- **初始状态 `x_0` (9x1)**:
  ```text
  x_0 = [det_x, det_y, det_z, 0, 0, 0, 0, 0, 0]^T
  ```
- **初始协方差 `P_0` (9x9)**:
  ```text
  P_0 = diag(0.1, 0.1, 0.1,  1.0, 1.0, 1.0,  10.0, 10.0, 10.0)
  ```
  (位置、速度、加速度方差分别为 0.1, 1.0, 10.0)

### 4.2 预测步骤 (Prediction)
以 3D 模式为例。
状态向量: `x = [px, py, pz, vx, vy, vz, ax, ay, az]^T`

**1. 状态预测方程**:
```text
p_{new} = p + v * dt + 0.5 * a * dt^2
v_{new} = v + a * dt
a_{new} = a
```
(对 x, y, z 三轴分别应用)

**2. 状态转移矩阵 F (9x9)**:
```text
F = | I3  I3*dt  I3*0.5*dt^2 |
    | 03  I3     I3*dt       |
    | 03  03     I3          |
```

**3. 过程噪声 Q**:
基于离散白噪声加加速度模型 (Discrete White Noise Jerk)。
设 Jerk 标准差为 `sigma`。
对于每个轴，噪声块 `Q_block` (3x3) 为:
```text
q_var = sigma^2
Q_block = q_var * | dt^5/20  dt^4/8  dt^3/6 |
                  | dt^4/8   dt^3/3  dt^2/2 |
                  | dt^3/6   dt^2/2  dt     |
```
总 `Q` 为块对角矩阵。

**4. 协方差预测**: 同 CV 模型。

### 4.3 更新步骤 (Update)
同 CV 模型，但 `H` 为 3x9 矩阵 (前 3 列为单位阵，其余为 0)。

---

## 5. CTRA 模型 (Constant Turn Rate and Acceleration)

适用于车辆，非线性模型。

### 5.1 初始化
输入检测: `det = [det_x, det_y, det_z]`
- **初始状态 `x_0` (7x1)**:
  ```text
  x_0 = [det_x, det_y, det_z, 0, 0, 0, 0]^T
  ```
  对应: `[x, y, z, v, a, yaw, yaw_rate]`
- **初始协方差 `P_0` (7x7)**:
  ```text
  P_0 = diag(0.1, 0.1, 0.1, 1.0, 10.0, 0.5, 1.0)
  ```

### 5.2 预测步骤 (Prediction) - 扩展卡尔曼滤波 (EKF)

**1. 非线性状态转移函数 `f(x)`**:
输入状态: `x_k = [px, py, pz, v, a, psi, omega]`
中间变量:
```text
v_next = v + a * dt
psi_next = psi + omega * dt
```

**情况 A: `omega` 接近 0 (直线运动)**
```text
dist = v * dt + 0.5 * a * dt^2
px_{k+1} = px + dist * cos(psi)
py_{k+1} = py + dist * sin(psi)
```

**情况 B: `omega` 不为 0 (转弯运动)**
```text
O_inv = 1.0 / omega
O_inv_sq = 1.0 / omega^2

px_{k+1} = px + O_inv_sq * ( v_next*omega*sin(psi_next) + a*cos(psi_next) - 
                             v*omega*sin(psi) - a*cos(psi) )

py_{k+1} = py + O_inv_sq * ( -v_next*omega*cos(psi_next) + a*sin(psi_next) + 
                             v*omega*cos(psi) - a*sin(psi) )
```

其余状态:
```text
pz_{k+1} = pz
v_{k+1}  = v_next
a_{k+1}  = a
psi_{k+1} = psi_next
omega_{k+1} = omega
```

**2. 雅可比矩阵 F (7x7) 计算**:
`F[i, j] = d(x_i)/d(x_j)`。
大部分元素为 0 或 1，主要复杂项在位置 `px (idx 0)` 和 `py (idx 1)` 对 `v, a, psi, omega` 的偏导。

令 `sin_p = sin(psi)`, `cos_p = cos(psi)`, `sin_pn = sin(psi_next)`, `cos_pn = cos(psi_next)`。

**F(0, :) - px 的偏导数:**
- `F(0,3) [dx/dv]` = `-O_inv * (sin_p - sin_pn)`
- `F(0,4) [dx/da]` = `-O_inv_sq * (cos_p - cos_pn) + O_inv * dt * sin_pn`
- `F(0,5) [dx/dpsi]` = `O_inv_sq * a * (sin_p - sin_pn) + O_inv * (v_next * cos_pn - v * cos_p)`
- **`F(0,6) [dx/domega]` (复杂项展开)**:
  ```text
  term1 = O_inv^3 * 2.0 * a * (cos_p - cos_pn)
  term2 = O_inv^2 * (v * sin_p - v_next * sin_pn - a * dt * sin_pn)
  term3 = O_inv * dt * v_next * cos_pn
  F(0,6) = term1 + term2 + term3
  ```

**F(1, :) - py 的偏导数:**
- `F(1,3) [dy/dv]` = `O_inv * (cos_p - cos_pn)`
- `F(1,4) [dy/da]` = `-O_inv_sq * (sin_p - sin_pn) - O_inv * dt * cos_pn`
- `F(1,5) [dy/dpsi]` = `O_inv_sq * a * (-cos_p + cos_pn) + O_inv * (v_next * sin_pn - v * sin_p)`
- **`F(1,6) [dy/domega]` (复杂项展开)**:
  ```text
  term1 = O_inv^3 * 2.0 * a * (sin_p - sin_pn)
  term2 = O_inv^2 * (v_next * cos_pn - v * cos_p + a * dt * cos_pn)
  term3 = O_inv * dt * v_next * sin_pn
  F(1,6) = term1 + term2 + term3
  ```

**其他非零项:**
- `F(3,3)=1`, `F(3,4)=dt`
- `F(5,5)=1`, `F(5,6)=dt`
- `F(2,2)=1`, `F(4,4)=1`, `F(6,6)=1`

**3. 过程噪声 Q**:
```text
Q = diag(0.1, 0.1, 0.01, 1.0, 10.0, 0.1, 1.0)
```

**4. 协方差预测**:
```text
P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q
```

**5. 角度归一化**:
预测后，需将 `psi` 归一化到 `[-pi, pi]`。

### 5.3 更新步骤 (Update)

**1. 测量预测**:
`h(x)` 函数直接提取位置:
```text
z_pred = [px, py, pz]^T
```

**2. 计算新息**:
```text
y = z_meas - z_pred
```
**注意**: 此处无需角度归一化，因为测量值不含角度。

**3. 观测雅可比矩阵 H (3x7)**:
线性观测，H 为常数矩阵:
```text
H = | 1 0 0 0 0 0 0 |
    | 0 1 0 0 0 0 0 |
    | 0 0 1 0 0 0 0 |
```

**4. EKF 更新 (Joseph Form)**:
```text
S = H * P_{k|k-1} * H^T + R
K = P_{k|k-1} * H^T * S^{-1}
x_{k|k} = x_{k|k-1} + K * y
I_KH = (I - K * H)
P_{k|k} = I_KH * P_{k|k-1} * I_KH^T + K * R * K^T
```

**5. 最终角度归一化**:
更新后，再次将状态中的 `psi` 归一化到 `[-pi, pi]`。

---

## 6. 辅助函数

**WrapToPi (角度归一化)**:
```text
function wrapToPi(angle):
    while angle > PI:
        angle -= 2 * PI
    while angle < -PI:
        angle += 2 * PI
    return angle
```
