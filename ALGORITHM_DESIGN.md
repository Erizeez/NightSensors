# NightSensors: 暗光/全黑环境下的多传感器融合与空间几何漂移补偿系统

## 1. 系统概述与背景痛点

在传统的 iOS 增强现实与空间感知方案中（如标准 ARKit VIO 视觉惯性里程计），系统高度依赖广角可见光（RGB）摄像头捕获的图像特征点（角点、光流）。
**核心痛点**：在黑夜、停电、野外或完全无光（Zero Light）的极限环境下，RGB 摄像头输出几乎纯黑的无效图像（信噪比极低），导致 VIO 视觉追踪迅速失效，系统退化为纯惯导（Pure IMU Dead Reckoning）。
而 iPhone 上的 MEMS 陀螺仪与加速度计存在固有的**零点偏置漂移（Gyroscope Bias Drift）**与二阶积分随机游走。在完全失去视觉纠偏的情况下，只需数秒，姿态与空间位置就会产生不可逆的严重漂移（Drift & Divergence），导致虚拟网格、点云在视野中急速滑移或倾覆。

针对此痛点，本项目设计并实现了一套**基于 iPhone Pro 硬件特性的纯硬件级空间传感器融合与几何自适应漂移补偿引擎**。

---

## 2. 核心传感器物理特性与空间布局

### 2.1 硬件传感器矩阵
1. **后置 dToF LiDAR 激光雷达扫描仪**
   - **物理原理**：发射 940nm 垂直腔面发射激光（VCSEL）红外脉冲，计算光子往返飞行时间。
   - **环境无关性**：完全不依赖任何环境可见光，在 $0 \text{ Lux}$ 绝对全黑环境下性能不变。
   - **测量能力**：提供最高 60Hz、最远 5.0m 的稠密深度图（`sceneDepth`, 256×192 32-bit float）及实时 3D 几何拓扑网格（`ARMeshAnchor`）。
2. **前置 TrueDepth 原深感模组**
   - **物理原理**：点阵投影器投射约 30,000 个红外散斑，红外泛光感应元件补光，配合红外感光镜头成像。
   - **环境无关性**：主动发射红外光，近距离（0.2m ~ 1.2m）具备亚毫米级深度感知能力，并可读取原始单通道红外强度图（Infrared Luminance）。
3. **机载 6-DoF 高频 IMU（加速度计 + 陀螺仪）**
   - 采样率 100Hz+，输出三轴角速度 $\boldsymbol{\omega}_m$ 与三轴线加速度 $\mathbf{a}_m$。

### 2.2 空间外参矩阵与杠杆臂效应（Lever-Arm Effect）
机载各传感器在机身刚体结构中存在固定物理位姿差。设 IMU 中心坐标系为机体基准系 $\{B\}$，LiDAR 坐标系为 $\{L\}$，前置 TrueDepth 为 $\{T\}$：
- 坐标变换：
  $$\mathbf{p}_B = \mathbf{R}_{B L} \mathbf{p}_L + \mathbf{t}_{B L}$$
- 杠杆臂线速度补偿：
  当设备绕 IMU 旋转时，LiDAR 观测到的线速度存在旋转感应分量：
  $$\mathbf{v}_L = \mathbf{R}_{B L}^T (\mathbf{v}_B + \boldsymbol{\omega}_B \times \mathbf{t}_{B L})$$

系统内置标准 iPhone Pro 模组几何标定参数，确保多传感器融合在物理空间的一致性。

---

## 3. 核心算法设计：误差状态卡尔曼滤波与几何闭环 (ESKF + LiDAR Odometry)

系统采用紧耦合的 **误差状态卡尔曼滤波器 (Error-State Kalman Filter, ESKF)** 结合 **点到平面 ICP (Point-to-Plane ICP)** 与 **重力绝对倾角约束**，在线动态估计并完全消除陀螺仪偏置漂移。

```
                       ┌─────────────────────────┐
                       │   High-Rate IMU (100Hz) │
                       │    (Accel + Gyroscope)  │
                       └────────────┬────────────┘
                                    │
                         Kinematic Propagation
                                    ▼
                       ┌─────────────────────────┐
                       │  ESKF Nominal State &   │
                       │   Error Covariance (P)  │
                       └────────────┬────────────┘
                                    │
           ┌────────────────────────┴────────────────────────┐
           │                                                 │
           ▼ (30-60Hz)                                       ▼ (100Hz)
┌─────────────────────────────────┐               ┌─────────────────────────────────┐
│ LiDAR Active Depth Point Cloud  │               │ Gravitational Vector Constraint │
│  - Voxel Grid Downsampling      │               │  - Low-Pass Acceleration Filter │
│  - Surface Normal Estimation    │               │  - Gravity-Alignment (Pitch/Roll)│
│  - Point-to-Plane Gauss-Newton  │               └────────────────┬────────────────┘
│  - RANSAC Ground Floor Locking  │                                │
└────────────────┬────────────────┘                                │
                 │                                                 │
                 └────────────────────────┬────────────────────────┘
                                          ▼
                               ┌─────────────────────┐
                               │ Kalman Gain & Error │
                               │ State Measurement   │
                               │ Update [r = z - h]  │
                               └──────────┬──────────┘
                                          │
                  ┌───────────────────────┴───────────────────────┐
                  ▼                                               ▼
     ┌────────────────────────┐                     ┌───────────────────────────┐
     │ Corrected 6-DoF Pose   │                     │ Online Gyroscope Bias     │
     │ (Drift-Free In Darkness)│                    │ Compensation (b_g -> 0)   │
     └────────────────────────┘                     └───────────────────────────┘
```

### 3.1 状态向量定义
系统状态分为真值状态 $\mathbf{x}$、名义状态 $\hat{\mathbf{x}}$ 与误差状态 $\delta \mathbf{x}$：
$$\delta \mathbf{x} = [\delta \mathbf{p}, \delta \mathbf{v}, \delta \boldsymbol{\theta}, \delta \mathbf{b}_a, \delta \mathbf{b}_g]^T \in \mathbb{R}^{15}$$
- $\delta \mathbf{p} \in \mathbb{R}^3$：位置误差
- $\delta \mathbf{v} \in \mathbb{R}^3$：速度误差
- $\delta \boldsymbol{\theta} \in \mathbb{R}^3$：旋转姿态微小误差向量（李代数 $\mathfrak{so}(3)$ 表达，满足四元数乘法 $\mathbf{q} \approx \hat{\mathbf{q}} \otimes [1, \frac{1}{2}\delta \boldsymbol{\theta}^T]^T$）
- $\delta \mathbf{b}_a \in \mathbb{R}^3$：加速度计偏置误差
- $\delta \mathbf{b}_g \in \mathbb{R}^3$：**陀螺仪零点漂移偏置误差（核心估计量）**

### 3.2 IMU 高频状态名义传播与协方差积分
给定陀螺仪读数 $\boldsymbol{\omega}_m$ 与加速度计读数 $\mathbf{a}_m$：
1. 偏置校正：
   $$\hat{\boldsymbol{\omega}} = \boldsymbol{\omega}_m - \hat{\mathbf{b}}_g, \quad \hat{\mathbf{a}} = \mathbf{a}_m - \hat{\mathbf{b}}_a$$
2. 名义位姿积分（采用四阶 Runge-Kutta 或中点法）：
   $$\hat{\mathbf{q}}_{k+1} = \hat{\mathbf{q}}_k \otimes \exp\left(\frac{1}{2} \hat{\boldsymbol{\omega}} \Delta t\right)$$
   $$\hat{\mathbf{v}}_{k+1} = \hat{\mathbf{v}}_k + (\mathbf{R}(\hat{\mathbf{q}}_k) \hat{\mathbf{a}} + \mathbf{g}) \Delta t$$
   $$\hat{\mathbf{p}}_{k+1} = \hat{\mathbf{p}}_k + \hat{\mathbf{v}}_k \Delta t + \frac{1}{2} (\mathbf{R}(\hat{\mathbf{q}}_k) \hat{\mathbf{a}} + \mathbf{g}) \Delta t^2$$
3. 误差协方差传播：
   $$\mathbf{P}_{k+1} = \mathbf{F}_x \mathbf{P}_k \mathbf{F}_x^T + \mathbf{F}_w \mathbf{Q} \mathbf{F}_w^T$$
   其中 $\mathbf{F}_x$ 为离散状态转移雅可比矩阵，将陀螺仪漂移对旋转误差的影响耦合至协方差传播中。

### 3.3 LiDAR 几何特征迭代对齐 (Point-to-Plane ICP)
当捕获新一帧深度点云时，首先反投影并经过三维体素网格（Voxel Grid）降采样，然后通过邻域主成分分析（PCA）求解表面法向量 $\mathbf{n}_i$。
- **目标函数**（点到平面距离最小化）：
  $$J(\Delta \mathbf{T}) = \sum_{i=1}^N \left( \left( \exp([\delta \boldsymbol{\phi}]_\times) \mathbf{R} \mathbf{p}_i + \mathbf{t} + \delta \mathbf{t} - \mathbf{q}_i \right) \cdot \mathbf{n}_i \right)^2$$
- 对微小扰动 $[\delta \boldsymbol{\phi}, \delta \mathbf{t}]^T \in \mathbb{R}^6$ 进行一阶泰勒展开，得到线性方程组：
  $$\mathbf{H} \begin{bmatrix} \delta \boldsymbol{\phi} \\ \delta \mathbf{t} \end{bmatrix} = -\mathbf{b}$$
  其中：
  $$\mathbf{J}_i = [ -(\mathbf{R} \mathbf{p}_i \times \mathbf{n}_i)^T, \ \mathbf{n}_i^T ] \in \mathbb{R}^{1 \times 6}$$
  $$\mathbf{H} = \sum_{i} \mathbf{J}_i^T \mathbf{J}_i, \quad \mathbf{b} = \sum_{i} \mathbf{J}_i^T e_i$$
  通过快速 Cholesky 分解求解 $\delta \boldsymbol{\xi}$ 并迭代 3~5 次收敛，直接测出机身在无光环境下的真实空间旋转与平移增量 $(\Delta \mathbf{R}_{L}, \Delta \mathbf{t}_{L})$。

### 3.4 绝对重力与水平地平面约束 (Gravity & Ground Plane Anchor)
- **重力倾角绝对校正**：
  加速度计低通滤波后的稳态输出为绝对重力矢量在机体系下的投影：
  $$\mathbf{z}_g = \frac{\mathbf{a}_{\text{lowpass}}}{\|\mathbf{a}_{\text{lowpass}}\|} \approx \mathbf{R}^T \frac{\mathbf{g}}{\|\mathbf{g}\|}$$
  构造倾角测量残差，直接作为卡尔曼滤波观测，彻底锁定 **Pitch（俯仰角）** 和 **Roll（横滚角）**，切断这两个维度的陀螺仪漂移。
- **RANSAC 地面提取与高度锚定**：
  在点云中寻找法向量平行于重力的大平面，拟合方程 $A x + B y + C z + D = 0$。地面距离 $D$ 给出机身相对地面的物理绝对标高，彻底消除垂直方向的漂移下沉。

### 3.5 观测更新与陀螺仪偏置在线自校正
将点到平面 ICP 解算出的位姿增量与重力矢量构成测量量 $\mathbf{z}$：
1. 计算卡尔曼增益：
   $$\mathbf{K} = \mathbf{P} \mathbf{H}_m^T (\mathbf{H}_m \mathbf{P} \mathbf{H}_m^T + \mathbf{R}_m)^{-1}$$
2. 更新误差状态：
   $$\delta \mathbf{x} = \mathbf{K} (\mathbf{z} - h(\hat{\mathbf{x}}))$$
3. **注入名义状态并重置误差**：
   - 修正名义位姿 $\hat{\mathbf{q}} \leftarrow \hat{\mathbf{q}} \otimes \exp(\frac{1}{2}\delta \boldsymbol{\theta})$
   - **动态更新陀螺仪偏置**：$\hat{\mathbf{b}}_g \leftarrow \hat{\mathbf{b}}_g + \delta \mathbf{b}_g$
   - 协方差重置：$\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K} \mathbf{H}_m) \mathbf{P}$

通过这种闭环反馈，陀螺仪的物理零漂在几秒钟的连续运行中被迅速识别并从源头扣除，无论环境多黑，AR 坐标系始终坚如磐石！

---

## 4. 图形渲染管线 (Metal 60fps Pipeline)

为避免 CPU 与 GPU 之间的大内存带宽拷贝，整个渲染管线基于 **Metal 零拷贝共享内存 (MTLStorageModeShared)** 构建：
1. **深度图直通 GPU**：`CVPixelBuffer` 深度纹理直接绑定到 `MTLRenderCommandEncoder`。
2. **多模式 Shader 矩阵**：
   - **Thermal (军用伪彩色热成像)**：根据距离自动映射到冷暖色相谱（Turbo / Jet LUT）。
   - **Cyberpunk Wireframe (赛博霓虹网格)**：动态读取 `ARMeshGeometry` 顶点与索引缓冲区，执行边缘渐变光效渲染。
   - **Edge Glow (深度轮廓发光)**：在片段着色器中执行实时 Sobel 深度梯度算子，突出障碍物轮廓。
   - **PointCloud Particles (3D 粒子流)**：根据逆投影矩阵将深度像素生成 3D 浮动粒子。

---

## 5. 跨模态触觉变频雷达 (Haptic Sonar)

系统通过 `CoreHaptics` 构建了多频段触觉引擎：
- 屏幕中央十字准星射出虚空间射线（Raycast），与深度网格求交获得前向障碍物距离 $d$。
- 触觉发生器映射函数：
  $$\text{Frequency} = f_{\min} + (1.0 - \frac{d}{d_{\max}}) \cdot (f_{\max} - f_{\min})$$
  $$\text{Intensity} = \left(1.0 - \frac{d}{d_{\max}}\right)^2$$
- 距离从 5.0m 递减至 0.3m 时，震动从低频舒缓脉冲迅速演变为急促的高强度蜂鸣触感，提供精准的盲探避障辅助。
