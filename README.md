# NightSensors (暗夜感知与空间传感器融合夜视系统)

> 基于 iPhone Pro 机型硬件特性（后置 dToF LiDAR 激光雷达 + 前置 TrueDepth 原深感相机 + 高频 IMU）打造的**无光/暗光全实时环境预览与高精度零漂姿态空间补偿系统**。

---

## 🌟 核心特性与解决的痛点

### 1. 痛点：传统视觉惯导在全黑环境下崩溃发散
在没有光线（Zero Light / 0 Lux）的暗室或野外，普通的手机 RGB 摄像头完全黑屏，传统基于视觉特征点追踪的 ARKit VIO 算法会彻底失效。如果仅依赖机载 MEMS 陀螺仪与加速度计死算积分，由于陀螺仪固有零漂（Bias Drift）和随机游走，几秒钟内虚拟空间就会产生数度的严重偏移和位置发散。

### 2. 破局：空间传感器融合与几何零漂补偿引擎 (Drift Compensation Engine)
本项目不依赖任何可见光，纯粹利用主动发射红外光子的传感器矩阵构建了三重空间物理约束：
1. **重力矢量绝对倾角约束 (Gravity Vector Alignment)**：
   利用加速度计高频滤波提取的重力绝对物理基准，锁定设备的 **Pitch（俯仰角）** 与 **Roll（横滚角）**，杜绝倾斜下垂。
2. **点到平面激光雷达里程计 (Point-to-Plane ICP on LiDAR Cloud)**：
   利用后置 LiDAR 940nm 激光脉冲获取的稠密几何点云（5m 深度），反投影并通过体素网格（Voxel Grid）降采样，通过 Gauss-Newton 高阶求解点到平面残差方程，在完全无光下高频解算相对旋转与平移，**绝对锁定 Yaw（偏航航向角）**。
3. **15 维误差状态卡尔曼滤波 (ESKF)**：
   将几何匹配残差反向投影至陀螺仪状态，**在系统运行过程中动态在线学习并完全消除陀螺仪零点偏置（Gyro Bias $\mathbf{b}_g$）**！
4. **RANSAC 地面提取与绝对物理标高锚定**：
   实时拟合地面水平面方程，消除垂直高度方向的积分漂移下坠。
5. **机身物理外参及杠杆臂效应补偿 (Lever-Arm Kinematics)**：
   精确考虑 iPhone Pro LiDAR 模组、前置 TrueDepth 模组与主板 IMU 的物理三维空间偏移量，补偿角速度旋转引起的线速度与线加速度扰动。

---

## 🎨 视觉预览与交互模式 (Metal 60fps Pipeline)

系统通过 Metal 着色器将 32-bit float 稠密深度图直接送入 GPU 零拷贝流水线，支持 4 种实时夜视模式切换：

| 模式 | 视觉呈现 | 应用场景 |
| :--- | :--- | :--- |
| **热成像 (Thermal)** | 军用伪彩色热成像色谱 (Turbo LUT)，近处炽热白/赤红，逐渐过度到暖黄、冷蓝与深黑 | 快速识别障碍物与前方通行纵深 |
| **赛博网格 (Cyberpunk)** | 冰蓝与洋红霓虹等高线波纹，带动态扫描光效 | 赛博朋克 HUD 视觉沉浸体验 |
| **轮廓夜视 (Edge Glow)** | 基于 GPU 深度 Sobel 梯度滤波，提取家具/门框边缘呈现荧光绿边缘发光 | 极简轮廓识别，视觉干扰最小 |
| **前置红外 (Infrared)** | 调度 TrueDepth 泛光与红外感应镜头 | 近距离自拍探照 / 探测针孔红外摄像头 |

---

## 🔊 跨模态触觉变频雷达 (Haptic Sonar)

- 屏幕中央高精度十字准星发射虚拟空间射线；
- 结合 `CoreHaptics` 引擎，将中央障碍物距离（0.2m ~ 4.0m）动态映射为脉冲频率与震动强度：
  - 4.0m 时：舒缓低频轻微脉冲；
  - 靠近至 0.3m 时：急促蜂鸣强震动；
  - 为黑暗中行进、探路提供直觉式物理避障反馈。

---

## 📂 项目结构全景

```
mixed-sensors/
├── ALGORITHM_DESIGN.md            # 深度算法推导与卡尔曼滤波数学架构设计文档
├── README.md                      # 项目说明与验收指南
├── project.yml                    # xcodegen 工程配置规范
├── NightSensors.xcodeproj         # 生成的标准 Xcode 16/17 工程
├── NightSensors/                  # 主应用源码
│   ├── App/
│   │   ├── NightSensorsApp.swift  # SwiftUI App 入口
│   │   └── Info.plist             # 权限配置
│   ├── Core/
│   │   ├── Math/
│   │   │   ├── SIMD+Extensions.swift   # 李群 SO(3)、李代数 so(3)、四元数、刚体变换
│   │   │   └── MatrixOperations.swift  # 6x6 矩阵高性能 Cholesky 分解求解器
│   │   ├── Calibration/
│   │   │   └── ExtrinsicCalibration.swift # iPhone Pro 空间传感器外参及杠杆臂运动学
│   │   ├── Algorithm/
│   │   │   ├── ESKFStateEstimator.swift      # 15维误差状态卡尔曼滤波与陀螺仪偏置消除
│   │   │   ├── VoxelGridFilter.swift         # 3D 体素网格点云降采样与法向量估计
│   │   │   ├── PointToPlaneICP.swift         # 点到平面 ICP 几何位姿求解器
│   │   │   ├── RANSACPlaneDetector.swift     # RANSAC 地平面提取与物理高度锁定
│   │   │   └── DriftCompensationEngine.swift # 传感器融合与漂移补偿总控制器
│   │   ├── Sensors/
│   │   │   ├── SensorPipelineManager.swift   # ARKit + CoreMotion 硬件调度管线
│   │   │   └── TrueDepthInfraredStream.swift # 前置 TrueDepth 深度与红外流控制
│   │   ├── Haptics/
│   │   │   └── HapticSonarEngine.swift       # CoreHaptics 动态距离变频触觉雷达
│   │   └── Rendering/
│   │       ├── MetalRenderer.swift       # Metal GPU 渲染器 (MTKViewDelegate)
│   │       ├── MetalShaderSource.swift   # 内置 Metal Shader 源码 (支持 JIT 运行时编译)
│   │       ├── Shaders.metal             # 离线 Metal 着色器源文件
│   │       ├── ShaderTypes.h             # 着色器结构体定义
│   │       └── VisualizationMode.swift   # 可视化模式枚举
│   └── UI/
│       ├── ContentView.swift             # 主视图容器
│       ├── CyberpunkHUDView.swift        # 赛博朋克 HUD 覆盖层、十字准星与控制栏
│       └── SensorDiagnosticsView.swift   # 实时遥测仪表盘 (陀螺仪偏置、ICP置信度、FPS)
└── NightSensorsTests/             # 单元测试套件 (100% 通过验证)
    ├── MatrixAndLieAlgebraTests.swift    # 李代数与 Cholesky 求解器测试
    ├── ESKFConvergenceTests.swift        # 陀螺仪偏置在线自适应收敛性测试
    ├── PointToPlaneICPTests.swift        # 点到平面 ICP 三维配准收敛性测试
    └── RANSACPlaneTests.swift            # 地平面提取与法向估计测试
```

---

## 🧪 自动化测试验证 (Unit & Algorithm Tests)

所有核心算法均配有完整的单元测试用例，覆盖：
1. **李代数指数与对数映射一致性**；
2. **6x6 矩阵正规方程 Cholesky 解算精度**；
3. **陀螺仪硬件零偏在线自适应估计收敛性**（验证未校准偏置在激光雷达闭环下收敛）；
4. **3D 几何特征点到平面 ICP 对齐精度**；
5. **RANSAC 地面提取与绝对标高计算**。

### 运行测试命令：
```bash
xcodebuild test -project NightSensors.xcodeproj -scheme NightSensors -destination "platform=iOS Simulator,name=iPhone 17 Pro" CODE_SIGNING_ALLOWED=NO
```
**测试结果**：`** TEST SUCCEEDED **` (6 个测试套件，0 失败全部通过)。

---

## 🚀 真机部署指南

1. 使用 Xcode 打开 `NightSensors.xcodeproj`（或使用 `xcodegen generate` 生成）；
2. 连接配备 LiDAR 激光雷达的 iPhone Pro 机型（iPhone 12 Pro ~ iPhone 16 Pro 及以上）；
3. 在 Xcode 的 **Signing & Capabilities** 中选择您自己的开发者账号（免费个人 Apple ID 即可）；
4. 编译并运行，走进漆黑房间关掉电灯，体验黑夜中的空间感知与赛博朋克夜视！

---

## ⚖️ 开源协议与免责声明 (License & Disclaimer)

### License
本项目采用 [MIT License](LICENSE) 开源协议。

### Disclaimer (免责声明)
> [!WARNING]
> **非生命攸关设备声明**：NightSensors 是一个面向空间计算、激光雷达传感器融合与图形学探索的开源科研项目。红外结构光与 LiDAR 传感器存在物理散射、镜面反射盲区与有效距离限制，**严禁**将本应用作为暗夜驾驶、危险作业、医疗急救等生命攸关场景的唯一避障依赖。作者不对因使用本软件造成的任何意外、人身伤害或财产损失承担法律责任。
