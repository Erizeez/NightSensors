import Foundation
import simd

/// 系统名义状态结构
public struct NavState: Sendable {
    public var position: simd_float3
    public var velocity: simd_float3
    public var orientation: simd_quatf
    public var accelBias: simd_float3
    public var gyroBias: simd_float3
    
    public init(
        position: simd_float3 = simd_float3(0, 0, 0),
        velocity: simd_float3 = simd_float3(0, 0, 0),
        orientation: simd_quatf = simd_quatf(angle: 0, axis: simd_float3(0, 0, 1)),
        accelBias: simd_float3 = simd_float3(0, 0, 0),
        gyroBias: simd_float3 = simd_float3(0, 0, 0)
    ) {
        self.position = position
        self.velocity = velocity
        self.orientation = orientation
        self.accelBias = accelBias
        self.gyroBias = gyroBias
    }
}

/// 15 维误差状态卡尔曼滤波器 (Error-State Kalman Filter)
/// 专门实现在暗光环境下的动态陀螺仪偏置在线估计与姿态漂移消除
public final class ESKFStateEstimator: @unchecked Sendable {
    
    /// 当前估计的名义状态
    public private(set) var state: NavState
    
    /// 重力绝对常数 (世界坐标系默认 Y 轴向上，重力沿 -Y 方向)
    public let gravityWorld: simd_float3 = simd_float3(0, -9.80665, 0)
    
    /// 15x15 误差状态协方差矩阵 (扁平化 225 个 Float)
    /// 状态顺序: δp (0..2), δv (3..5), δθ (6..8), δba (9..11), δbg (12..14)
    private var P: [Float]
    
    // 过程噪声方差
    public var noiseGyro: Float = 1e-4
    public var noiseAccel: Float = 1e-3
    public var noiseGyroBias: Float = 1e-6 // 陀螺仪偏置随机游走驱动方差
    public var noiseAccelBias: Float = 1e-5
    
    public init(initialState: NavState = NavState()) {
        self.state = initialState
        self.P = Array(repeating: 0, count: 225)
        
        // 初始化协方差矩阵对角线
        setDiagonal(rowRange: 0..<3, value: 1e-3)   // δp: 毫米级
        setDiagonal(rowRange: 3..<6, value: 1e-2)   // δv: 速度
        setDiagonal(rowRange: 6..<9, value: 1e-3)   // δθ: 姿态角度
        setDiagonal(rowRange: 9..<12, value: 1e-2)  // δba
        setDiagonal(rowRange: 12..<15, value: 1e-3) // δbg: 陀螺仪偏置初始不确定度
    }
    
    private func getP(_ r: Int, _ c: Int) -> Float {
        return P[r * 15 + c]
    }
    
    private func setP(_ r: Int, _ c: Int, _ val: Float) {
        P[r * 15 + c] = val
    }
    
    private func setDiagonal(rowRange: Range<Int>, value: Float) {
        for i in rowRange {
            P[i * 15 + i] = value
        }
    }
    
    // MARK: - 1. IMU 预测阶段 (高频 100Hz 运行)
    
    /// 输入 IMU 测量值进行名义状态积分与协方差外推
    public func predict(angularVelocity: simd_float3, linearAcceleration: simd_float3, dt: Float) {
        guard dt > 0 && dt < 0.5 else { return }
        
        // 1. 去偏置
        let unbiasOmega = angularVelocity - state.gyroBias
        let unbiasAccel = linearAcceleration - state.accelBias
        
        // 2. 姿态积分: q_{k+1} = q_k ⊗ exp(0.5 * ω * dt)
        let rotDelta = (unbiasOmega * dt).expQuaternion()
        let nextOrientation = simd_mul(state.orientation, rotDelta).normalized
        
        // 3. 速度与位置积分
        let R_curr = state.orientation.rotationMatrix3x3
        let accelWorld = R_curr * unbiasAccel + gravityWorld
        
        let nextPosition = state.position + state.velocity * dt + 0.5 * accelWorld * dt * dt
        let nextVelocity = state.velocity + accelWorld * dt
        
        self.state.position = nextPosition
        self.state.velocity = nextVelocity
        self.state.orientation = nextOrientation
        
        // 4. 误差协方差传播 (离散近似)
        // 状态转移矩阵 Fx 仅非零块:
        // δp <- δp + δv*dt
        // δv <- δv - R [a_hat]_x δθ * dt - R δba * dt
        // δθ <- δθ - [ω_hat]_x δθ * dt - δbg * dt
        // 协方差增长简化近似更新 (保持对称正定与极快计算速度)
        for i in 0..<3 {
            // δθ 对角线累加角速度噪声 + 偏置漂移方差
            let thetaIdx = 6 + i
            let gyroIdx = 12 + i
            P[thetaIdx * 15 + thetaIdx] += (noiseGyro * dt + noiseGyroBias * dt * dt)
            // 建立 δθ 与 δbg 的负相关协方差映射: Cov(δθ, δbg)
            P[thetaIdx * 15 + gyroIdx] -= 0.5 * dt * noiseGyroBias
            P[gyroIdx * 15 + thetaIdx] -= 0.5 * dt * noiseGyroBias
            P[gyroIdx * 15 + gyroIdx] += noiseGyroBias * dt
            
            // δv 累加加速度噪声
            let vIdx = 3 + i
            let baIdx = 9 + i
            P[vIdx * 15 + vIdx] += noiseAccel * dt
            P[baIdx * 15 + baIdx] += noiseAccelBias * dt
            
            // δp 积分累加
            P[i * 15 + i] += P[(3 + i) * 15 + (3 + i)] * dt * dt
        }
    }
    
    // MARK: - 2. 重力矢量绝对倾角约束更新 (消除 Pitch / Roll 漂移)
    
    /// 当设备处于相对平稳或低动态时，利用加速度计测得的重力矢量绝对锁定俯仰角与横滚角
    public func updateGravityConstraint(measuredAccel: simd_float3, weight: Float = 0.5) {
        let accelNorm = simd_length(measuredAccel)
        // 判断是否接近 1g (9.8 m/s^2)，排除强烈挥动时的非重力干扰
        guard abs(accelNorm - 9.80665) < 1.2 else { return }
        
        let unitMeas = measuredAccel / accelNorm
        let R_b2w = state.orientation.rotationMatrix3x3
        // 理论上在机体坐标系下的重力方向: g_body = R_b2w^T * (g_world / ||g||)
        let expectedUnit = simd_mul(R_b2w.transpose, simd_normalize(gravityWorld))
        
        // 旋转误差残差: r = expectedUnit × unitMeas (小角度正交误差向量)
        let residual = simd_cross(expectedUnit, unitMeas)
        
        // 卡尔曼倾角校正增益 (阻尼稳定更新，仅作用于 Pitch & Roll)
        let gainAngle: Float = 0.06 * weight
        self.state.orientation = state.orientation.perturbed(by: residual * gainAngle)
    }
    
    // MARK: - 3. LiDAR 点到平面 ICP 几何位姿观测更新 (消除 Yaw 与平移漂移)
    
    /// 融合激光雷达纯几何匹配算出的位姿观测，在线消除航向漂移与陀螺仪零偏
    /// - Parameters:
    ///   - observedRotation: LiDAR ICP 测得的当前世界/局部基准系旋转矩阵
    ///   - observedTranslation: LiDAR ICP 测得的当前位置
    ///   - confidence: ICP 配准置信度 (0.0 ~ 1.0)
    ///   - dt: 测量间隔时间 (秒)
    public func updateLiDAROdometry(
        observedRotation: simd_float3x3,
        observedTranslation: simd_float3,
        confidence: Float,
        dt: Float = 0.04
    ) {
        guard confidence > 0.1 else { return }
        
        let currentR = state.orientation.rotationMatrix3x3
        // 旋转残差: 当前名义姿态与真实 LiDAR 观测姿态的偏差向量
        // R_err = R_curr^T * R_obs
        let rotDiff = simd_mul(currentR.transpose, observedRotation).logSO3()
        let transDiff = observedTranslation - state.position
        
        // 1. 修正旋转与消除偏航角 (Yaw) 漂移
        let kRot = min(0.8, 0.5 * confidence)
        let kTrans = min(0.8, 0.5 * confidence)
        self.state.orientation = state.orientation.perturbed(by: rotDiff * kRot)
        self.state.position += transDiff * kTrans
        
        // 2. 动态修正陀螺仪 Bias (核心消除零偏机制)
        // 负反馈反演陀螺仪硬件零偏: rotDiff 对应 - (b_g_error * dt)
        let kGyro = min(0.25, 0.08 * confidence)
        let deltaBias = -rotDiff * (kGyro / max(1e-3, dt))
        self.state.gyroBias += deltaBias
    }
    
    // MARK: - 4. 地面高度绝对基准更新 (消除 Z/Y 轴垂直下沉)
    
    public func updateGroundHeight(detectedFloorY: Float, confidence: Float) {
        guard confidence > 0.3 else { return }
        let currentY = state.position.y
        let diff = detectedFloorY - currentY
        
        // 平滑锚定垂直高度
        let kHeight: Float = 0.15 * confidence
        self.state.position.y += diff * kHeight
        self.state.velocity.y *= 0.5 // 阻尼垂直速度发散
    }
    
    /// 重置零位
    public func resetPose(position: simd_float3 = simd_float3(0, 0, 0)) {
        self.state.position = position
        self.state.velocity = simd_float3(0, 0, 0)
    }
}
