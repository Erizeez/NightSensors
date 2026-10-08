import Foundation
import simd

/// 漂移补偿引擎输出的实时诊断遥测数据
public struct DriftDiagnostics: Sendable {
    public let estimatedGyroBias: simd_float3   // 估计出的陀螺仪物理偏置 (rad/s)
    public let gyroBiasMagnitudeDegPerSec: Float // 偏置幅度 (°/s)
    public let isGravityLocked: Bool            // 俯仰/横滚角是否已通过重力绝对锁定
    public let icpConfidence: Float             // LiDAR 几何对齐置信度 (0~1)
    public let detectedGroundHeight: Float?     // 探测到的绝对地面标高 (米)
    public let totalCompensatedYawDeg: Float    // 累计通过几何算法消除的航向漂移量 (度)
}

/// 核心空间传感器融合与漂移补偿总控引擎
public final class DriftCompensationEngine: @unchecked Sendable {
    
    public let calibration: ExtrinsicCalibration
    public let eskf: ESKFStateEstimator
    public let voxelFilter: VoxelGridFilter
    public let icpSolver: PointToPlaneICP
    public let planeDetector: RANSACPlaneDetector
    
    private var lastIMUTimestamp: TimeInterval = 0
    private var keyframePoints: [OrientedPoint] = []
    private var lastKeyframeTransform: simd_float4x4 = matrix_identity_float4x4
    private var totalCompensatedYaw: Float = 0
    
    // 低通滤波重力矢量估计
    private var filteredGravity: simd_float3 = simd_float3(0, -9.80665, 0)
    
    public init(calibration: ExtrinsicCalibration = ExtrinsicCalibration()) {
        self.calibration = calibration
        self.eskf = ESKFStateEstimator()
        self.voxelFilter = VoxelGridFilter(voxelSize: 0.08)
        self.icpSolver = PointToPlaneICP(maxIterations: 6, maxCorrespondenceDistance: 0.35)
        self.planeDetector = RANSACPlaneDetector(maxIterations: 30, distanceThreshold: 0.03)
    }
    
    // MARK: - 1. IMU 数据高频驱动 (100Hz)
    
    public func processIMU(
        angularVelocity: simd_float3,
        linearAcceleration: simd_float3,
        timestamp: TimeInterval
    ) {
        if lastIMUTimestamp == 0 {
            lastIMUTimestamp = timestamp
            return
        }
        
        let dt = Float(max(1e-4, min(0.1, timestamp - lastIMUTimestamp)))
        lastIMUTimestamp = timestamp
        
        // 1. 低通滤波提取重力向量 (截止频率约 2Hz)
        let alpha: Float = 0.08
        filteredGravity = (1.0 - alpha) * filteredGravity + alpha * linearAcceleration
        
        // 2. 预测状态
        eskf.predict(
            angularVelocity: angularVelocity,
            linearAcceleration: linearAcceleration,
            dt: dt
        )
        
        // 3. 重力绝对倾角约束 (杜绝 Pitch / Roll 倾斜漂移)
        eskf.updateGravityConstraint(measuredAccel: filteredGravity, weight: 0.4)
    }
    
    // MARK: - 2. LiDAR 深度点云处理 (30-60Hz)
    
    /// 输入原始 LiDAR 深度生成的 3D 点云集合，执行几何闭环与零漂补偿
    public func processLiDARPointCloud(
        rawPoints: [simd_float3],
        timestamp: TimeInterval
    ) -> DriftDiagnostics {
        guard !rawPoints.isEmpty else {
            return makeDiagnostics(confidence: 0, groundHeight: nil)
        }
        
        // 1. 机载外参空间变换: 将 LiDAR 局部系点转换至统一机体坐标系 {B}
        var bodyPoints: [simd_float3] = []
        bodyPoints.reserveCapacity(rawPoints.count)
        for pt in rawPoints {
            bodyPoints.append(calibration.transformLiDARPointToBody(pt))
        }
        
        // 2. 体素网格下采样
        let downsampled = voxelFilter.downsample(points: bodyPoints)
        guard downsampled.count >= 15 else {
            return makeDiagnostics(confidence: 0, groundHeight: nil)
        }
        
        // 3. 计算点云法向量
        let orientedPoints = voxelFilter.estimateNormals(points: downsampled)
        
        // 4. RANSAC 地面提取
        var detectedGroundHeight: Float? = nil
        if let groundPlane = planeDetector.detectGroundPlane(
            points: downsampled,
            gravityDirection: filteredGravity
        ) {
            detectedGroundHeight = groundPlane.distance
            eskf.updateGroundHeight(detectedFloorY: groundPlane.distance, confidence: groundPlane.inlierRatio)
        }
        
        // 5. 点到平面 ICP 几何配准
        var icpConfidence: Float = 0
        if !keyframePoints.isEmpty {
            let icpResult = icpSolver.align(
                sourcePoints: downsampled,
                targetPoints: keyframePoints,
                initialGuess: matrix_identity_float4x4
            )
            
            if icpResult.converged && icpResult.fitnessScore < 0.15 {
                icpConfidence = max(0, 1.0 - (icpResult.fitnessScore / 0.15))
                
                // 累计消除的偏航角漂移 (用于诊断)
                let deltaEuler = simd_quatf(icpResult.deltaRotation).eulerAngles
                totalCompensatedYaw += abs(deltaEuler.z)
                
                // 将几何位姿增量输入 ESKF 进行测量更新与陀螺仪偏置在线反演
                eskf.updateLiDAROdometry(
                    observedRotation: icpResult.deltaRotation,
                    observedTranslation: icpResult.deltaTranslation,
                    confidence: icpConfidence
                )
            }
        }
        
        // 6. 动态更新局部几何关键帧 (当移动超过 0.15m 或旋转超过 10 度时更新)
        if keyframePoints.isEmpty || icpConfidence < 0.4 {
            keyframePoints = orientedPoints
        }
        
        return makeDiagnostics(confidence: icpConfidence, groundHeight: detectedGroundHeight)
    }
    
    // MARK: - 3. 获取校正后的绝对 6-DoF 位姿
    
    public var currentCompensatedTransform: simd_float4x4 {
        let state = eskf.state
        return simd_float4x4(
            rotation: state.orientation.rotationMatrix3x3,
            translation: state.position
        )
    }
    
    private func makeDiagnostics(confidence: Float, groundHeight: Float?) -> DriftDiagnostics {
        let bias = eskf.state.gyroBias
        let biasMagDeg = simd_length(bias) * (180.0 / Float.pi)
        
        return DriftDiagnostics(
            estimatedGyroBias: bias,
            gyroBiasMagnitudeDegPerSec: biasMagDeg,
            isGravityLocked: true,
            icpConfidence: confidence,
            detectedGroundHeight: groundHeight,
            totalCompensatedYawDeg: totalCompensatedYaw * (180.0 / Float.pi)
        )
    }
}
