import Foundation
import simd

/// iPhone Pro 硬件物理外参定义与空间杠杆臂补偿
public struct ExtrinsicCalibration {
    
    /// 机体中心坐标系 (Body Frame {B}): 规定以 iPhone 几何中心主板 IMU 安装位置为原点
    /// X: 指向手机右侧 (扬声器/音量键方向)
    /// Y: 指向手机顶部 (灵动岛方向)
    /// Z: 垂直屏幕指向屏幕正面 (朝向用户)
    
    /// 后置 LiDAR 相对机体 IMU 的外参刚体变换 T_B_LiDAR
    public let T_body_lidar: simd_float4x4
    
    /// 前置 TrueDepth 相对机体 IMU 的外参刚体变换 T_B_TrueDepth
    public let T_body_truedepth: simd_float4x4
    
    /// LiDAR 相对 IMU 的空间位置杠杆臂向量 (米)
    public var leverArmLiDAR: simd_float3 {
        return T_body_lidar.translation3
    }
    
    /// TrueDepth 相对 IMU 的空间位置杠杆臂向量 (米)
    public var leverArmTrueDepth: simd_float3 {
        return T_body_truedepth.translation3
    }
    
    /// 初始化 iPhone Pro 系列标准出厂几何外参
    public init(modelName: String = "iPhone Pro") {
        // 1. 后置 LiDAR 外参：位于手机背部右上区域，光轴指向 -Z (背后)
        // 旋转：绕 Y 轴旋转 180 度 (从朝向用户 Z 到朝向背后 -Z)
        let R_lidar = simd_float3x3(
            simd_float3(-1, 0, 0),
            simd_float3(0, 1, 0),
            simd_float3(0, 0, -1)
        )
        // 物理偏移: X 约 +0.024m (靠右), Y 约 +0.052m (靠上), Z 约 -0.008m (背壳凸台处)
        let t_lidar = simd_float3(0.024, 0.052, -0.008)
        self.T_body_lidar = simd_float4x4(rotation: R_lidar, translation: t_lidar)
        
        // 2. 前置 TrueDepth 外参：位于手机正面顶部灵动岛，光轴指向 +Z (朝向用户)
        let R_truedepth = matrix_identity_float3x3
        // 物理偏移: X 约 0.0m (居中), Y 约 +0.068m (顶部), Z 约 +0.004m (正面玻璃表面)
        let t_truedepth = simd_float3(0.0, 0.068, 0.004)
        self.T_body_truedepth = simd_float4x4(rotation: R_truedepth, translation: t_truedepth)
    }
    
    /// 自定义外参构造函数 (支持后续高精度在线标定)
    public init(T_body_lidar: simd_float4x4, T_body_truedepth: simd_float4x4) {
        self.T_body_lidar = T_body_lidar
        self.T_body_truedepth = T_body_truedepth
    }
    
    // MARK: - 空间坐标系变换
    
    /// 将 LiDAR 测距点转换至机体统一坐标系 {B}
    @inlinable
    public func transformLiDARPointToBody(_ pointInLiDAR: simd_float3) -> simd_float3 {
        let p4 = T_body_lidar * simd_float4(pointInLiDAR, 1.0)
        return simd_float3(p4.x, p4.y, p4.z)
    }
    
    /// 将 TrueDepth 测距点转换至机体统一坐标系 {B}
    @inlinable
    public func transformTrueDepthPointToBody(_ pointInTrueDepth: simd_float3) -> simd_float3 {
        let p4 = T_body_truedepth * simd_float4(pointInTrueDepth, 1.0)
        return simd_float3(p4.x, p4.y, p4.z)
    }
    
    // MARK: - 杠杆臂物理效应动态补偿 (Lever-Arm Kinematics)
    
    /// 补偿由于机身旋转导致的 LiDAR 线速度增益：v_L = v_B + ω × r
    @inlinable
    public func compensateLiDARVelocity(bodyVelocity: simd_float3, angularVelocity: simd_float3) -> simd_float3 {
        let tangentialVelocity = simd_cross(angularVelocity, leverArmLiDAR)
        return bodyVelocity + tangentialVelocity
    }
    
    /// 补偿由于机身角加速度与旋转导致的 LiDAR 线加速度偏差：
    /// a_L = a_B + α × r + ω × (ω × r)
    public func compensateLiDARAcceleration(
        bodyAcceleration: simd_float3,
        angularVelocity: simd_float3,
        angularAcceleration: simd_float3
    ) -> simd_float3 {
        let tangential = simd_cross(angularAcceleration, leverArmLiDAR)
        let centripetal = simd_cross(angularVelocity, simd_cross(angularVelocity, leverArmLiDAR))
        return bodyAcceleration + tangential + centripetal
    }
}
