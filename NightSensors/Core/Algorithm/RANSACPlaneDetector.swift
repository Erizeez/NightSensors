import Foundation
import simd

/// 拟合出的几何平面结构
public struct DetectedPlane: Sendable {
    public let normal: simd_float3 // 平面单位法向量 (指向天花板方向，或设备前方)
    public let distance: Float    // 平面到坐标原点的垂直有向距离 (米)
    public let inlierCount: Int   // 支持该平面的内点数量
    public let inlierRatio: Float // 内点比例
    public let isFloorPlane: Bool // 是否被判定为地面平面
    
    public init(normal: simd_float3, distance: Float, inlierCount: Int, inlierRatio: Float, isFloorPlane: Bool) {
        self.normal = normal
        self.distance = distance
        self.inlierCount = inlierCount
        self.inlierRatio = inlierRatio
        self.isFloorPlane = isFloorPlane
    }
    
    /// 计算某一点到该平面的有向垂直距离
    public func signedDistance(to point: simd_float3) -> Float {
        return simd_dot(normal, point) + distance
    }
}

/// 基于 RANSAC 的无光环境主平面与地面鲁棒探测器
public final class RANSACPlaneDetector: @unchecked Sendable {
    public let maxIterations: Int
    public let distanceThreshold: Float // 内点判定距离阈值 (例如 3cm)
    public let minInlierRatio: Float
    
    public init(
        maxIterations: Int = 40,
        distanceThreshold: Float = 0.03,
        minInlierRatio: Float = 0.25
    ) {
        self.maxIterations = maxIterations
        self.distanceThreshold = distanceThreshold
        self.minInlierRatio = minInlierRatio
    }
    
    /// 从点云中提取主平面，并结合重力矢量锁定地面
    /// - Parameters:
    ///   - points: 3D 点云
    ///   - gravityDirection: 当前加速度计测量的机体坐标系下的重力方向 (通常指向下方)
    public func detectGroundPlane(
        points: [simd_float3],
        gravityDirection: simd_float3
    ) -> DetectedPlane? {
        guard points.count >= 20 else { return nil }
        
        let nPoints = points.count
        var bestInliers = 0
        var bestNormal = simd_float3(0, 1, 0)
        var bestDistance: Float = 0
        
        let gNorm = simd_normalize(gravityDirection)
        
        for _ in 0..<maxIterations {
            // 随机采样 3 个不共线的点
            let idx1 = Int.random(in: 0..<nPoints)
            var idx2 = Int.random(in: 0..<nPoints)
            while idx2 == idx1 { idx2 = Int.random(in: 0..<nPoints) }
            var idx3 = Int.random(in: 0..<nPoints)
            while idx3 == idx1 || idx3 == idx2 { idx3 = Int.random(in: 0..<nPoints) }
            
            let p1 = points[idx1]
            let p2 = points[idx2]
            let p3 = points[idx3]
            
            let v12 = p2 - p1
            let v13 = p3 - p1
            let crossProd = simd_cross(v12, v13)
            let crossLen = simd_length(crossProd)
            if crossLen < 1e-5 { continue } // 三点共线
            
            var normal = crossProd / crossLen
            var d = -simd_dot(normal, p1)
            
            // 检查法向量是否与重力方向平行 (夹角在 20 度以内，判定为可能的地面或天花板)
            let alignmentWithG = abs(simd_dot(normal, gNorm))
            let isHorizontal = alignmentWithG > 0.94 // cos(20°) ≈ 0.939
            if !isHorizontal { continue }
            
            // 确保法向量与重力反向 (垂直向上)
            if simd_dot(normal, gNorm) > 0 {
                normal = -normal
                d = -d
            }
            
            // 统计内点
            var inliers = 0
            for p in points {
                let dist = abs(simd_dot(normal, p) + d)
                if dist < distanceThreshold {
                    inliers += 1
                }
            }
            
            if inliers > bestInliers {
                bestInliers = inliers
                bestNormal = normal
                bestDistance = d
            }
        }
        
        let ratio = Float(bestInliers) / Float(nPoints)
        guard ratio >= minInlierRatio && bestInliers >= 15 else {
            return nil
        }
        
        return DetectedPlane(
            normal: bestNormal,
            distance: bestDistance,
            inlierCount: bestInliers,
            inlierRatio: ratio,
            isFloorPlane: true
        )
    }
}
