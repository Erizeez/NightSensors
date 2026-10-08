import Foundation
import simd

/// 3D 几何特征点（包含三维坐标与表面法向量）
public struct OrientedPoint: Sendable {
    public var position: simd_float3
    public var normal: simd_float3
    
    public init(position: simd_float3, normal: simd_float3) {
        self.position = position
        self.normal = normal
    }
}

/// 3D 体素网格点云下采样与法向量估计器
public final class VoxelGridFilter: @unchecked Sendable {
    public let voxelSize: Float
    
    public init(voxelSize: Float = 0.08) { // 默认 8cm 体素网格
        self.voxelSize = voxelSize
    }
    
    /// 将无序点云通过哈希体素网格聚合降采样，提取代表点
    public func downsample(points: [simd_float3]) -> [simd_float3] {
        guard !points.isEmpty else { return [] }
        
        struct VoxelKey: Hashable {
            let x: Int32
            let y: Int32
            let z: Int32
        }
        
        var voxelMap: [VoxelKey: (sum: simd_float3, count: Int)] = [:]
        let invVoxel = 1.0 / voxelSize
        
        for p in points {
            // 过滤无效或超远点 (有效测距范围 0.2m ~ 5.0m)
            let distSq = simd_length_squared(p)
            if distSq < 0.04 || distSq > 25.0 || p.x.isNaN || p.y.isNaN || p.z.isNaN {
                continue
            }
            
            let key = VoxelKey(
                x: Int32(floor(p.x * invVoxel)),
                y: Int32(floor(p.y * invVoxel)),
                z: Int32(floor(p.z * invVoxel))
            )
            
            if let existing = voxelMap[key] {
                voxelMap[key] = (existing.sum + p, existing.count + 1)
            } else {
                voxelMap[key] = (p, 1)
            }
        }
        
        return voxelMap.values.map { $0.sum / Float($0.count) }
    }
    
    /// 对降采样后的点云进行局部表面法向量估计 (基于 k-NN 协方差 PCA)
    public func estimateNormals(points: [simd_float3], kNeighbors: Int = 6) -> [OrientedPoint] {
        guard points.count >= kNeighbors else {
            return points.map { OrientedPoint(position: $0, normal: simd_float3(0, 0, 1)) }
        }
        
        var result: [OrientedPoint] = []
        result.reserveCapacity(points.count)
        
        // 针对点数较适中的体素集合进行局部邻域法向量估算
        for i in 0..<points.count {
            let pi = points[i]
            
            // 寻找距离 pi 最近的 k 个邻近点
            var neighbors: [(distSq: Float, pt: simd_float3)] = []
            for j in 0..<points.count {
                if i == j { continue }
                let dSq = simd_length_squared(pi - points[j])
                if dSq < (voxelSize * 3.0) * (voxelSize * 3.0) {
                    neighbors.append((dSq, points[j]))
                }
            }
            
            if neighbors.count < 3 {
                // 邻近点不足，默认法向量朝向视线反方向
                let defaultNormal = -simd_normalize(pi)
                result.append(OrientedPoint(position: pi, normal: defaultNormal))
                continue
            }
            
            // 排序选取最近的 k 个点
            neighbors.sort { $0.distSq < $1.distSq }
            let kPts = Array(neighbors.prefix(kNeighbors).map { $0.pt })
            
            // 计算质心
            var centroid = pi
            for pt in kPts { centroid += pt }
            centroid /= Float(kPts.count + 1)
            
            // 计算 3x3 协方差矩阵 C = sum((p - c) * (p - c)^T)
            var cov = matrix_identity_float3x3 * 0
            let allPts = [pi] + kPts
            for p in allPts {
                let diff = p - centroid
                cov.columns.0 += diff * diff.x
                cov.columns.1 += diff * diff.y
                cov.columns.2 += diff * diff.z
            }
            
            // 采用幂迭代法/反幂法求解最小特征值对应的特征向量作为法向量
            let normal = estimateSmallestEigenvector(cov)
            
            // 统一法向量朝向（朝向传感器原点）
            let orientedNormal = simd_dot(normal, -pi) > 0 ? normal : -normal
            result.append(OrientedPoint(position: pi, normal: orientedNormal))
        }
        
        return result
    }
    
    /// 计算 3x3 矩阵最小特征值对应的特征向量
    private func estimateSmallestEigenvector(_ mat: simd_float3x3) -> simd_float3 {
        // 利用平移反幂迭代或解析法估算最小特征向量
        // 初始猜测向量
        var v = simd_normalize(simd_float3(1.0, 1.0, 1.0))
        let maxIter = 5
        let reg = mat + matrix_identity_float3x3 * 1e-4
        
        for _ in 0..<maxIter {
            // 计算伴随矩阵或初等高斯解法近似反演
            let det = simd_determinant(reg)
            if abs(det) < 1e-7 { break }
            let inv = simd_inverse(reg)
            let nextV = inv * v
            let norm = simd_length(nextV)
            if norm < 1e-7 { break }
            v = nextV / norm
        }
        
        return simd_length(v) > 0.5 ? simd_normalize(v) : simd_float3(0, 0, 1)
    }
}
