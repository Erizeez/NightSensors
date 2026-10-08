import Foundation
import simd

/// 点到平面 ICP 配准结果
public struct ICPResult: Sendable {
    public let deltaRotation: simd_float3x3   // 相对旋转矩阵
    public let deltaTranslation: simd_float3  // 相对平移向量 (米)
    public let fitnessScore: Float           // 平均配准残差 (米)
    public let converged: Bool               // 是否收敛
    public let iterations: Int               // 实际迭代次数
    
    public var deltaTransform: simd_float4x4 {
        return simd_float4x4(rotation: deltaRotation, translation: deltaTranslation)
    }
}

/// 高性能纯点到平面 ICP 几何配准引擎
public final class PointToPlaneICP: @unchecked Sendable {
    public let maxIterations: Int
    public let maxCorrespondenceDistance: Float
    public let convergenceThreshold: Float
    
    public init(
        maxIterations: Int = 8,
        maxCorrespondenceDistance: Float = 0.35, // 35cm 内寻找对应法向点
        convergenceThreshold: Float = 1e-4
    ) {
        self.maxIterations = maxIterations
        self.maxCorrespondenceDistance = maxCorrespondenceDistance
        self.convergenceThreshold = convergenceThreshold
    }
    
    /// 执行点到平面配准
    /// - Parameters:
    ///   - sourcePoints: 当前帧点云 P_curr
    ///   - targetPoints: 上一帧或局部地图带法向点云 Q_target
    ///   - initialGuess: IMU 惯导给出的先验相对变换 (R_prior, t_prior)
    public func align(
        sourcePoints: [simd_float3],
        targetPoints: [OrientedPoint],
        initialGuess: simd_float4x4 = matrix_identity_float4x4
    ) -> ICPResult {
        guard !sourcePoints.isEmpty && targetPoints.count >= 10 else {
            return ICPResult(
                deltaRotation: initialGuess.rotation3x3,
                deltaTranslation: initialGuess.translation3,
                fitnessScore: Float.infinity,
                converged: false,
                iterations: 0
            )
        }
        
        var currentR = initialGuess.rotation3x3
        var currentT = initialGuess.translation3
        var lastError: Float = Float.greatestFiniteMagnitude
        var actualIterations = 0
        var isConverged = false
        
        let maxDistSq = maxCorrespondenceDistance * maxCorrespondenceDistance
        
        for iter in 0..<maxIterations {
            actualIterations += 1
            
            var H = Matrix6x6()
            var b = [Float](repeating: 0, count: 6)
            var totalResidual: Float = 0
            var matchedCount = 0
            
            // 遍历源点云寻找目标点云中的最近邻
            for sp in sourcePoints {
                // 将源点由当前估计位姿转换至目标系
                let pTransformed = currentR * sp + currentT
                
                // 线性查找距离最近的目标点 (在体素下采样后的几百个点中极快，<1ms)
                var bestDistSq = maxDistSq
                var bestTarget: OrientedPoint? = nil
                
                for tp in targetPoints {
                    let dSq = simd_length_squared(pTransformed - tp.position)
                    if dSq < bestDistSq {
                        bestDistSq = dSq
                        bestTarget = tp
                    }
                }
                
                guard let target = bestTarget else { continue }
                
                // 计算点到平面残差: e = (pTransformed - q) • n
                let diff = pTransformed - target.position
                let residual = simd_dot(diff, target.normal)
                
                // 鲁棒核函数 Hubber Loss 截断 (抑制动态物体粗差)
                let weight: Float = abs(residual) < 0.05 ? 1.0 : (0.05 / max(1e-4, abs(residual)))
                let weightedResidual = residual * weight
                
                // 计算雅可比 J = [(pTransformed x n)^T,  n^T]
                let cross = simd_cross(pTransformed, target.normal)
                let J0 = cross.x
                let J1 = cross.y
                let J2 = cross.z
                let J3 = target.normal.x
                let J4 = target.normal.y
                let J5 = target.normal.z
                
                let JTuple = (J0, J1, J2, J3, J4, J5)
                H.accumulateOuterProduct(JTuple)
                
                // 右端项 b = - J^T * e
                b[0] -= J0 * weightedResidual
                b[1] -= J1 * weightedResidual
                b[2] -= J2 * weightedResidual
                b[3] -= J3 * weightedResidual
                b[4] -= J4 * weightedResidual
                b[5] -= J5 * weightedResidual
                
                totalResidual += abs(residual)
                matchedCount += 1
            }
            
            guard matchedCount >= 10 else { break }
            let meanResidual = totalResidual / Float(matchedCount)
            
            // 求解正规方程 H * delta_xi = b
            guard let deltaXi = H.solveCholesky(b: b, damping: 1e-3) else {
                break
            }
            
            let deltaRotVec = simd_float3(deltaXi[0], deltaXi[1], deltaXi[2])
            let deltaTrans = simd_float3(deltaXi[3], deltaXi[4], deltaXi[5])
            
            // 李群更新: R <- exp([δϕ]_x) * R,  t <- exp([δϕ]_x) * t + δt
            let dR = deltaRotVec.expSO3()
            currentR = dR * currentR
            currentT = dR * currentT + deltaTrans
            
            let stepNorm = simd_length(deltaRotVec) + simd_length(deltaTrans)
            if stepNorm < convergenceThreshold || abs(lastError - meanResidual) < 1e-5 {
                isConverged = true
                lastError = meanResidual
                break
            }
            lastError = meanResidual
        }
        
        return ICPResult(
            deltaRotation: currentR,
            deltaTranslation: currentT,
            fitnessScore: lastError,
            converged: isConverged,
            iterations: actualIterations
        )
    }
}
